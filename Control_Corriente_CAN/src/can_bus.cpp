/*
 * can_bus.cpp
 * ─────────────────────────────────────────────────────────────
 * Driver MCP2515 (SPI manual, misma "forma de transacción" que el
 * nodo TX de referencia) + recepción por interrupción + lógica de
 * gateway del agente 1. Corre íntegramente en Core 0 (taskCAN).
 *
 * SPI: VSPI, 500 kbps @ cristal 8 MHz.
 * Solo esta tarea usa el bus SPI → no requiere mutex de SPI.
 * ─────────────────────────────────────────────────────────────
 */

#include "can_bus.h"
#include "shared.h"

#include <SPI.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

/* ============================================================
   INSTRUCCIONES SPI / REGISTROS / MODOS  (idénticos a referencia)
============================================================ */
#define INSTR_RESET   0xC0
#define INSTR_READ    0x03
#define INSTR_WRITE   0x02
#define INSTR_BITMOD  0x05
#define INSTR_RTS_TX0 0x81

#define REG_CANSTAT   0x0E
#define REG_CANCTRL   0x0F
#define REG_TEC       0x1C
#define REG_CNF3      0x28
#define REG_CNF2      0x29
#define REG_CNF1      0x2A
#define REG_CANINTE   0x2B
#define REG_CANINTF   0x2C
#define REG_EFLG      0x2D
#define REG_RXB0CTRL  0x60
#define REG_RXB0SIDH  0x61
#define REG_RXB0SIDL  0x62
#define REG_RXB0DLC   0x65
#define REG_RXB0D0    0x66
#define REG_RXB1CTRL  0x70
#define REG_RXB1SIDH  0x71
#define REG_RXB1SIDL  0x72
#define REG_RXB1DLC   0x75
#define REG_RXB1D0    0x76
#define REG_TXB0CTRL  0x30
#define REG_TXB0SIDH  0x31
#define REG_TXB0SIDL  0x32
#define REG_TXB0DLC   0x35
#define REG_TXB0D0    0x36

/* Flags CANINTF */
#define INTF_RX0IF    0x01
#define INTF_RX1IF    0x02
#define INTF_TX0IF    0x04
/* Habilitación CANINTE */
#define INTE_RX0IE    0x01
#define INTE_RX1IE    0x02

#define MODE_NORMAL   0x00
#define MODE_CONFIG   0x80
#define MODE_MASK     0xE0

/* ============================================================
   struct compatible con autowp
============================================================ */
struct can_frame {
  uint32_t can_id;
  uint8_t  can_dlc;
  uint8_t  data[8];
};

/* ============================================================
   ESTADO INTERNO
============================================================ */
static SPIClass    spi(VSPI);
static SPISettings spiCfg(CAN_SPI_HZ, MSBFIRST, SPI_MODE0);

/* Semáforo señalizado desde la ISR del pin INT */
static SemaphoreHandle_t canIntSem = nullptr;

/* Datos recibidos de los agentes remotos (índice 0 → agente2, 1 → agente3) */
static CanAgentData      remote[2];
static SemaphoreHandle_t canMutex = nullptr;

/* Cola de configuraciones a transmitir (comms → CAN) */
struct CanConfigItem { uint8_t agent; uint8_t idx; float val; };
static QueueHandle_t canCfgQueue = nullptr;

static volatile bool canReady = false;

/* ============================================================
   ISR pin INT (solo señaliza; el SPI se hace en la tarea)
============================================================ */
static void IRAM_ATTR canIsr() {
  BaseType_t hpw = pdFALSE;
  if (canIntSem) xSemaphoreGiveFromISR(canIntSem, &hpw);
  if (hpw) portYIELD_FROM_ISR();
}

/* ============================================================
   DRIVER MCP2515  (misma forma de transacción que el TX de ref.)
============================================================ */
static void csLow()  { digitalWrite(PIN_CAN_CS, LOW); }
static void csHigh() { digitalWrite(PIN_CAN_CS, HIGH); }

static void mcpReset() {
  spi.beginTransaction(spiCfg);
  csLow();  spi.transfer(INSTR_RESET);  csHigh();
  spi.endTransaction();
  delay(20);
}

static uint8_t mcpRead(uint8_t reg) {
  spi.beginTransaction(spiCfg);
  csLow();
  spi.transfer(INSTR_READ);
  spi.transfer(reg);
  uint8_t v = spi.transfer(0x00);
  csHigh();
  spi.endTransaction();
  return v;
}

static void mcpWrite(uint8_t reg, uint8_t val) {
  spi.beginTransaction(spiCfg);
  csLow();
  spi.transfer(INSTR_WRITE);
  spi.transfer(reg);
  spi.transfer(val);
  csHigh();
  spi.endTransaction();
}

static void mcpBitMod(uint8_t reg, uint8_t mask, uint8_t val) {
  spi.beginTransaction(spiCfg);
  csLow();
  spi.transfer(INSTR_BITMOD);
  spi.transfer(reg);
  spi.transfer(mask);
  spi.transfer(val);
  csHigh();
  spi.endTransaction();
}

static void mcpRTS_TX0() {
  spi.beginTransaction(spiCfg);
  csLow();  spi.transfer(INSTR_RTS_TX0);  csHigh();
  spi.endTransaction();
}

static bool setMode(uint8_t mode, const char* nombre) {
  mcpBitMod(REG_CANCTRL, MODE_MASK, mode);
  for (int i = 0; i < 50; i++) {
    uint8_t s = mcpRead(REG_CANSTAT);
    if ((s & MODE_MASK) == mode) {
      Serial.print("[CAN][OK] modo ");  Serial.println(nombre);
      return true;
    }
    delay(2);
  }
  Serial.print("[CAN][FAIL] modo ");  Serial.println(nombre);
  return false;
}

static bool setBitrate500k_8MHz() {
  mcpWrite(REG_CNF1, 0x00);
  mcpWrite(REG_CNF2, 0x90);
  mcpWrite(REG_CNF3, 0x02);
  if (mcpRead(REG_CNF1) != 0x00) { Serial.println("[CAN][FAIL] CNF1"); return false; }
  if (mcpRead(REG_CNF2) != 0x90) { Serial.println("[CAN][FAIL] CNF2"); return false; }
  if (mcpRead(REG_CNF3) != 0x02) { Serial.println("[CAN][FAIL] CNF3"); return false; }
  Serial.println("[CAN][OK] bitrate 500kbps@8MHz");
  return true;
}

/* Declaraciones adelantadas: sendMessage drena RX mientras espera la
   confirmación de TX, así no se pierden frames durante la propia ráfaga. */
static bool readMessage(can_frame* f);
static void handleRxFrame(const can_frame* f);

/* TX por TXB0 (igual que el nodo de referencia) */
static bool sendMessage(const can_frame* f) {
  uint8_t txctrl = mcpRead(REG_TXB0CTRL);
  if (txctrl & 0x08) {                 // TXREQ aún activo → abortar
    mcpBitMod(REG_TXB0CTRL, 0x08, 0x00);
  }

  uint16_t sid = f->can_id & 0x7FF;
  mcpWrite(REG_TXB0SIDH, (sid >> 3) & 0xFF);
  mcpWrite(REG_TXB0SIDL, (sid & 0x07) << 5);

  uint8_t dlc = f->can_dlc;
  if (dlc > 8) dlc = 8;
  mcpWrite(REG_TXB0DLC, dlc);

  for (uint8_t i = 0; i < dlc; i++)
    mcpWrite(REG_TXB0D0 + i, f->data[i]);

  mcpBitMod(REG_CANINTF, INTF_TX0IF, 0x00);
  mcpRTS_TX0();

  /* Espera SIN delay: mientras no se confirme la TX, drenamos RX (sondeo). */
  uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < 30000UL) {   // timeout 30 ms
    uint8_t intf = mcpRead(REG_CANINTF);
    if (intf & INTF_TX0IF) {
      mcpBitMod(REG_CANINTF, INTF_TX0IF, 0x00);
      return true;
    }
    can_frame rf;
    if (readMessage(&rf)) handleRxFrame(&rf);   // tiempo de espera productivo
  }

  mcpBitMod(REG_TXB0CTRL, 0x08, 0x00);
  return false;
}

/* RX: lee un frame pendiente de RXB0/RXB1. Retorna false si no hay. */
static bool readMessage(can_frame* f) {
  uint8_t intf = mcpRead(REG_CANINTF);

  uint8_t sidh_reg, sidl_reg, dlc_reg, d0_reg, clearFlag;
  if (intf & INTF_RX0IF) {
    sidh_reg = REG_RXB0SIDH; sidl_reg = REG_RXB0SIDL;
    dlc_reg  = REG_RXB0DLC;  d0_reg   = REG_RXB0D0;  clearFlag = INTF_RX0IF;
  } else if (intf & INTF_RX1IF) {
    sidh_reg = REG_RXB1SIDH; sidl_reg = REG_RXB1SIDL;
    dlc_reg  = REG_RXB1DLC;  d0_reg   = REG_RXB1D0;  clearFlag = INTF_RX1IF;
  } else {
    return false;
  }

  uint8_t sidh = mcpRead(sidh_reg);
  uint8_t sidl = mcpRead(sidl_reg);
  f->can_id = ((uint16_t)sidh << 3) | (sidl >> 5);   // ID estándar 11 bits

  uint8_t dlc = mcpRead(dlc_reg) & 0x0F;
  if (dlc > 8) dlc = 8;
  f->can_dlc = dlc;
  for (uint8_t i = 0; i < dlc; i++)
    f->data[i] = mcpRead(d0_reg + i);

  mcpBitMod(REG_CANINTF, clearFlag, 0x00);
  return true;
}

/* ============================================================
   HELPERS de payload
============================================================ */
static float bytesToFloat(const uint8_t* d) {
  float v; memcpy(&v, d, 4); return v;
}

static bool txFloat(uint16_t id, float v) {
  can_frame f;
  f.can_id  = id;
  f.can_dlc = 4;
  memcpy(f.data, &v, 4);
  return sendMessage(&f);
}

static bool txFaultFrame(uint8_t code) {
  can_frame f;
  f.can_id  = canMakeId(CAN_CLASS_FAULT, THIS_AGENT_ID, code);
  f.can_dlc = 1;
  f.data[0] = code;
  return sendMessage(&f);
}

/* ============================================================
   API: datos remotos (thread-safe)
============================================================ */
bool canGetAgentData(uint8_t agent, CanAgentData* out) {
  if (agent < 2 || agent > 3 || !out) return false;
  int idx = agent - 2;
  if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
    /* recalcular online según timeout */
    if (remote[idx].online &&
        (millis() - remote[idx].lastRxMs > CAN_AGENT_TIMEOUT_MS))
      remote[idx].online = false;
    *out = remote[idx];
    xSemaphoreGive(canMutex);
    return true;
  }
  return false;
}

/* ============================================================
   API: encolar configuración (comms → CAN)
============================================================ */
bool canQueueConfig(uint8_t destAgent, uint8_t paramIdx, float value) {
  if (!canCfgQueue || destAgent < 2 || destAgent > 3 || paramIdx > 0xF)
    return false;
  CanConfigItem it{ destAgent, paramIdx, value };
  return xQueueSend(canCfgQueue, &it, 0) == pdTRUE;
}

/* ============================================================
   HELPERS DE DIAGNÓSTICO (nombres legibles)
============================================================ */
#if CAN_DEBUG
static const char* canClassName(uint8_t cls) {
  switch (cls) {
    case CAN_CLASS_FAULT:  return "FAULT";
    case CAN_CLASS_HIGH:   return "HIGH";
    case CAN_CLASS_LOW:    return "LOW";
    case CAN_CLASS_CONFIG: return "CONFIG";
    default:               return "?";
  }
}
static const char* cfgName(uint8_t idx) {
  switch (idx) {
    case CFG_TARGET_V:  return "target_v";
    case CFG_OMEGA_V:   return "omega_v";
    case CFG_OMEGA_I:   return "omega_i";
    case CFG_PLANT_VDC: return "plant_vdc";
    case CFG_KD_DROOP:  return "kd_droop";
    case CFG_IREF_MAX:  return "i_ref_max";
    case CFG_IREF_MIN:  return "i_ref_min";
    case CFG_OV_TRIP:   return "ov_trip";
    case CFG_UV_TRIP:   return "uv_trip";
    case CFG_OC_TRIP:   return "oc_trip";
    case CFG_RUN_MODE:  return "run_mode";
    case CFG_FAULT_ACK: return "fault_ack";
    default:            return "?";
  }
}
#endif

/* ============================================================
   Procesar un frame recibido → actualizar `remote[]`
============================================================ */
static void handleRxFrame(const can_frame* f) {
  uint16_t id    = f->can_id & 0x7FF;
  uint8_t  cls   = canIdClass(id);
  uint8_t  agent = canIdAgent(id);
  uint8_t  var   = canIdVar(id);

#if CAN_DEBUG
  /* Imprime TODO lo que llega al bus → confirma el enlace y correlaciona
     con el monitor del agente 2. */
  if (f->can_dlc >= 4)
    Serial.printf("[CAN][RX] id=0x%03X %-6s a%u v%u dlc=%u = %.4f\n",
                  id, canClassName(cls), agent, var, f->can_dlc, bytesToFloat(f->data));
  else
    Serial.printf("[CAN][RX] id=0x%03X %-6s a%u v%u dlc=%u d0=%u\n",
                  id, canClassName(cls), agent, var, f->can_dlc,
                  f->can_dlc ? f->data[0] : 0);
#endif

  /* Solo nos interesan agentes remotos 2 y 3 */
  if (agent < 2 || agent > 3) return;
  int idx = agent - 2;

  if (xSemaphoreTake(canMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;

  remote[idx].online   = true;
  remote[idx].lastRxMs = millis();

  switch (cls) {
    case CAN_CLASS_HIGH:
      if (f->can_dlc >= 4 && var == CAN_VAR_IOUT)
        remote[idx].i_out = bytesToFloat(f->data);
      break;
    case CAN_CLASS_LOW:
      if (f->can_dlc >= 4) {
        float v = bytesToFloat(f->data);
        if      (var == CAN_VAR_SOC)  remote[idx].soc   = v;
        else if (var == CAN_VAR_VBAT) remote[idx].v_bat = v;
        else if (var == CAN_VAR_VBUS) remote[idx].v_bus = v;
      }
      break;
    case CAN_CLASS_FAULT: {
      uint8_t c = (f->can_dlc >= 1) ? f->data[0] : var;
      if (c == CAN_FAULT_GENERIC) {        // código 0 = fault despejado (clear)
        remote[idx].fault     = false;
        remote[idx].faultCode = 0;
      } else {
        remote[idx].fault     = true;
        remote[idx].faultCode = c;
      }
      break;
    }
    default:
      break;   // CONFIG u otras clases: ignorar (no son para nosotros)
  }

  xSemaphoreGive(canMutex);
}

/* ============================================================
   TX: telemetría y faults propios del agente 1
============================================================ */
#if CAN_TX_OWN_TELEMETRY
static void txOwnTelemetry() {
  Snapshot s = takeSnapshot();
  bool ok __attribute__((unused)) = true;
  /* Solo i_out en alta frecuencia; v_bat/SoC son lentos. La potencia se
     calcula en la web (i_out · v_bus). El gateway no difunde su v_bus por
     CAN: ya va a Firebase y es la referencia del bus. */
  ok &= txFloat(canMakeId(CAN_CLASS_HIGH, THIS_AGENT_ID, CAN_VAR_IOUT), s.i_out);
  ok &= txFloat(canMakeId(CAN_CLASS_LOW,  THIS_AGENT_ID, CAN_VAR_VBAT), s.v_bat);
  ok &= txFloat(canMakeId(CAN_CLASS_LOW,  THIS_AGENT_ID, CAN_VAR_SOC),  s.soc);
#if CAN_DEBUG
  /* [OK] = ACKeado por el agente 2; [FAIL] = sin ACK (cableado/terminación/GND). */
  Serial.printf("[CAN][TX] a%u i_out=%.3f v_bat=%.3f soc=%.3f  [%s]\n",
                THIS_AGENT_ID, s.i_out, s.v_bat, s.soc, ok ? "OK" : "FAIL");
#endif
}
#endif

/* ============================================================
   Drenar y transmitir configuraciones encoladas
============================================================ */
static void drainConfigQueue() {
#if CAN_RELAY_CONFIG
  CanConfigItem it;
  while (canCfgQueue && xQueueReceive(canCfgQueue, &it, 0) == pdTRUE) {
    uint16_t id = canMakeId(CAN_CLASS_CONFIG, it.agent, it.idx);
    bool ok __attribute__((unused)) = txFloat(id, it.val);
#if CAN_DEBUG
    Serial.printf("[CAN][CFG->a%u] %s (idx=%u) = %.4f  [%s]\n",
                  it.agent, cfgName(it.idx), it.idx, it.val, ok ? "OK" : "FAIL");
#endif
  }
#endif
}

/* ============================================================
   INIT del MCP2515 (dentro de la tarea, como sensorsInit)
============================================================ */
static bool canHwInit() {
  pinMode(PIN_CAN_CS, OUTPUT);
  csHigh();
  spi.begin(PIN_CAN_SCK, PIN_CAN_MISO, PIN_CAN_MOSI, PIN_CAN_CS);
  delay(50);   // margen para que arranque el oscilador del MCP2515

  /* Reset + verificación de modo CONFIG, con reintentos por si el módulo
     arranca lento o hubo un transitorio de alimentación. CANSTAT=0x00 de
     forma persistente ⇒ el MCP2515 no responde por SPI (problema de HW). */
  uint8_t s = 0;
  bool inConfig = false;
  for (int attempt = 1; attempt <= 5; attempt++) {
    mcpReset();
    delay(10);
    s = mcpRead(REG_CANSTAT);
    Serial.printf("[CAN] intento %d/5: CANSTAT=0x%02X\n", attempt, s);
    if ((s & MODE_MASK) == MODE_CONFIG) { inConfig = true; break; }
    delay(50);
  }
  if (!inConfig) {
    Serial.println("[CAN] ABORT: MCP2515 no responde por SPI.");
    Serial.println("           Revisa: alimentación del módulo, MISO(19)/MOSI(23)/SCK(18)/CS(5), cristal y GND común.");
    return false;
  }

  if (!setBitrate500k_8MHz()) return false;

  /* RX abiertos a cualquier ID (sin máscara/filtro).
     RXB0CTRL=0x64 → BUKT=1: si RXB0 se llena, desborda a RXB1 (FIFO de 2). */
  mcpWrite(REG_RXB0CTRL, 0x64);
  mcpWrite(REG_RXB1CTRL, 0x60);

  /* Habilitar interrupción de recepción (RX0IE | RX1IE) */
  mcpWrite(REG_CANINTE, INTE_RX0IE | INTE_RX1IE);
  Serial.println("[CAN][OK] RX + INT habilitados");

  if (!setMode(MODE_NORMAL, "NORMAL")) return false;
  return true;
}

/* ============================================================
   TAREA FREERTOS — Core 0
============================================================ */
void taskCAN(void* pvParams) {
  canMutex    = xSemaphoreCreateMutex();
  canIntSem   = xSemaphoreCreateBinary();
  canCfgQueue = xQueueCreate(16, sizeof(CanConfigItem));
  memset(remote, 0, sizeof(remote));

  if (!canHwInit()) {
    Serial.println("[CAN] Inicialización fallida. Tarea CAN detenida.");
    vTaskDelete(nullptr);
    return;
  }

  /* INT del MCP2515: activo bajo, open-drain → pull-up interno + FALLING */
  pinMode(PIN_CAN_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_CAN_INT), canIsr, FALLING);
  canReady = true;
  Serial.println("[CAN] Operativo (agente 1 gateway).");

  uint32_t lastTxMs = millis();
  bool     prevFault = false;

  /* Drenar cualquier frame que ya estuviera pendiente al arrancar */
  { can_frame f; while (readMessage(&f)) handleRxFrame(&f); }

  for (;;) {
    bool busy = false;

    /* ── RX 100% por sondeo: drenar todo lo pendiente cada iteración ── */
    can_frame f;
    while (readMessage(&f)) { handleRxFrame(&f); busy = true; }

    /* ── Config encolada por la web (comms) ── */
    drainConfigQueue();

    /* ── Fault propio: emitir en el flanco de subida ── */
#if CAN_TX_OWN_FAULTS
    bool    fault = false;
    uint8_t code  = CAN_FAULT_GENERIC;
    if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      fault = faultLatched;
      code  = faultCode;
      xSemaphoreGive(xMutex);
    }
    if (fault && !prevFault) {
      bool ok __attribute__((unused)) = txFaultFrame(code);
#if CAN_DEBUG
      Serial.printf("[CAN][TX] FAULT code=%u  [%s]\n", code, ok ? "OK" : "FAIL");
#endif
    }
    prevFault = fault;
#endif

    /* ── Telemetría propia periódica ── */
#if CAN_TX_OWN_TELEMETRY
    if (millis() - lastTxMs >= CAN_TX_PERIOD_MS) {
      lastTxMs = millis();
      txOwnTelemetry();
      busy = true;
    }
#endif

    /* Si hubo actividad, seguir sondeando sin ceder (no perder la ráfaga).
       Si el bus está inactivo, ceder hasta 1 ms (la INT lo despierta antes
       si llega un frame) para no matar el watchdog ni a taskComms (WiFi). */
    if (!busy) xSemaphoreTake(canIntSem, pdMS_TO_TICKS(1));
  }
}
