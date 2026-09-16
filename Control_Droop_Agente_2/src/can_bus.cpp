/*
 * can_bus.cpp  — AGENTE 2 (nodo CAN)
 * ─────────────────────────────────────────────────────────────
 * Driver MCP2515 (SPI manual, misma forma de transacción que el
 * gateway del agente 1) + recepción por interrupción.
 * Corre íntegramente en Core 0 (taskCAN).
 *
 * Diferencias con el gateway:
 *   - RX: solo procesa frames CONFIG dirigidos a THIS_AGENT_ID y los
 *     aplica a las variables compartidas (con los mismos clamps que
 *     readParams usaba desde Firebase).
 *   - TX: difunde su telemetría y faults (con frame de "clear" al
 *     despejar el fault).
 *   - Sin tabla de agentes remotos ni cola de relay.
 *
 * SPI: VSPI, 500 kbps @ cristal 8 MHz.
 * ─────────────────────────────────────────────────────────────
 */

#include "can_bus.h"
#include "shared.h"
#include "control.h"

#include <SPI.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/* ============================================================
   INSTRUCCIONES SPI / REGISTROS / MODOS
============================================================ */
#define INSTR_RESET   0xC0
#define INSTR_READ    0x03
#define INSTR_WRITE   0x02
#define INSTR_BITMOD  0x05
#define INSTR_RTS_TX0 0x81

#define REG_CANSTAT   0x0E
#define REG_CANCTRL   0x0F
#define REG_CNF3      0x28
#define REG_CNF2      0x29
#define REG_CNF1      0x2A
#define REG_CANINTE   0x2B
#define REG_CANINTF   0x2C
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
   DRIVER MCP2515
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

/* TX por TXB0 */
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

  /* Espera de confirmación de TX. Los primeros ~2 ms se sondea sin ceder
     (drenando RX) para capturar el ACK normal (<1 ms). Si la TX se atasca
     —p. ej. ningún otro nodo en el bus → sin ACK— se cede 1 ms por vuelta
     para no starve el watchdog (IDLE0) ni a otras tareas. */
  uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < 30000UL) {   // timeout 30 ms
    uint8_t intf = mcpRead(REG_CANINTF);
    if (intf & INTF_TX0IF) {
      mcpBitMod(REG_CANINTF, INTF_TX0IF, 0x00);
      return true;
    }
    can_frame rf;
    if (readMessage(&rf)) handleRxFrame(&rf);   // tiempo de espera productivo
    if ((uint32_t)(micros() - t0) > 2000UL) vTaskDelay(1);
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
   CONSENSO DISTRIBUIDO DEL VOLTAJE DE BUS (opción B)
   Estima el promedio de las mediciones intercambiando la estimación por CAN
   (CAN_VAR_VCONS). Corre en taskCAN, lento respecto al lazo primario.
============================================================ */
static float    consEst[4] = {0, 0, 0, 0};
static uint32_t consMs[4]  = {0, 0, 0, 0};
static bool     consInit   = false;
static uint32_t lastConsMs = 0;

/* ── Seguridad de conexión al bus: v_bus real de cada par (solo lo difunden con
   control ON). Comparamos el propio contra estos para saber si estamos en el
   mismo bus físico. ── */
static float    busVpeer[4]   = {0, 0, 0, 0};
static uint32_t busVpeerMs[4] = {0, 0, 0, 0};
static uint32_t busMismatchSince = 0;
static uint32_t lastBusChkMs     = 0;

/* ¿El ADS local está sano? Si está desconectado (hwStatus=HW_NO_ADS) la
   medición del bus queda congelada/errónea: NO difundimos telemetría ni
   consenso para no contaminar el promedio de la red. El FAULT sí se sigue
   emitiendo (así el gateway y el dashboard saben del sensor caído). */
static bool localSensorsHealthy() {
  HwStatus st = HW_OK;
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    st = hwStatus;
    xSemaphoreGive(xMutex);
  }
  return st == HW_OK;
}

static bool consNeighborFresh() {
  uint32_t now = millis();
  for (uint8_t a = 1; a <= 3; a++)
    if (a != THIS_AGENT_ID && consMs[a] != 0 && (now - consMs[a]) < CONS_TIMEOUT_MS) return true;
  return false;
}

static void runConsensus(float dt) {
  float u;
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) != pdTRUE) return;
  u = v_bus;                              // medición local (u_i)
  xSemaphoreGive(xMutex);
  if (u <= 0.01f) return;

  if (!consInit) { consEst[THIS_AGENT_ID] = u; consInit = true; }
  float x = consEst[THIS_AGENT_ID];
  uint32_t now = millis();
  float sum = 0.0f;
  for (uint8_t a = 1; a <= 3; a++)
    if (a != THIS_AGENT_ID && consMs[a] != 0 && (now - consMs[a]) < CONS_TIMEOUT_MS)
      sum += (consEst[a] - x);
  x += dt * (CONS_GAMMA * (u - x) + CONS_KC * sum);
  x = clampF(x, 0.0f, 15.0f);
  consEst[THIS_AGENT_ID] = x;
  vBusConsensus = x;
  txFloat(canMakeId(CAN_CLASS_HIGH, THIS_AGENT_ID, CAN_VAR_VCONS), x);
}

/* ============================================================
   SEGURIDAD DE CONEXIÓN AL BUS (comparación de voltaje)
   - Difunde v_bus SOLO si el control está realmente conduciendo el bus
     (run ON, sin fault y no ya-desconectado): así nadie compara contra un
     agente apagado o desconectado.
   - Compara el v_bus propio contra los pares frescos; si no coincide con
     ninguno de forma sostenida ⇒ se asume desconectado (auto-recupera).
============================================================ */
static void txBusVoltageIfDriving() {
  bool  drive = false;  float vb = 0.0f;
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    drive = (runMode == RUN_ON) && !faultLatched && !busDisconnected;
    vb    = v_bus;
    xSemaphoreGive(xMutex);
  }
  if (drive) txFloat(canMakeId(CAN_CLASS_HIGH, THIS_AGENT_ID, CAN_VAR_VBUS_ACT), vb);
}

static void updateBusMembership() {
  if (!busChkEnable) { busDisconnected = false; busMismatchSince = 0; return; }

  bool  requested = false;  float myV = 0.0f, tol = 0.30f;
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    requested = (requestedMode == RUN_ON) && !faultLatched;   // "quiero controlar el bus"
    myV = v_bus; tol = busChkTol;
    xSemaphoreGive(xMutex);
  }
  /* Solo juzgamos si este agente quiere estar en el bus; si no, no aplica. */
  if (!requested) { busDisconnected = false; busMismatchSince = 0; return; }

  uint32_t now = millis();
  bool anyFresh = false, anyMatch = false;
  for (uint8_t a = 1; a <= 3; a++) {
    if (a == THIS_AGENT_ID) continue;
    if (busVpeerMs[a] != 0 && (now - busVpeerMs[a]) < BUS_CHK_FRESH_MS) {
      anyFresh = true;
      if (fabsf(myV - busVpeer[a]) <= tol) anyMatch = true;
    }
  }
  if (!anyFresh) { busDisconnected = false; busMismatchSince = 0; return; }  // nadie con quien comparar

  if (anyMatch) { busDisconnected = false; busMismatchSince = 0; }
  else {
    if (busMismatchSince == 0) busMismatchSince = now;
    else if ((uint32_t)(now - busMismatchSince) > BUS_CHK_DEBOUNCE_MS) busDisconnected = true;
  }
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
   APLICAR CONFIG RECIBIDA POR CAN → variables compartidas
   Mismos clamps que readParams usaba desde Firebase.
============================================================ */
static void canApplyConfig(uint8_t idx, float v) {
  bool gainsChanged = false;
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;

  switch (idx) {
    case CFG_TARGET_V:  targetV          = clampF(v,  3.0f,  6.0f);                 break;
    case CFG_OMEGA_V:   omegaV           = clampF(v,  1.0f, 100000.0f); gainsChanged = true; break;
    case CFG_OMEGA_I:   omegaI           = clampF(v,  1.0f, 100000.0f); gainsChanged = true; break;
    case CFG_PLANT_VDC: plantVdc         = clampF(v,  0.5f, 100.0f);    gainsChanged = true; break;
    case CFG_KD_DROOP:  kdDroop          = clampF(v,  0.0f, 20.0f);                 break;
    case CFG_IREF_MAX:  iRefMax          = clampF(v,  0.0f,  5.0f);                 break;
    case CFG_IREF_MIN:  iRefMin          = clampF(v, -5.0f,  0.0f);                 break;
    case CFG_OV_TRIP:   overvoltageTrip  = clampF(v,  4.0f,  8.0f);                 break;
    case CFG_UV_TRIP:   undervoltageTrip = clampF(v,  0.0f,  4.0f);                 break;
    case CFG_OC_TRIP:   overcurrentTrip  = clampF(v,  0.1f,  5.0f);                 break;
    case CFG_RUN_MODE:
      /* Solo on/off. Si hay un fault latcheado, el lazo de control sigue
         forzando OFF hasta recibir CFG_FAULT_ACK. */
      requestedMode = (v != 0.0f) ? RUN_ON : RUN_OFF;
      break;
    case CFG_FAULT_ACK:
      /* Acknowledge: despeja el fault latcheado (difunde "clear" por CAN
         en el próximo ciclo de taskCAN) y deja el agente en OFF. */
      faultLatched  = false;
      faultCode     = CAN_FAULT_CLEAR;
      requestedMode = RUN_OFF;
      break;
    /* ── Auto-calibración (el nodo calcula sus propios factores) ── */
    case CFG_CAL_BUS:
      if (rawVoltsA0 > 0.05f && v > 0.1f)
        divFactorBus = clampF(v / rawVoltsA0, 0.5f, 10.0f);   // v = voltaje real del bus
      break;
    case CFG_CAL_BAT:
      if (rawVoltsA1 > 0.05f && v > 0.1f)
        divFactorBat = clampF(v / rawVoltsA1, 0.5f, 10.0f);   // v = voltaje real de batería
      break;
    case CFG_ACS_ZERO:
      /* Cero con la corriente FILTRADA: corrige el offset para que i_out_filt→0
         (robusto a ruido). Equivale a fijar el offset = lectura cruda filtrada. */
      acsOffset = clampF(acsOffset + acsSign * acsSensitivity * i_out_filt, 0.0f, 5.0f);
      break;
    case CFG_ACS_SENS: {
      float av = (v < 0.0f) ? -v : v;
      acsSign        = (v < 0.0f) ? -1.0f : 1.0f;             // signo embebido en el valor
      acsSensitivity = clampF(av, 0.001f, 1.0f);
      break;
    }
    default: break;
  }
  xSemaphoreGive(xMutex);

  if (gainsChanged) recomputeGains();
#if CAN_DEBUG
  Serial.printf("[CAN][CFG] %s (idx=%u) = %.4f  [aplicado]\n", cfgName(idx), idx, v);
#endif
}

/* ============================================================
   Config EXTENDIDA (idx en payload) → parámetros que no caben en el nibble.
============================================================ */
static void canApplyConfig2Ext(uint8_t idx, float v) {
  bool gains = false;
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
  switch (idx) {
    case C2_ZETA_V: zetaV = clampF(v, 0.1f, 5.0f); gains = true; break;
    case C2_ZETA_I: zetaI = clampF(v, 0.1f, 5.0f); gains = true; break;
    case C2_BUS_CHK_EN:  busChkEnable = (v != 0.0f); break;
    case C2_BUS_CHK_TOL: busChkTol    = clampF(v, 0.02f, 5.0f); break;
    default: break;
  }
  xSemaphoreGive(xMutex);
  if (gains) recomputeGains();
}

/* ============================================================
   Procesar un frame recibido. Solo nos interesan los CONFIG
   dirigidos a este agente.
============================================================ */
static void handleRxFrame(const can_frame* f) {
  uint16_t id    = f->can_id & 0x7FF;
  uint8_t  cls   = canIdClass(id);
  uint8_t  agent = canIdAgent(id);
  uint8_t  var   = canIdVar(id);

#if CAN_DEBUG
  /* Imprime TODO lo que llega al bus (telemetría del agente 1 incluida),
     no solo lo dirigido a este nodo → confirma el enlace físico. */
  if (f->can_dlc >= 4)
    Serial.printf("[CAN][RX] id=0x%03X %-6s a%u v%u dlc=%u = %.4f\n",
                  id, canClassName(cls), agent, var, f->can_dlc, bytesToFloat(f->data));
  else
    Serial.printf("[CAN][RX] id=0x%03X %-6s a%u v%u dlc=%u d0=%u\n",
                  id, canClassName(cls), agent, var, f->can_dlc,
                  f->can_dlc ? f->data[0] : 0);
#endif

  /* Referencia de corriente común que difunde el maestro (droop) */
  if (cls == CAN_CLASS_HIGH && var == CAN_VAR_IREFV &&
      agent == MASTER_AGENT_ID && f->can_dlc >= 4) {
    iRefVExt   = bytesToFloat(f->data);
    iRefVExtMs = millis();
    return;
  }

  /* Consenso del voltaje de bus (opción B): estimación de otro agente */
  if (cls == CAN_CLASS_HIGH && var == CAN_VAR_VCONS && f->can_dlc >= 4) {
    if (agent >= 1 && agent <= 3 && agent != THIS_AGENT_ID) {
      consEst[agent] = bytesToFloat(f->data);
      consMs[agent]  = millis();
    }
    return;
  }

  /* Seguridad de bus: v_bus real que difunde un par CON control ON */
  if (cls == CAN_CLASS_HIGH && var == CAN_VAR_VBUS_ACT && f->can_dlc >= 4) {
    if (agent >= 1 && agent <= 3 && agent != THIS_AGENT_ID) {
      busVpeer[agent]   = bytesToFloat(f->data);
      busVpeerMs[agent] = millis();
    }
    return;
  }

  if (cls != CAN_CLASS_CONFIG)   return;   // ignorar telemetría/faults de otros
  if (agent != THIS_AGENT_ID)    return;   // config dirigida a otro agente
  /* Frame extendido (dlc=5): [u8 idx][f32]. El clásico es dlc=4. */
  if (f->can_dlc >= 5) { canApplyConfig2Ext(f->data[0], bytesToFloat(f->data + 1)); return; }
  if (f->can_dlc < 4)            return;

  canApplyConfig(var, bytesToFloat(f->data));
}

/* ============================================================
   TX: telemetría y faults propios
============================================================ */
#if CAN_TX_OWN_TELEMETRY
static void txOwnTelemetry() {
  Snapshot s = takeSnapshot();
  bool ok __attribute__((unused)) = true;
  /* Solo i_out va en alta frecuencia; v_bat/v_bus/SoC son lentos/redundantes.
     La potencia ya no se envía: se calcula en la web (i_out · v_bus). */
  /* Estado empaquetado: bit0 = control activo (run ON), bit1 = topología BOOST,
     bit2 = desconectado del bus (seguridad) */
  float status = (float)((s.runMode == RUN_ON ? 1 : 0) |
                         (s.activeTopology == TOPO_BOOST ? 2 : 0) |
                         (busDisconnected ? 4 : 0));
  ok &= txFloat(canMakeId(CAN_CLASS_HIGH, THIS_AGENT_ID, CAN_VAR_IOUT),   s.i_out_filt);
  ok &= txFloat(canMakeId(CAN_CLASS_LOW,  THIS_AGENT_ID, CAN_VAR_VBAT),   s.v_bat);
  ok &= txFloat(canMakeId(CAN_CLASS_LOW,  THIS_AGENT_ID, CAN_VAR_VBUS),   s.v_bus);
  ok &= txFloat(canMakeId(CAN_CLASS_LOW,  THIS_AGENT_ID, CAN_VAR_SOC),    s.soc);
  ok &= txFloat(canMakeId(CAN_CLASS_LOW,  THIS_AGENT_ID, CAN_VAR_EDELIV), s.e_deliv);
  ok &= txFloat(canMakeId(CAN_CLASS_LOW,  THIS_AGENT_ID, CAN_VAR_EABSORB),s.e_absorb);
  ok &= txFloat(canMakeId(CAN_CLASS_LOW,  THIS_AGENT_ID, CAN_VAR_STATUS), status);
#if CAN_DEBUG
  /* [OK] = ACKeado por el agente 1; [FAIL] = sin ACK (cableado/terminación/GND). */
  Serial.printf("[CAN][TX] a%u i_out=%.3f v_bat=%.3f v_bus=%.3f soc=%.3f  [%s]\n",
                THIS_AGENT_ID, s.i_out, s.v_bat, s.v_bus, s.soc, ok ? "OK" : "FAIL");
#endif
}
#endif

/* ============================================================
   INIT del MCP2515
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
  canIntSem = xSemaphoreCreateBinary();

  if (!canHwInit()) {
    Serial.println("[CAN] Inicialización fallida. Tarea CAN detenida.");
    vTaskDelete(nullptr);
    return;
  }

  /* INT del MCP2515: activo bajo, open-drain → pull-up interno + FALLING */
  pinMode(PIN_CAN_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_CAN_INT), canIsr, FALLING);
  canReady = true;
  Serial.println("[CAN] Operativo (agente 2 nodo).");

  uint32_t lastTxMs  = millis();
  bool     prevFault = false;

  /* Drenar cualquier frame que ya estuviera pendiente al arrancar */
  { can_frame f; while (readMessage(&f)) handleRxFrame(&f); }

  for (;;) {
    bool busy = false;

    /* ── RX 100% por sondeo: drenar todo lo pendiente cada iteración ── */
    can_frame f;
    while (readMessage(&f)) { handleRxFrame(&f); busy = true; }

    /* ADS caído → se emite el FAULT pero NO telemetría ni consenso. */
    bool healthy = localSensorsHealthy();

    /* ── Fault propio: emitir en el flanco; "clear" al despejar ── */
#if CAN_TX_OWN_FAULTS
    bool    fault = false;
    uint8_t code  = CAN_FAULT_GENERIC;
    if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      fault = faultLatched;
      code  = faultCode;
      xSemaphoreGive(xMutex);
    }
    if (fault && !prevFault) {                                // flanco de subida
      bool ok __attribute__((unused)) = txFaultFrame(code);
#if CAN_DEBUG
      Serial.printf("[CAN][TX] FAULT code=%u  [%s]\n", code, ok ? "OK" : "FAIL");
#endif
    }
    if (!fault && prevFault) {                                // fault despejado
      bool ok __attribute__((unused)) = txFaultFrame(CAN_FAULT_CLEAR);
#if CAN_DEBUG
      Serial.printf("[CAN][TX] FAULT CLEAR  [%s]\n", ok ? "OK" : "FAIL");
#endif
    }
    prevFault = fault;
#endif

    /* ── Telemetría propia periódica ── */
#if CAN_TX_OWN_TELEMETRY
    if (healthy && millis() - lastTxMs >= CAN_TX_PERIOD_MS) {
      lastTxMs = millis();
      txOwnTelemetry();
      busy = true;
    }
#endif

    /* ── Consenso del voltaje de bus (opción B) ── */
    if (healthy) {
      uint32_t nowc = millis();
      uint32_t per  = consNeighborFresh() ? CONS_PERIOD_MS : CONS_BEACON_MS;
      if (nowc - lastConsMs >= per) {
        float dt = clampF((float)(nowc - lastConsMs) * 0.001f, 0.005f, 0.1f);
        lastConsMs = nowc;
        runConsensus(dt);
        busy = true;
      }
    }

    /* ── Seguridad de conexión al bus: difundir v_bus real (si conduce) y
       comparar contra los pares. Solo con sensores sanos. ── */
    if (healthy && (millis() - lastBusChkMs >= BUS_CHK_TX_MS)) {
      lastBusChkMs = millis();
      txBusVoltageIfDriving();
      updateBusMembership();
      busy = true;
    }

    /* Si hubo actividad, seguir sondeando sin ceder (no perder la ráfaga).
       Si el bus está inactivo, ceder hasta 1 ms (la INT lo despierta antes
       si llega un frame) para no matar el watchdog ni otras tareas. */
    if (!busy) xSemaphoreTake(canIntSem, pdMS_TO_TICKS(1));
  }
}
