/*
 * can_bus.cpp — AGENTE 3 (nodo CAN esclavo droop, protocolo extendido)
 * ─────────────────────────────────────────────────────────────
 * Driver MCP2515 (SPI manual, idéntico a los otros agentes) + RX por sondeo.
 * Core 0 (taskCAN).
 *   RX:  i_ref_v del maestro (HIGH) + config EXTENDIDA dirigida al agente 3.
 *   TX:  telemetría EXTENDIDA (un índice por tick) + faults (flanco).
 * Config/telemetría extendidas: payload [u8 índice][float32].
 * ─────────────────────────────────────────────────────────────
 */
#include "can_bus.h"
#include "shared.h"
#include "control.h"

#include <SPI.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/* ── Instrucciones SPI / registros / modos (MCP2515) ── */
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
#define INTF_RX0IF    0x01
#define INTF_RX1IF    0x02
#define INTF_TX0IF    0x04
#define INTE_RX0IE    0x01
#define INTE_RX1IE    0x02
#define MODE_NORMAL   0x00
#define MODE_CONFIG   0x80
#define MODE_MASK     0xE0

struct can_frame { uint32_t can_id; uint8_t can_dlc; uint8_t data[8]; };

static SPIClass    spi(VSPI);
static SPISettings spiCfg(CAN_SPI_HZ, MSBFIRST, SPI_MODE0);
static SemaphoreHandle_t canIntSem = nullptr;

/* INT del MCP2515 en GPIO4 (pin 26); GPIO27 queda libre para el PWM del PV. */
static void IRAM_ATTR canIsr() {
  BaseType_t hpw = pdFALSE;
  if (canIntSem) xSemaphoreGiveFromISR(canIntSem, &hpw);
  if (hpw) portYIELD_FROM_ISR();
}

static void csLow()  { digitalWrite(PIN_CAN_CS, LOW); }
static void csHigh() { digitalWrite(PIN_CAN_CS, HIGH); }

static void mcpReset() { spi.beginTransaction(spiCfg); csLow(); spi.transfer(INSTR_RESET); csHigh(); spi.endTransaction(); delay(20); }
static uint8_t mcpRead(uint8_t reg) {
  spi.beginTransaction(spiCfg); csLow();
  spi.transfer(INSTR_READ); spi.transfer(reg);
  uint8_t v = spi.transfer(0x00); csHigh(); spi.endTransaction(); return v;
}
static void mcpWrite(uint8_t reg, uint8_t val) {
  spi.beginTransaction(spiCfg); csLow();
  spi.transfer(INSTR_WRITE); spi.transfer(reg); spi.transfer(val);
  csHigh(); spi.endTransaction();
}
static void mcpBitMod(uint8_t reg, uint8_t mask, uint8_t val) {
  spi.beginTransaction(spiCfg); csLow();
  spi.transfer(INSTR_BITMOD); spi.transfer(reg); spi.transfer(mask); spi.transfer(val);
  csHigh(); spi.endTransaction();
}
static void mcpRTS_TX0() { spi.beginTransaction(spiCfg); csLow(); spi.transfer(INSTR_RTS_TX0); csHigh(); spi.endTransaction(); }

static bool setMode(uint8_t mode, const char* n) {
  mcpBitMod(REG_CANCTRL, MODE_MASK, mode);
  for (int i = 0; i < 50; i++) {
    if ((mcpRead(REG_CANSTAT) & MODE_MASK) == mode) { Serial.printf("[CAN][OK] modo %s\n", n); return true; }
    delay(2);
  }
  Serial.printf("[CAN][FAIL] modo %s\n", n); return false;
}
static bool setBitrate500k_8MHz() {
  mcpWrite(REG_CNF1, 0x00); mcpWrite(REG_CNF2, 0x90); mcpWrite(REG_CNF3, 0x02);
  if (mcpRead(REG_CNF1) != 0x00 || mcpRead(REG_CNF2) != 0x90 || mcpRead(REG_CNF3) != 0x02) {
    Serial.println("[CAN][FAIL] CNF"); return false;
  }
  Serial.println("[CAN][OK] bitrate 500kbps@8MHz"); return true;
}

static bool readMessage(can_frame* f);
static void handleRxFrame(const can_frame* f);

static bool sendMessage(const can_frame* f) {
  uint8_t txctrl = mcpRead(REG_TXB0CTRL);
  if (txctrl & 0x08) mcpBitMod(REG_TXB0CTRL, 0x08, 0x00);
  uint16_t sid = f->can_id & 0x7FF;
  mcpWrite(REG_TXB0SIDH, (sid >> 3) & 0xFF);
  mcpWrite(REG_TXB0SIDL, (sid & 0x07) << 5);
  uint8_t dlc = f->can_dlc > 8 ? 8 : f->can_dlc;
  mcpWrite(REG_TXB0DLC, dlc);
  for (uint8_t i = 0; i < dlc; i++) mcpWrite(REG_TXB0D0 + i, f->data[i]);
  mcpBitMod(REG_CANINTF, INTF_TX0IF, 0x00);
  mcpRTS_TX0();

  uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < 30000UL) {
    if (mcpRead(REG_CANINTF) & INTF_TX0IF) { mcpBitMod(REG_CANINTF, INTF_TX0IF, 0x00); return true; }
    can_frame rf;
    if (readMessage(&rf)) handleRxFrame(&rf);
    if ((uint32_t)(micros() - t0) > 2000UL) vTaskDelay(1);
  }
  mcpBitMod(REG_TXB0CTRL, 0x08, 0x00);
  return false;
}

static bool readMessage(can_frame* f) {
  uint8_t intf = mcpRead(REG_CANINTF);
  uint8_t sidh_reg, sidl_reg, dlc_reg, d0_reg, clearFlag;
  if (intf & INTF_RX0IF) { sidh_reg = REG_RXB0SIDH; sidl_reg = REG_RXB0SIDL; dlc_reg = REG_RXB0DLC; d0_reg = REG_RXB0D0; clearFlag = INTF_RX0IF; }
  else if (intf & INTF_RX1IF) { sidh_reg = REG_RXB1SIDH; sidl_reg = REG_RXB1SIDL; dlc_reg = REG_RXB1DLC; d0_reg = REG_RXB1D0; clearFlag = INTF_RX1IF; }
  else return false;
  uint8_t sidh = mcpRead(sidh_reg), sidl = mcpRead(sidl_reg);
  f->can_id = ((uint16_t)sidh << 3) | (sidl >> 5);
  uint8_t dlc = mcpRead(dlc_reg) & 0x0F; if (dlc > 8) dlc = 8;
  f->can_dlc = dlc;
  for (uint8_t i = 0; i < dlc; i++) f->data[i] = mcpRead(d0_reg + i);
  mcpBitMod(REG_CANINTF, clearFlag, 0x00);
  return true;
}

/* ── Helpers de payload ── */
static float bytesToFloat(const uint8_t* d) { float v; memcpy(&v, d, 4); return v; }

static void setCurrentMode(CurrentCal& cal, float modeValue) {
  setCurrentRange(cal, (modeValue >= 0.5f) ? CUR_RANGE_20A : CUR_RANGE_5A);
}

/* Frame extendido: [u8 idx][f32], dlc=5 */
static bool txExt(uint8_t cls, uint8_t idx, float v) {
  can_frame f;
  f.can_id  = canMakeId(cls, THIS_AGENT_ID, CAN_VAR_EXT);
  f.can_dlc = 5;
  f.data[0] = idx;
  memcpy(f.data + 1, &v, 4);
  return sendMessage(&f);
}
static bool txFaultFrame(uint8_t code) {
  can_frame f;
  f.can_id  = canMakeId(CAN_CLASS_FAULT, THIS_AGENT_ID, code);
  f.can_dlc = 1; f.data[0] = code;
  return sendMessage(&f);
}
/* Frame clásico float32 (dlc=4) — para el consenso del bus (opción B). */
static bool txFloat(uint16_t id, float v) {
  can_frame f;
  f.can_id = id; f.can_dlc = 4;
  memcpy(f.data, &v, 4);
  return sendMessage(&f);
}

/* ============================================================
   CONSENSO DISTRIBUIDO DEL VOLTAJE DE BUS (opción B)
   Estima el promedio de las mediciones intercambiando la estimación por CAN
   (CAN_VAR_VCONS, frame clásico). Corre en taskCAN. u_i = v_bus5 local.
============================================================ */
static float    consEst[4] = {0, 0, 0, 0};
static uint32_t consMs[4]  = {0, 0, 0, 0};
static bool     consInit   = false;
static uint32_t lastConsMs = 0;

/* ── Seguridad de conexión al bus: v_bus5 real de cada par (lo difunden solo con
   control ON). Comparamos el propio contra estos. ── */
static float    busVpeer[4]      = {0, 0, 0, 0};
static uint32_t busVpeerMs[4]    = {0, 0, 0, 0};
static uint32_t busMismatchSince = 0;
static uint32_t lastBusChkMs     = 0;

/* ¿El ADS del bus (ADS1) está sano? Si está desconectado (hwStatus=HW_NO_ADS)
   la medición v_bus5 queda congelada/errónea: NO difundimos telemetría ni
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
  u = v_bus5;                             // medición local (u_i)
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
   Para AG3 "conducir el bus" = buckboost activo (run ON, sin fault y con el
   buckboost habilitado por el modo de operación). Usa v_bus5.
============================================================ */
static bool bbEnabledByModeCAN() {
  return operationMode != OP_PV_ONLY;   // FULL o BUCKBOOST_ONLY
}
static void txBusVoltageIfDriving() {
  bool  drive = false;  float vb = 0.0f;
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    drive = (runMode == RUN_ON) && !faultLatched && bbEnabledByModeCAN() && !busDisconnected;
    vb    = v_bus5;
    xSemaphoreGive(xMutex);
  }
  if (drive) txFloat(canMakeId(CAN_CLASS_HIGH, THIS_AGENT_ID, CAN_VAR_VBUS_ACT), vb);
}

static void updateBusMembership() {
  if (!busChkEnable) { busDisconnected = false; busMismatchSince = 0; return; }

  bool  requested = false;  float myV = 0.0f, tol = 0.30f;
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    requested = (requestedMode == RUN_ON) && !faultLatched && bbEnabledByModeCAN();
    myV = v_bus5; tol = busChkTol;
    xSemaphoreGive(xMutex);
  }
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
  if (!anyFresh) { busDisconnected = false; busMismatchSince = 0; return; }

  if (anyMatch) { busDisconnected = false; busMismatchSince = 0; }
  else {
    if (busMismatchSince == 0) busMismatchSince = now;
    else if ((uint32_t)(now - busMismatchSince) > BUS_CHK_DEBOUNCE_MS) busDisconnected = true;
  }
}

/* ============================================================
   APLICAR CONFIG EXTENDIDA RECIBIDA POR CAN
   Las calibraciones se calculan desde los CRUDOS más recientes que
   capturó Core 1 (sin tocar el I2C desde aquí ⇒ sin conflicto de bus).
============================================================ */
static void canApplyConfig3(uint8_t idx, float v) {
  bool gains = false;
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
  switch (idx) {
    /* Buckboost */
    case C3_TARGET_V:   targetV = clampF(v, 0.5f, 9.0f); break;
    case C3_OMEGA_V:    omegaV = clampF(v, 1.0f, 100000.0f); gains = true; break;
    case C3_OMEGA_I:    omegaI = clampF(v, 1.0f, 100000.0f); gains = true; break;
    case C3_ZETA_V:     zetaV  = clampF(v, 0.1f, 5.0f);      gains = true; break;
    case C3_ZETA_I:     zetaI  = clampF(v, 0.1f, 5.0f);      gains = true; break;
    case C3_PLANT_VDC:  plantVdc = clampF(v, 0.5f, 100.0f); gains = true; break;
    case C3_KD_DROOP:   kdDroop = clampF(v, 0.0f, 20.0f); break;
    case C3_IREF_MAX:   iRefMax = clampF(v, 0.0f, 10.0f); break;
    case C3_IREF_MIN:   iRefMin = clampF(v, -10.0f, 0.0f); break;
    case C3_OV_TRIP:    overvoltageTrip = clampF(v, 4.0f, 8.0f); break;
    case C3_UV_TRIP:    undervoltageTrip = clampF(v, 0.0f, 5.0f); break;
    case C3_OC_TRIP:    overcurrentTrip = clampF(v, 0.1f, 10.0f); break;
    case C3_RUN_MODE:   requestedMode = (v != 0.0f) ? RUN_ON : RUN_OFF; break;
    case C3_FAULT_ACK:  faultLatched = false; faultCode = CAN_FAULT_CLEAR; requestedMode = RUN_OFF; break;
    case C3_BAT_OV_TRIP: bat2sOvTrip = clampF(v, 3.0f, 12.0f); break;
    case C3_BAT_UV_TRIP: bat2sUvTrip = clampF(v, 0.0f, 12.0f); break;
    case C3_SOC_EMPTY_V: socEmptyV = clampF(v, 0.0f, 12.0f); break;
    case C3_SOC_FULL_V:  socFullV = clampF(v, 0.0f, 12.0f); break;
    case C3_PWM_PERIOD_US: pwmPeriodUs = (uint32_t)clampF(v, 50.0f, 500.0f); break;
    case C3_TON_MAX_US:  tonMaxUs = clampF(v, 0.0f, 99.0f); break;
    case C3_MIN_OFF_US:  minOffUs = clampF(v, 0.0f, 40.0f); break;
    case C3_SOFTSTART_S: softStartS = clampF(v, 0.05f, 10.0f); break;

    /* PV / MPPT */
    case C3_PV_MODE:     pvMode = (PvBoostMode)clampU8((int)(v + 0.5f), 0, 4); break;
    case C3_PV_ARM:      if (v != 0.0f) pvArmRequest = true; else pvArmed = false; break;
    case C3_PV_FIXED_DUTY: pvFixedDuty = clampF(v, 0.0f, 0.80f); break;
    case C3_PV_VPV_REF:  pvVpvRef = clampF(v, pvVpvRefMin, pvVpvRefMax); break;
    case C3_PV_VPV_REF_MIN: pvVpvRefMin = clampF(v, 0.0f, pvVpvRefMax); break;
    case C3_PV_VPV_REF_MAX: pvVpvRefMax = fmaxf(v, pvVpvRefMin); break;
    case C3_PV_KP:       pvKpVpv = clampF(v, 0.0f, 5.0f); break;
    case C3_PV_KI:       pvKiVpv = clampF(v, 0.0f, 20.0f); break;
    case C3_PV_DUTY_MIN: pvDutyMin = clampF(v, 0.0f, pvDutyMax); break;
    case C3_PV_DUTY_START: pvDutyStart = clampF(v, 0.0f, pvDutyMax); break;
    case C3_PV_DUTY_MAX: pvDutyMax = clampF(v, 0.02f, 0.80f); break;
    case C3_PV_DUTY_SLEW: pvDutySlewPerStep = clampF(v, 0.0005f, 0.10f); break;
    case C3_PV_SOFTSTART_MS: pvSoftstartMs = (uint32_t)clampF(v, 0.0f, 20000.0f); break;
    case C3_PV_CTRL_MS:  pvControlPeriodMs = (uint32_t)clampF(v, 10.0f, 1000.0f); break;
    case C3_PV_PANEL_TRIPLO: pvPanelTripLow = clampF(v, 0.0f, 10.0f); break;
    case C3_PV_BAT_WARN_HI: pvBatWarnHigh = clampF(v, 0.0f, pvBatTripHigh); break;
    case C3_PV_BAT_TRIP_HI: pvBatTripHigh = clampF(v, 0.0f, 12.0f); break;
    case C3_PV_IIN_MAX:  pvInputCurrentMax = clampF(v, 0.05f, 20.0f); break;
    case C3_PV_IBAT_MAX: pvBatteryChargeCurrentMax = clampF(v, 0.05f, 20.0f); break;
    case C3_PV_IBAT_LIM: pvIbatChargeLim = clampF(v, 0.0f, 20.0f); break;
    case C3_IBAT_IDLE:   iBatIdle = clampF(v, 0.0f, 5.0f); break;
    case C3_BUS_CHK_EN:  busChkEnable = (v != 0.0f); break;
    case C3_BUS_CHK_TOL: busChkTol    = clampF(v, 0.02f, 5.0f); break;
    case C3_PV_PIN_MAX:  pvInputPowerMax = clampF(v, 0.5f, 100.0f); break;
    case C3_PV_IREV_MAX: pvReverseCurrentMax = clampF(v, 0.0f, 5.0f); break;
    case C3_MPPT_STEP_V: mpptStepV = clampF(v, 0.001f, 0.25f); break;
    case C3_MPPT_EPS_POWER: mpptEpsPowerW = clampF(v, 0.0f, 5.0f); break;
    case C3_MPPT_MIN_POWER: mpptMinPowerW = clampF(v, 0.0f, 100.0f); break;
    case C3_MPPT_PERIOD_MS: mpptPeriodMs = (uint32_t)clampF(v, 100.0f, 60000.0f); break;
    case C3_MPPT_REACQ_MS:  mpptReacquireMs = (uint32_t)clampF(v, 500.0f, 120000.0f); break;
    case C3_OP_MODE:     operationMode = (OperationMode)clampU8((int)(v + 0.5f), 0, 2); break;
    case C3_PV_FAULT_ACK: pvFaultLatched = false; pvFaultCode = CAN_FAULT_CLEAR; pvMode = PV_OFF; break;

    /* Calibración (desde crudos recientes) */
    case C3_CAL_BUS5:   if (rawBus5  > 0.05f && v > 0.1f) divFactorBus5  = clampF(v / rawBus5,  0.1f, 50.0f); break;
    case C3_CAL_BAT2S:  if (rawBat2s > 0.05f && v > 0.1f) divFactorPv2s  = clampF(v / rawBat2s, 0.1f, 50.0f); break;
    case C3_CAL_PANEL:  if (rawPanel > 0.05f && v > 0.1f) divFactorPanel = clampF(v / rawPanel, 0.1f, 50.0f); break;
    case C3_CAL_SPARE:  if (rawSpare > 0.05f && v > 0.1f) divFactorSpare = clampF(v / rawSpare, 0.1f, 50.0f); break;
    case C3_DIV_BUS5:   divFactorBus5  = clampF(v, 0.1f, 50.0f); break;
    case C3_DIV_BAT2S:  divFactorPv2s  = clampF(v, 0.1f, 50.0f); break;
    case C3_DIV_PANEL:  divFactorPanel = clampF(v, 0.1f, 50.0f); break;
    case C3_DIV_SPARE:  divFactorSpare = clampF(v, 0.1f, 50.0f); break;
    case C3_IZERO_IBUS:   /* cero con la corriente FILTRADA (i_bb_filt→0) */
                          calIBus5.zero  = clampF(calIBus5.zero + (float)calIBus5.sign * calIBus5.sens * i_bb_filt, 0.0f, 4.096f); calIBus5.ready = false; break;
    case C3_ISENS_IBUS:   calIBus5.sign  = (v < 0.0f) ? -1 : 1; calIBus5.sens = clampF(fabsf(v), 0.02f, 0.30f); calIBus5.ready = false; break;
    case C3_IZERO_IPANEL: /* cero con la corriente FILTRADA (i_panel→0) */
                          calIPanel.zero = clampF(calIPanel.zero + (float)calIPanel.sign * calIPanel.sens * calIPanel.filtered, 0.0f, 4.096f); calIPanel.ready = false; break;
    case C3_ISENS_IPANEL: calIPanel.sign = (v < 0.0f) ? -1 : 1; calIPanel.sens = clampF(fabsf(v), 0.02f, 0.30f); calIPanel.ready = false; break;
    case C3_IZERO_IBAT:   /* cero con la corriente FILTRADA para que i_bat parta en +iBatIdle
                             (las baterías alimentan los sensores: nunca hay 0 A real). */
                          calIBat.zero   = clampF(calIBat.zero + (float)calIBat.sign * calIBat.sens * (calIBat.filtered - iBatIdle), 0.0f, 4.096f); calIBat.ready = false; break;
    case C3_ISENS_IBAT:   calIBat.sign   = (v < 0.0f) ? -1 : 1; calIBat.sens = clampF(fabsf(v), 0.02f, 0.30f); calIBat.ready = false; break;
    case C3_IMODE_IBUS:   setCurrentMode(calIBus5, v); break;
    case C3_IMODE_IPANEL: setCurrentMode(calIPanel, v); break;
    case C3_IMODE_IBAT:   setCurrentMode(calIBat, v); break;
    case C3_KCL_ETA:    boostKclEfficiency = clampF(v, 0.20f, 1.05f); break;
    case C3_SAVE_CFG:   break;   // NVS deshabilitado: la config vive en Firebase (no-op)
    default: break;
  }
  xSemaphoreGive(xMutex);
  if (gains) recomputeGains();
#if CAN_DEBUG
  Serial.printf("[CAN][CFG3] idx=%u = %.4f\n", idx, v);
#endif
}

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
#endif

static void handleRxFrame(const can_frame* f) {
  uint16_t id = f->can_id & 0x7FF;
  uint8_t cls = canIdClass(id), agent = canIdAgent(id), var = canIdVar(id);

#if CAN_DEBUG
  /* Imprime TODO lo que llega al bus (incluida la telemetría del AG1) → así
     confirmas el enlace físico y el ACK, no solo lo dirigido a este nodo. */
  if (var == CAN_VAR_EXT && f->can_dlc >= 5)
    Serial.printf("[CAN][RX] id=0x%03X %-6s a%u EXT idx=%u = %.4f\n",
                  id, canClassName(cls), agent, f->data[0], bytesToFloat(f->data + 1));
  else if (f->can_dlc >= 4)
    Serial.printf("[CAN][RX] id=0x%03X %-6s a%u v%u dlc=%u = %.4f\n",
                  id, canClassName(cls), agent, var, f->can_dlc, bytesToFloat(f->data));
  else
    Serial.printf("[CAN][RX] id=0x%03X %-6s a%u v%u dlc=%u d0=%u\n",
                  id, canClassName(cls), agent, var, f->can_dlc, f->can_dlc ? f->data[0] : 0);
#endif

  /* Referencia común del maestro (droop) */
  if (cls == CAN_CLASS_HIGH && var == CAN_VAR_IREFV && agent == MASTER_AGENT_ID && f->can_dlc >= 4) {
    iRefVExt = bytesToFloat(f->data); iRefVExtMs = millis();
#if CAN_DEBUG
    Serial.printf("       └─ i_ref_v del maestro (a%u) = %.4f A\n", agent, iRefVExt);
#endif
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
  /* Config extendida dirigida a este agente: [u8 idx][f32] */
  if (cls == CAN_CLASS_CONFIG && agent == THIS_AGENT_ID && var == CAN_VAR_EXT && f->can_dlc >= 5) {
    canApplyConfig3(f->data[0], bytesToFloat(f->data + 1));
    return;
  }
}

/* ── Telemetría: valor por índice ── */
static float telemValue(const Snapshot& s, uint8_t idx) {
  switch (idx) {
    case T3_V_BUS5:   return s.v_bus5;
    case T3_V_BAT2S:  return s.v_bat2s;
    case T3_V_PANEL:  return s.v_panel;
    case T3_I_BB_BUS5:return s.i_bb_filt;   // corriente filtrada al dashboard
    case T3_I_PANEL:  return s.i_panel;
    case T3_I_BAT:    return s.i_bat;
    case T3_P_PANEL:  return s.p_panel;
    case T3_SOC:      return s.soc;
    case T3_STATUS: {
      uint32_t st = 0;
      if (s.runMode == RUN_ON)            st |= T3_ST_BB_RUN;
      if (s.activeTopology == TOPO_BOOST) st |= T3_ST_TOPO_BOOST;
      if (s.pvArmed)                      st |= T3_ST_PV_ARMED;
      if (s.fault)                        st |= T3_ST_BB_FAULT;
      if (s.pvFault)                      st |= T3_ST_PV_FAULT;
      if (busDisconnected)                st |= T3_ST_BUS_DISC;
      st |= ((uint32_t)s.pvMode & 0x7) << T3_ST_PVMODE_SHIFT;
      return (float)st;
    }
    case T3_TON_CMD:  return s.tonCmd;
    case T3_PV_DUTY:  return s.pvDutyApplied;
    case T3_PV_VPV_REF: return s.pvVpvRef;
    case T3_BB_FAULT_CODE: return s.fault   ? (float)s.faultCode   : 0.0f;
    case T3_PV_FAULT_CODE: return s.pvFault ? (float)s.pvFaultCode : 0.0f;
    case T3_E_DELIV:  return s.e_deliv;
    case T3_E_ABSORB: return s.e_absorb;
    case T3_V_MID:    return s.v_spare;   // punto medio de la 2S
    default: return 0.0f;
  }
}

/* ============================================================
   INIT MCP2515
============================================================ */
static bool canHwInit() {
  pinMode(PIN_CAN_CS, OUTPUT); csHigh();
  spi.begin(PIN_CAN_SCK, PIN_CAN_MISO, PIN_CAN_MOSI, PIN_CAN_CS);
  delay(50);
  bool inConfig = false; uint8_t s = 0;
  for (int a = 1; a <= 5; a++) {
    mcpReset(); delay(10);
    s = mcpRead(REG_CANSTAT);
    Serial.printf("[CAN] intento %d/5: CANSTAT=0x%02X\n", a, s);
    if ((s & MODE_MASK) == MODE_CONFIG) { inConfig = true; break; }
    delay(50);
  }
  if (!inConfig) { Serial.println("[CAN] ABORT: MCP2515 no responde por SPI."); return false; }
  if (!setBitrate500k_8MHz()) return false;
  mcpWrite(REG_RXB0CTRL, 0x64);   // BUKT rollover
  mcpWrite(REG_RXB1CTRL, 0x60);
  mcpWrite(REG_CANINTE, INTE_RX0IE | INTE_RX1IE);
  if (!setMode(MODE_NORMAL, "NORMAL")) return false;
  return true;
}

/* ============================================================
   TAREA — Core 0
============================================================ */
void taskCAN(void* pvParams) {
  canIntSem = xSemaphoreCreateBinary();
  if (!canHwInit()) { Serial.println("[CAN] init fallida, tarea detenida."); vTaskDelete(nullptr); return; }
  pinMode(PIN_CAN_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_CAN_INT), canIsr, FALLING);
  Serial.println("[CAN] Operativo (agente 3 esclavo).");

  uint32_t lastTxMs = millis();
  uint8_t  telIdx = 0;
  bool prevFault = false, prevPvFault = false;

  { can_frame f; while (readMessage(&f)) handleRxFrame(&f); }

  for (;;) {
    bool busy = false;
    can_frame f;
    while (readMessage(&f)) { handleRxFrame(&f); busy = true; }

    /* ADS del bus caído → se emite el FAULT pero NO telemetría ni consenso. */
    bool healthy = localSensorsHealthy();

    /* Faults (flanco de subida; estado autoritativo va en telemetría) */
    bool fault = false, pvFault = false;
    uint8_t code = CAN_FAULT_GENERIC, pvCode = CAN_FAULT_PV;
    if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      fault = faultLatched; code = faultCode;
      pvFault = pvFaultLatched; pvCode = pvFaultCode;
      xSemaphoreGive(xMutex);
    }
    if (fault && !prevFault)     txFaultFrame(code);
    if (pvFault && !prevPvFault) txFaultFrame(pvCode);
    prevFault = fault; prevPvFault = pvFault;

    /* Telemetría extendida: un índice por tick (set completo ≈ 1.1 s) */
    if (healthy && millis() - lastTxMs >= CAN_TX_PERIOD_MS) {
      lastTxMs = millis();
      Snapshot s = takeSnapshot();
      bool ok __attribute__((unused)) = txExt(CAN_CLASS_LOW, telIdx, telemValue(s, telIdx));
#if CAN_DEBUG && CAN_DEBUG_TX
      /* Detallado: cada frame TX (ruidoso). ACK=OK ⇒ el AG1 lo confirmó. */
      Serial.printf("[CAN][TX] a3 EXT idx=%u = %.4f  ACK=%s\n",
                    telIdx, telemValue(s, telIdx), ok ? "OK" : "FAIL");
#elif CAN_DEBUG
      /* Silencioso: solo un latido por ciclo completo (~1.1 s) para no tapar
         los RX de config. FAIL ⇒ nadie ACKeó (revisa CANH/CANL, 120Ω, GND). */
      if (telIdx == 0)
        Serial.printf("[CAN][TX] a3 telemetría (ciclo)  ACK=%s\n", ok ? "OK" : "FAIL");
#endif
      telIdx = (telIdx + 1) % T3_TELE_COUNT;
      busy = true;
    }

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

    /* ── Seguridad de conexión al bus: difundir v_bus5 real (si conduce) y
       comparar contra los pares. Solo con sensores sanos. ── */
    if (healthy && (millis() - lastBusChkMs >= BUS_CHK_TX_MS)) {
      lastBusChkMs = millis();
      txBusVoltageIfDriving();
      updateBusMembership();
      busy = true;
    }

    if (!busy) xSemaphoreTake(canIntSem, pdMS_TO_TICKS(1));   // duerme hasta INT o 1 ms
  }
}
