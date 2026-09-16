/*
 * shared.cpp — AGENTE 3
 * Definición de variables compartidas, defaults y snapshot atómico.
 */

#include "shared.h"

SemaphoreHandle_t xMutex = nullptr;
volatile HwStatus hwStatus = HW_OK;

/* ── Mediciones ── */
volatile float v_bus5    = 0.0f;
volatile float v_bat2s   = 0.0f;
volatile float v_panel   = 0.0f;
volatile float v_spare   = 0.0f;
volatile float i_bb_bus5 = 0.0f;
volatile float i_bb_filt = 0.0f;
volatile float vBusConsensus = 0.0f;

/* Seguridad de conexión al bus (comparación de voltaje entre agentes). */
volatile bool  busDisconnected = false;
volatile bool  busChkEnable    = true;
volatile float busChkTol       = 0.30f;   // V — debe superar el error de calibración entre sensores
volatile float i_panel   = 0.0f;
volatile float i_bat     = 0.0f;
volatile float p_panel   = 0.0f;
volatile float soc       = 0.5f;
volatile float e_deliv   = 0.0f;    // Wh entregados al bus (acumulado)
volatile float e_absorb  = 0.0f;    // Wh absorbidos del bus (acumulado)

volatile float rawBus5   = 0.0f;
volatile float rawBat2s  = 0.0f;
volatile float rawSpare  = 0.0f;
volatile float rawPanel  = 0.0f;
volatile float rawIBus   = 0.0f;
volatile float rawIPanel = 0.0f;
volatile float rawIBat   = 0.0f;

volatile uint8_t adsMainAddr = 0x48;
volatile uint8_t adsMpptAddr = 0x49;

volatile float divFactorBus5  = 3.12f;
volatile float divFactorPv2s  = 3.12f;
volatile float divFactorPanel = 3.12f;
volatile float divFactorSpare = 3.12f;

CurrentCal calIBus5  = { 2.50f, 0.185f, -1, CUR_RANGE_5A,  0.30f, 0.04f, 0.0f, false }; // ACS712-05B
CurrentCal calIPanel = { 2.50f, 0.185f, -1, CUR_RANGE_5A,  0.30f, 0.04f, 0.0f, false }; // ACS712-05B
CurrentCal calIBat   = { 2.50f, 0.185f, -1, CUR_RANGE_5A,  0.30f, 0.04f, 0.0f, false };
volatile float boostKclEfficiency = 0.90f;

/* ── Salidas del buckboost ── */
volatile float i_ref       = 0.0f;
volatile float i_ref_v     = 0.0f;
volatile float i_ref_d     = 0.0f;
volatile float dutyCmd     = 0.0f;
volatile float dutyApplied = 0.0f;
volatile float tonCmdUs    = 0.0f;
volatile TopologyMode activeTopology = TOPO_BUCK;

/* ── Parámetros del buckboost ── */
volatile RunMode requestedMode = RUN_OFF;
volatile RunMode runMode       = RUN_OFF;
volatile bool    faultLatched      = false;
volatile uint8_t faultCode         = 0;
volatile bool    pendingFaultWrite = false;
volatile bool    faultsEnabled     = true;

volatile float targetV = 5.0f;

volatile float    iRefVExt   = 0.0f;
volatile uint32_t iRefVExtMs = 0;

/* Planta del buckboost del agente 3 (ajustable; valores de partida) */
volatile float plantL     = 2.8e-3f;
volatile float plantC     = 2200e-6f;
volatile float softStartS = 1.0f;

/* Lazos. plantVdc por defecto 8.4 (en BUCK conmuta desde la batería 2S). */
volatile float omegaV   = 10.0f;
volatile float omegaI   = 100.0f;
volatile float zetaV    = 0.707f;
volatile float zetaI    = 0.707f;
volatile float plantVdc = 8.4f;
volatile float kpV = 0.0311f, kiV = 0.22f, kpI = 0.0470f, kiI = 3.33f;

volatile float kdDroop = 0.1f;
volatile float iRefMax =  1.0f;
volatile float iRefMin = -1.0f;

volatile uint32_t pwmPeriodUs = 50;     // 20 kHz (fuera del rango audible)
volatile float    tonMaxUs    = 42.5f;  // 85% del período (rescalado de 85/100)
volatile float    minOffUs    = 8.0f;   // µs absolutos (bootstrap/dead-time), NO escala con fsw

volatile float overvoltageTrip  = 5.60f;  // bus alto
volatile float undervoltageTrip = 0.0f;   // bus bajo (0 = deshabilitado)
volatile float overcurrentTrip  = 1.50f;  // i_bb abs
volatile float bat2sOvTrip      = 8.70f;
volatile float bat2sUvTrip      = 6.00f;
volatile float socEmptyV        = 6.00f;
volatile float socFullV         = 8.40f;

/* ── PV / MPPT (defaults del firmware base) ── */
volatile PvBoostMode   pvMode        = PV_OFF;
volatile OperationMode operationMode = OP_FULL;
volatile bool    pvArmed         = false;
volatile bool    pvFaultLatched  = false;
volatile uint8_t pvFaultCode     = 0;
volatile bool    pvPendingFaultWrite = false;
volatile bool    pvArmRequest    = false;

volatile float pvDutyCmd       = 0.0f;
volatile float pvDutyApplied   = 0.0f;
volatile float pvDutySoftLimit = 0.0f;

volatile float pvFixedDuty  = 0.05f;
volatile float pvVpvRef     = 4.60f;   // pv_vpvref
volatile float pvVpvRefMin  = 3.40f;   // pv_vpvmin
volatile float pvVpvRefMax  = 5.60f;   // pv_vpvmax
volatile float pvKpVpv      = 0.060f;
volatile float pvKiVpv      = 0.900f;
volatile float pvDutyMin    = 0.00f;
volatile float pvDutyStart  = 0.03f;
volatile float pvDutyMax    = 0.60f;   // pv_dmax
volatile float pvDutySlewPerStep = 0.0015f;  // pv_slew
volatile uint32_t pvSoftstartMs    = 8000;   // pv_ssms
volatile uint32_t pvControlPeriodMs = 50;
volatile float pvPanelTripLow = 2.70f;  // pv_panelmin
volatile float pvBatWarnHigh  = 8.10f;  // pv_batwarn
volatile float pvBatTripHigh  = 8.20f;  // pv_battrip
volatile float pvInputCurrentMax = 2.60f;   // pv_imax
volatile float pvBatteryChargeCurrentMax = 1.80f;  // pv_ibatmax (TRIP duro)
volatile float pvIbatChargeLim = 0.700f;    // pv_ibatlim — límite SOFT de carga PV (A, magnitud). 0 = desactivado
volatile float iBatIdle        = 0.050f;    // ibat_idle — corriente de idle del sistema (A): las baterías
                                            // siempre alimentan los sensores, nunca hay 0 A real. Baseline del
                                            // cero de i_bat (queda partiendo en +iBatIdle).
volatile float pvInputPowerMax = 14.0f;     // pv_pmax
volatile float pvReverseCurrentMax = 0.30f;
volatile float mpptStepV     = 0.040f;  // mppt_step
volatile float mpptEpsPowerW = 0.015f;  // mppt_epsp
volatile float mpptMinPowerW = 0.30f;   // mppt_pmin
volatile uint32_t mpptPeriodMs = 1000;  // mppt_ms
volatile uint32_t mpptReacquireMs = 5000;  // mppt_reacq_ms — sin potencia ⇒ barrido de re-adquisición

volatile bool     calibrationMode = false;
volatile uint32_t currentFaultInhibitUntilMs = 0;
volatile uint32_t calibrationInhibitMs = 60000;

volatile bool savePending = false;

/* ============================================================
   SNAPSHOT
============================================================ */
Snapshot takeSnapshot() {
  Snapshot s{};
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
    s.v_bus5 = v_bus5;   s.v_bat2s = v_bat2s; s.v_panel = v_panel; s.v_spare = v_spare;
    s.i_bb_bus5 = i_bb_bus5; s.i_bb_filt = i_bb_filt; s.i_panel = i_panel; s.i_bat = i_bat;
    s.p_panel = p_panel; s.soc = soc;
    s.e_deliv = e_deliv; s.e_absorb = e_absorb;
    s.i_ref = i_ref; s.i_ref_v = i_ref_v; s.i_ref_d = i_ref_d;
    s.dutyCmd = dutyCmd; s.dutyApplied = dutyApplied; s.tonCmd = tonCmdUs;
    s.pvDutyApplied = pvDutyApplied; s.pvVpvRef = pvVpvRef;
    s.runMode = (int)runMode; s.activeTopology = (int)activeTopology;
    s.pvMode = (int)pvMode; s.operationMode = (int)operationMode;
    s.fault = faultLatched; s.pvFault = pvFaultLatched; s.pvArmed = pvArmed;
    s.faultCode = faultCode; s.pvFaultCode = pvFaultCode;
    s.hwStatus = (int)hwStatus;
    xSemaphoreGive(xMutex);
  }
  return s;
}

/* ============================================================
   UTILIDADES
============================================================ */
float clampF(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}
uint8_t clampU8(int x, uint8_t lo, uint8_t hi) {
  if (x < (int)lo) return lo;
  if (x > (int)hi) return hi;
  return (uint8_t)x;
}
