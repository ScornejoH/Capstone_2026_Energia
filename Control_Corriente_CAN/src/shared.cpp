/*
 * shared.cpp
 * ─────────────────────────────────────────────────────────────
 * Definición de variables compartidas y funciones utilitarias.
 * ─────────────────────────────────────────────────────────────
 */

#include "shared.h"

/* ============================================================
   MUTEX
============================================================ */
SemaphoreHandle_t xMutex = nullptr;

/* ============================================================
   HARDWARE STATUS
============================================================ */
volatile HwStatus hwStatus = HW_OK;

/* ============================================================
   MEDICIONES
============================================================ */
volatile float v_bus      = 0.0f;
volatile float v_bat      = 0.0f;
volatile float i_out      = 0.0f;
volatile float soc        = 0.5f;

volatile float rawVoltsA0 = 0.0f;
volatile float rawVoltsA1 = 0.0f;
volatile float rawVoltsA3 = 0.0f;

volatile float divFactorBus = 3.05f;
volatile float divFactorBat = 3.05f;
volatile float acsOffset      = 2.500f;
volatile float acsSign        = -1.0f;
volatile float acsSensitivity = 0.100f;  // V/A — ACS712-20A por defecto (5A = 0.185)

/* ============================================================
   REFERENCIAS Y SALIDAS
============================================================ */
volatile float i_ref         = 0.0f;
volatile float i_ref_v       = 0.0f;
volatile float i_ref_d       = 0.0f;
volatile float dutyCmd       = 0.0f;
volatile float dutyApplied   = 0.0f;
volatile TopologyMode activeTopology = TOPO_BOOST;

/* ============================================================
   PARÁMETROS DE CONTROL
============================================================ */
volatile RunMode requestedMode = RUN_OFF;
volatile RunMode runMode       = RUN_OFF;
volatile bool    faultLatched      = false;
volatile uint8_t faultCode         = 0;       // 0 = sin fault / genérico
volatile bool    pendingFaultWrite = false;
volatile bool    faultsEnabled     = true;

volatile float targetV  = 5.0f;

/* wc_v=314 rad/s → Kp_v=0.691  Ki_v=21.7
 * wc_i=6283 rad/s → Kp_i=17.59  Ki_i=11060
 */
volatile float omegaV = 10.0f;    // 2π·10 rad/s  — lazo voltaje lento (~10 Hz)
volatile float omegaI = 100.0f;   // 2π·50 rad/s  — lazo corriente (10× voltaje)
volatile float plantVdc = 5.0f;    // V del bus DC que conmuta el medio puente (nominal ≈ targetV)
volatile float kpV = 0.691f;
volatile float kiV = 21.7f;
volatile float kpI = 17.59f;
volatile float kiI = 11060.0f;

volatile float kdDroop = 0.1f;   // A por unidad de SoC desviado

volatile float iRefMax =  1.0f;
volatile float iRefMin = -1.0f;

volatile uint32_t pwmPeriodUs = 100;   // 10 kHz
volatile float    tonMaxUs    = 95.0f;
volatile float    minOffUs    = 3.0f;

volatile float overvoltageTrip  = 7.5f;
volatile float undervoltageTrip = 0.0f;
volatile float overcurrentTrip  = 1.5f;

/* ============================================================
   SNAPSHOT ATÓMICO
============================================================ */
Snapshot takeSnapshot() {
  Snapshot s{};
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
    s.v_bus        = v_bus;      s.v_bat      = v_bat;
    s.i_out        = i_out;      s.soc        = soc;
    s.i_ref        = i_ref;      s.i_ref_v    = i_ref_v;    s.i_ref_d  = i_ref_d;
    s.dutyCmd      = dutyCmd;    s.dutyApplied = dutyApplied;
    s.rawVoltsA0   = rawVoltsA0; s.rawVoltsA1 = rawVoltsA1; s.rawVoltsA3 = rawVoltsA3;
    s.divFactorBus = divFactorBus; s.divFactorBat = divFactorBat;
    s.acsOffset      = acsOffset;
    s.acsSign        = acsSign;
    s.acsSensitivity = acsSensitivity;
    s.targetV      = targetV;
    s.omegaV = omegaV; s.omegaI = omegaI; s.plantVdc = plantVdc;
    s.kpV = kpV; s.kiV = kiV; s.kpI = kpI; s.kiI = kiI;
    s.kdDroop = kdDroop; s.iRefMax = iRefMax; s.iRefMin = iRefMin;
    s.ovTrip  = overvoltageTrip; s.uvTrip = undervoltageTrip; s.ocTrip = overcurrentTrip;
    s.pwmPeriod = (float)pwmPeriodUs; s.tonMax = tonMaxUs; s.minOff = minOffUs;
    s.fault          = faultLatched;
    s.faultsEnabled  = faultsEnabled;
    s.faultCode      = (int)faultCode;
    s.runMode        = (int)runMode;
    s.requestedMode  = (int)requestedMode;
    s.activeTopology = (int)activeTopology;
    s.hwStatus       = (int)hwStatus;
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
