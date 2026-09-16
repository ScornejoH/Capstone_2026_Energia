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
volatile float i_out_filt = 0.0f;
volatile float vBusConsensus = 0.0f;
volatile bool  busDisconnected = false;   // seguridad: mi v_bus no coincide con la red
volatile bool  busChkEnable    = true;
volatile float busChkTol       = 0.30f;   // V — debe superar el error de calibración entre sensores
volatile float soc        = 0.5f;
volatile float e_deliv    = 0.0f;   // Wh entregados al bus (acumulado)
volatile float e_absorb   = 0.0f;   // Wh absorbidos del bus (acumulado)

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

/* Referencia de corriente común recibida del maestro (esclavo droop) */
volatile float    iRefVExt   = 0.0f;
volatile uint32_t iRefVExtMs = 0;

/* Tuneo por ω_n y ζ (ubicación de polos). Defaults: ω_nv=10, ω_ni=100 rad/s,
 * ζ=0.707. recomputeGains() calcula Kp/Ki (L=2.8mH, C=2200uF, Vdc=5):
 *   Kp_v = 2·ζ_v·ω_nv·C = 0.0311   Ki_v = ω_nv²·C   = 0.22
 *   Kp_i = 2·ζ_i·ω_ni·L/Vdc = 0.0792  Ki_i = ω_ni²·L/Vdc = 5.6
 */
volatile float omegaV = 10.0f;     // ω_n lazo voltaje (rad/s)
volatile float omegaI = 100.0f;    // ω_n lazo corriente (rad/s)
volatile float zetaV  = 0.707f;    // ζ lazo voltaje
volatile float zetaI  = 0.707f;    // ζ lazo corriente
volatile float plantVdc = 5.0f;    // V del bus DC que conmuta el medio puente (nominal ≈ targetV)
volatile float kpV = 0.0311f;
volatile float kiV = 0.22f;
volatile float kpI = 0.0792f;
volatile float kiI = 5.6f;

volatile float kdDroop = 0.1f;   // A por unidad de SoC desviado

volatile float iRefMax =  1.0f;
volatile float iRefMin = -1.0f;

volatile uint32_t pwmPeriodUs = 50;    // 20 kHz (fuera del rango audible)
volatile float    tonMaxUs    = 47.5f; // 95% del período (rescalado de 95/100)
volatile float    minOffUs    = 3.0f;  // µs absolutos (bootstrap/dead-time), NO escala con fsw

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
    s.i_out        = i_out;      s.i_out_filt = i_out_filt;   s.soc = soc;
    s.e_deliv      = e_deliv;    s.e_absorb   = e_absorb;
    s.i_ref        = i_ref;      s.i_ref_v    = i_ref_v;    s.i_ref_d  = i_ref_d;
    s.dutyCmd      = dutyCmd;    s.dutyApplied = dutyApplied;
    s.rawVoltsA0   = rawVoltsA0; s.rawVoltsA1 = rawVoltsA1; s.rawVoltsA3 = rawVoltsA3;
    s.divFactorBus = divFactorBus; s.divFactorBat = divFactorBat;
    s.acsOffset      = acsOffset;
    s.acsSign        = acsSign;
    s.acsSensitivity = acsSensitivity;
    s.targetV      = targetV;
    s.omegaV = omegaV; s.omegaI = omegaI; s.zetaV = zetaV; s.zetaI = zetaI; s.plantVdc = plantVdc;
    s.kpV = kpV; s.kiV = kiV; s.kpI = kpI; s.kiI = kiI;
    s.kdDroop = kdDroop; s.iRefMax = iRefMax; s.iRefMin = iRefMin;
    s.ovTrip  = overvoltageTrip; s.uvTrip = undervoltageTrip; s.ocTrip = overcurrentTrip;
    s.pwmPeriod = (float)pwmPeriodUs; s.tonMax = tonMaxUs; s.minOff = minOffUs;
    s.fault          = faultLatched;
    s.faultsEnabled  = faultsEnabled;
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
