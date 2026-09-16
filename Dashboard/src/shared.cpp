/*
 * shared.cpp
 * ─────────────────────────────────────────────────────────────
 * Definición de todas las variables compartidas entre cores
 * y funciones utilitarias comunes (clamp, slew, snapshot).
 * ─────────────────────────────────────────────────────────────
 */

#include "shared.h"

/* ============================================================
   MUTEX
============================================================ */
SemaphoreHandle_t xMutex = nullptr;

/* ============================================================
   ESTADO DE HARDWARE
============================================================ */
volatile HwStatus hwStatus = HW_OK;

/* ── Modo deseado por el usuario (independiente del ADS) ── */
volatile RunMode requestedMode = RUN_OFF;

/* ── Flag de falla pendiente de escritura en Firebase ── */
volatile bool pendingFaultWrite = false;

/* ============================================================
   VARIABLES COMPARTIDAS — valores por defecto
============================================================ */
volatile TopologyMode topology      = TOPO_BOOST;
volatile RunMode      runMode       = RUN_OFF;
volatile bool         faultLatched  = false;

volatile float v_bus            = 0.0f;
volatile float v_ag1            = 0.0f;
volatile float targetV          = 5.00f;
volatile float kp               = 10.0f;
volatile float ki               = 5.0f;
volatile float ffScale          = 1.50f;
volatile float buckDiodeDrop    = 0.65f;
volatile float tonMaxUs         = 95.0f;
volatile float tonHardMaxUs     = 99.0f;
volatile float minOffUs         = 3.0f;
volatile float manualTonUs      = 10.0f;
volatile float tonCmdUs         = 0.0f;
volatile float tonAppliedUs     = 0.0f;
volatile float maxTonStepUpUs   = 0.5f;
volatile float maxTonStepDownUs = 3.0f;
volatile uint32_t pwmPeriodUs       = 100;
volatile uint32_t controlPeriodUs   = 1163;
volatile uint32_t fullReadPeriodMs  = 200;
volatile float divFactorA0      = 3.05f;
volatile float divFactorA1      = 3.05f;
volatile float rawVoltsA0       = 0.0f;
volatile float rawVoltsA1       = 0.0f;

// Umbrales de protección
volatile float boostBusTripHigh = 5.65f;
volatile float boostAg1TripLow  = 0.00f;
volatile float buckBusTripLow   = 0.00f;
volatile float buckBusTripHigh  = 5.60f;
volatile float buckOutTripHigh  = 4.30f;

/* ============================================================
   SNAPSHOT ATÓMICO
============================================================ */
Snapshot takeSnapshot() {
  Snapshot s{};
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
    s.v_bus        = v_bus;    s.v_ag1        = v_ag1;
    s.targetV      = targetV;  s.tonCmdUs     = tonCmdUs;
    s.tonAppliedUs = tonAppliedUs;
    s.rawVoltsA0   = rawVoltsA0;  s.rawVoltsA1 = rawVoltsA1;
    s.divFactorA0  = divFactorA0; s.divFactorA1 = divFactorA1;
    s.kp          = kp;       s.ki          = ki;
    s.ffScale     = ffScale;
    s.tonMaxUs    = tonMaxUs; s.tonHardMaxUs = tonHardMaxUs;
    s.minOffUs    = minOffUs; s.manualTonUs  = manualTonUs;
    s.slewUp      = maxTonStepUpUs;
    s.slewDown    = maxTonStepDownUs;
    s.pwmPeriod_f = (float)pwmPeriodUs;
    s.fault       = faultLatched;
    s.runMode     = (int)runMode;
    s.requestedMode = (int)requestedMode;
    s.topology    = (int)topology;
    s.hwStatus    = (int)hwStatus;
    xSemaphoreGive(xMutex);
  }
  return s;
}

/* ============================================================
   UTILIDADES COMUNES
============================================================ */
float clampF(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

float slewLimitAsym(float cur, float tgt, float mUp, float mDn) {
  if (tgt > cur + mUp)  return cur + mUp;
  if (tgt < cur - mDn)  return cur - mDn;
  return tgt;
}
