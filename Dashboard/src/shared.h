#pragma once
/*
 * shared.h
 * ─────────────────────────────────────────────────────────────
 * Variables globales compartidas entre los dos cores del ESP32,
 * tipos, mutex FreeRTOS y funciones utilitarias comunes.
 *
 * REGLA DE USO DEL MUTEX:
 *   Core 1 (control):  timeout máximo 1-5 ms  → no bloquear el lazo
 *   Core 0 (comms):    timeout máximo 30 ms   → operaciones de escritura
 * ─────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/* ============================================================
   CREDENCIALES
============================================================ */
#include "secrets.h"   // WIFI_SSID, WIFI_PASS, FIREBASE_EMAIL, FIREBASE_PASS_FB (ver secrets.example.h)



#define FIREBASE_HOST    "microred-dc-default-rtdb.firebaseio.com"
#define FIREBASE_APIKEY  "AIzaSyDwXigQYn1N0nreSYCJ8h0ta63i1mjIWK8"

#define TOKEN_REFRESH_MS  2700000UL   // 45 min (token dura 1 h)

/* ============================================================
   PINES
============================================================ */
static const int PIN_PWM_HIGH = 25;
static const int PIN_PWM_LOW  = 26;
static const int PIN_I2C_SDA  = 21;
static const int PIN_I2C_SCL  = 22;

/* ============================================================
   TOPOLOGÍA / MODOS
============================================================ */
enum TopologyMode { TOPO_BUCK, TOPO_BOOST };
enum RunMode      { RUN_OFF, RUN_MANUAL, RUN_AUTO_FFPI };

/* ============================================================
   ESTADO DE HARDWARE — visible en ambos cores y en la web
============================================================ */
enum HwStatus {
  HW_OK            = 0,   // todo funciona
  HW_NO_CAN        = 1,   // CAN desconectado, control activo
  HW_NO_ADS        = 2,   // ADS desconectado, control inhibido
  HW_NO_CAN_NO_ADS = 3    // falla crítica
};

extern volatile HwStatus hwStatus;   // definido en shared.cpp

/* ============================================================
   MUTEX GLOBAL
============================================================ */
extern SemaphoreHandle_t xMutex;

/* ============================================================
   VARIABLES COMPARTIDAS  (extern → definidas en shared.cpp)
============================================================ */
extern volatile TopologyMode topology;
extern volatile RunMode      runMode;
extern volatile bool         faultLatched;

extern volatile float v_bus;
extern volatile float v_ag1;
extern volatile float targetV;
extern volatile float kp;
extern volatile float ki;
extern volatile float ffScale;
extern volatile float buckDiodeDrop;
extern volatile float tonMaxUs;
extern volatile float tonHardMaxUs;
extern volatile float minOffUs;
extern volatile float manualTonUs;
extern volatile float tonCmdUs;
extern volatile float tonAppliedUs;   // ton real aplicado al PWM en el último ciclo
extern volatile float maxTonStepUpUs;
extern volatile float maxTonStepDownUs;
extern volatile uint32_t pwmPeriodUs;
extern volatile uint32_t controlPeriodUs;
extern volatile uint32_t fullReadPeriodMs;
extern volatile float divFactorA0;
extern volatile float divFactorA1;

// Voltajes crudos del ADS (antes de aplicar divFactor) — para calibración
extern volatile float rawVoltsA0;   // lectura directa canal A0/A2 en V
extern volatile float rawVoltsA1;   // lectura directa canal A1 en V

// Umbrales de protección
extern volatile float boostBusTripHigh;
extern volatile float boostAg1TripLow;
extern volatile float buckBusTripLow;
extern volatile float buckBusTripHigh;
extern volatile float buckOutTripHigh;

/* ============================================================
   SNAPSHOT — copia atómica para Core 0
============================================================ */
struct Snapshot {
  float v_bus, v_ag1, targetV, tonCmdUs, tonAppliedUs;
  float rawVoltsA0, rawVoltsA1;
  float divFactorA0, divFactorA1;
  float kp, ki, ffScale;
  float tonMaxUs, tonHardMaxUs, minOffUs, manualTonUs;
  float slewUp, slewDown, pwmPeriod_f;
  bool  fault;
  int   runMode;          // modo efectivo del hardware
  int   requestedMode;    // modo deseado por el usuario
  int   topology;         // 0=BUCK  1=BOOST
  int   hwStatus;
};

Snapshot takeSnapshot();

/* ============================================================
   UTILIDADES COMUNES
============================================================ */
float clampF(float x, float lo, float hi);
float slewLimitAsym(float cur, float tgt, float mUp, float mDn);

/* ============================================================
   MODO DESEADO vs MODO EFECTIVO
   ─────────────────────────────────────────────────────────────
   requestedMode : lo que pidió el usuario desde la web.
                   Se puede cambiar siempre, con o sin ADS.
   runMode       : lo que realmente ejecuta el hardware.
                   Control.cpp lo fuerza a OFF si ADS no está.
   La web muestra requestedMode para que el usuario vea que
   su comando fue recibido, y runMode para el estado real.
============================================================ */
extern volatile RunMode requestedMode;

/* ── Flag: Core 1 pide a Core 0 que escriba run_mode=OFF en Firebase ── */
/* Se activa en tripFault(), Core 0 la lee, hace el PATCH y la limpia.  */
extern volatile bool pendingFaultWrite;
