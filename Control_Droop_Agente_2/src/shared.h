#pragma once
/*
 * shared.h
 * ─────────────────────────────────────────────────────────────
 * Variables globales compartidas entre los dos cores del ESP32.
 *
 * REGLA DE USO DEL MUTEX:
 *   Core 1 (control):  timeout máximo 2–5 ms
 *   Core 0 (comms):    timeout máximo 30 ms
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

#define TOKEN_REFRESH_MS  2700000UL   // 45 min

/* ============================================================
   PINES
============================================================ */
static const int PIN_PWM_HIGH = 25;
static const int PIN_PWM_LOW  = 26;
static const int PIN_I2C_SDA  = 21;
static const int PIN_I2C_SCL  = 22;

/* ============================================================
   MODOS DE OPERACIÓN
============================================================ */
enum RunMode { RUN_OFF, RUN_ON };

/* Topología activa — determinada automáticamente por el signo de i_ref */
enum TopologyMode { TOPO_BUCK, TOPO_BOOST };

/* ============================================================
   ESTADO DE HARDWARE
============================================================ */
enum HwStatus {
  HW_OK     = 0,
  HW_NO_ADS = 1
};

extern volatile HwStatus hwStatus;

/* ============================================================
   MUTEX GLOBAL
============================================================ */
extern SemaphoreHandle_t xMutex;

/* ============================================================
   MEDICIONES (escritas por Core 1, leídas por Core 0)
============================================================ */
extern volatile float v_bus;        // voltaje bus (V)
extern volatile float v_bat;        // voltaje batería (V)
extern volatile float i_out;        // corriente de salida ACS712 CRUDA (A, + = hacia bus) — para protección
extern volatile float i_out_filt;   // corriente filtrada (~20 Hz) — para dashboard/telemetría
extern volatile float vBusConsensus; // estimación de consenso distribuido del voltaje de bus (opción B)

/* ── Seguridad: detección de conexión al bus (comparación de voltaje) ──
   Cada agente con control ON difunde su v_bus; si el propio no coincide con el
   de ningún par (control ON) → se asume desconectado y se apaga (auto-recupera). */
extern volatile bool  busDisconnected;  // true = mi v_bus no coincide con la red → desconectado
extern volatile bool  busChkEnable;     // habilita la seguridad (bus_chk_en)
extern volatile float busChkTol;        // tolerancia de coincidencia de voltaje (V) (bus_chk_tol)
extern volatile float soc;          // estado de carga estimado [0.0, 1.0]
extern volatile float e_deliv;      // energía ENTREGADA al bus acumulada (Wh) — medida en firmware
extern volatile float e_absorb;     // energía ABSORBIDA del bus acumulada (Wh) — medida en firmware

/* ── Voltajes crudos antes de divFactor (para calibración) ── */
extern volatile float rawVoltsA0;   // A0: bus
extern volatile float rawVoltsA1;   // A1: batería
extern volatile float rawVoltsA3;   // A3: ACS712

/* ── Factores de calibración divisores de voltaje ── */
extern volatile float divFactorBus;
extern volatile float divFactorBat;

/* ── Calibración ACS712 ── */
extern volatile float acsOffset;       // V en 0 A (nominalmente 2.5 V)
extern volatile float acsSign;         // +1.0 ó -1.0
extern volatile float acsSensitivity;  // V/A (nominal 0.185, medir con corriente conocida)

/* ============================================================
   REFERENCIAS Y SALIDAS DEL CONTROL (escritas por Core 1)
============================================================ */
extern volatile float i_ref;        // referencia de corriente total (A)
extern volatile float i_ref_v;      // aporte del PI de voltaje (A)
extern volatile float i_ref_d;      // aporte del droop (A)
extern volatile float dutyCmd;      // duty cycle comandado [0, 1]
extern volatile float dutyApplied;  // duty cycle aplicado al PWM
extern volatile TopologyMode activeTopology;  // topología activa en este ciclo

/* ============================================================
   PARÁMETROS DE CONTROL (escritos por Core 0 desde Firebase)
============================================================ */
extern volatile RunMode  requestedMode;
extern volatile RunMode  runMode;
extern volatile bool     faultLatched;
extern volatile uint8_t  faultCode;         // CAN_FAULT_* del último trip (para difusión CAN)
extern volatile bool     pendingFaultWrite;
extern volatile bool     faultsEnabled;    // false = ignorar protecciones (solo desarrollo)

/* ── Referencia de voltaje ── */
extern volatile float targetV;      // V (defecto 5.0)

/* ── Referencia de corriente común recibida del maestro por CAN (esclavo) ──
 *   La escribe taskCAN (Core 0) al recibir CAN_VAR_IREFV; la lee el control
 *   (Core 1). iRefVExtMs = millis() de la última recepción (para timeout). */
extern volatile float    iRefVExt;
extern volatile uint32_t iRefVExtMs;

/* ── Parámetros de planta (fijos en hardware) ── */
#define PLANT_L_H   2.8e-3f    // inductancia (H)
#define PLANT_C_F   2200e-6f   // capacitancia (F)

/* ── Soft-start: tiempo (s) en que el límite de corriente sube de 0 a pleno
 *   al pasar de OFF→ON. Evita el inrush al entrar con el bus ya levantado. */
#define SOFTSTART_S  1.0f

/* ── Frecuencias de cruce ajustables (rad/s) ──
 *   Planta corriente: Gi(s)=Vdc/(sL)  →  Kp_i = wc_i · L / Vdc   Ki_i = Kp_i · wc_i / 10
 *   Planta voltaje:   Gv(s)=1/(sC)    →  Kp_v = wc_v · C         Ki_v = Kp_v · wc_v / 10
 *   Regla de separación: wc_v ≤ wc_i / 10
 */
/* Ubicación de polos (2.º orden) — ω_n y ζ por lazo. recomputeGains() deriva:
 *   Voltaje  (planta 1/sC):     Kp_v = 2·ζ_v·ω_nv·C     Ki_v = ω_nv²·C
 *   Corriente(planta Vdc/sL):   Kp_i = 2·ζ_i·ω_ni·L/Vdc Ki_i = ω_ni²·L/Vdc
 */
extern volatile float omegaV;   // ω_n lazo voltaje  (rad/s)
extern volatile float omegaI;   // ω_n lazo corriente (rad/s)
extern volatile float zetaV;    // ζ lazo voltaje
extern volatile float zetaI;    // ζ lazo corriente
extern volatile float plantVdc; // tensión del bus DC (V) que conmuta el medio puente; lineariza el lazo de corriente
/* Ganancias calculadas por recomputeGains() */
extern volatile float kpV;
extern volatile float kiV;
extern volatile float kpI;
extern volatile float kiI;

/* ── Ganancia droop ── */
extern volatile float kdDroop;      // A por unidad de SoC desviado de 0.5

/* ── Límites de corriente referencia ── */
extern volatile float iRefMax;      // A (boost máximo)
extern volatile float iRefMin;      // A (buck máximo, valor negativo)

/* ── Parámetros PWM ── */
extern volatile uint32_t pwmPeriodUs;
extern volatile float    tonMaxUs;
extern volatile float    minOffUs;

/* ── Umbrales de protección ── */
extern volatile float overvoltageTrip;    // V en bus
extern volatile float undervoltageTrip;   // V en bus
extern volatile float overcurrentTrip;    // A (valor absoluto)

/* ============================================================
   SNAPSHOT — copia atómica para Core 0
============================================================ */
struct Snapshot {
  float v_bus, v_bat, i_out, i_out_filt, soc;
  float e_deliv, e_absorb;
  float i_ref, i_ref_v, i_ref_d;
  float dutyCmd, dutyApplied;
  float rawVoltsA0, rawVoltsA1, rawVoltsA3;
  float divFactorBus, divFactorBat, acsOffset, acsSign, acsSensitivity;
  float targetV;
  float omegaV, omegaI, zetaV, zetaI, plantVdc;
  float kpV, kiV, kpI, kiI;
  float kdDroop, iRefMax, iRefMin;
  float ovTrip, uvTrip, ocTrip;
  float pwmPeriod, tonMax, minOff;
  bool  fault;
  bool  faultsEnabled;
  int   runMode;
  int   requestedMode;
  int   activeTopology;
  int   hwStatus;
};

Snapshot takeSnapshot();

/* ============================================================
   UTILIDADES
============================================================ */
float clampF(float x, float lo, float hi);
