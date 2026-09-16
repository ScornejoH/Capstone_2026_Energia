#pragma once
/*
 * shared.h — AGENTE 3
 * ─────────────────────────────────────────────────────────────
 * Estado global compartido entre los dos cores del ESP32.
 *   Core 1 (taskControl): buckboost (lazo de corriente cascada, esclavo
 *                          droop) + control PV/MPPT. Escribe mediciones.
 *   Core 0 (taskCAN):      nodo CAN; recibe i_ref_v + config, difunde
 *                          telemetría/faults. Lee snapshot, escribe params.
 *
 * REGLA DEL MUTEX: Core 1 toma con timeout 2–5 ms; Core 0 con 20–30 ms.
 *
 * El buckboost usa EXACTAMENTE el mismo lazo que el agente 2:
 *   PI voltaje (targetV - v_bus5) → i_ref_v   (con anti-windup)
 *   droop      kd·(SoC - 0.5)     → i_ref_d
 *   i_ref = i_ref_v + i_ref_d   (limitado por soft-start)
 *   PI corriente (i_ref - i_bb)  → duty → ton → PWM
 * OJO: en el agente 3 la batería 2S (8.4V) > bus (5V), así que el mapeo
 *   topología↔dirección está INVERTIDO respecto a los agentes 1/2:
 *     entregar al bus (i_ref>0) = BUCK (2S→BUS5)
 *     cargar batería (i_ref<0)  = BOOST (BUS5→2S)
 * ─────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/* ============================================================
   PINES
============================================================ */
static const int PIN_BB_PWM_HIGH = 25;  // QH/high-side, activo LOW
static const int PIN_BB_PWM_LOW  = 26;  // QL/low-side,  activo HIGH
static const int PIN_PVBOOST_PWM = 27;  // PWM boost PV (low-side)
static const int PIN_PVBOOST_EN  = -1;  // sin enable dedicado
static const int PIN_PVBOOST_GND = 14;  // salida LOW fija (referencia aux)
static const int PIN_I2C_SDA     = 21;
static const int PIN_I2C_SCL     = 22;

/* ============================================================
   DEBUG
============================================================ */
#define I2C_DEBUG 0   // 1 = imprime cada lectura del ADS2 (raw + escalada); 0 = silencio
                      // Es macro (no const bool) para que funcione tanto en #if como en if().

/* ============================================================
   ENUMS
============================================================ */
enum RunMode      { RUN_OFF, RUN_ON };
enum TopologyMode { TOPO_BUCK, TOPO_BOOST };
enum HwStatus     { HW_OK = 0, HW_NO_ADS = 1 };

/* Subsistema PV */
enum PvBoostMode  { PV_OFF, PV_SENSORS, PV_FIXED, PV_VPV_PI, PV_MPPT_PO };
enum OperationMode{ OP_FULL, OP_BUCKBOOST_ONLY, OP_PV_ONLY };

/* ============================================================
   MUTEX / ESTADO HW
============================================================ */
extern SemaphoreHandle_t xMutex;
extern volatile HwStatus hwStatus;     // estado del ADS1 (control del buckboost)

/* ============================================================
   CALIBRACIÓN DE CORRIENTE (ACS712) — un sensor cada uno
============================================================ */
enum CurrentRange : uint8_t { CUR_RANGE_5A = 0, CUR_RANGE_20A = 1 };

struct CurrentCal {
  float  zero;       // V en 0 A
  float  sens;       // V/A
  int8_t sign;       // +1 / −1
   CurrentRange range; // 5A / 20A
  float  alpha;      // filtro paso bajo [0..1]
  float  deadband;   // |A| por debajo del cual se reporta 0
  float  filtered;   // estado del filtro
  bool   ready;
};

static inline float currentRangeSensitivity(CurrentRange range) {
   return (range == CUR_RANGE_20A) ? 0.100f : 0.185f;
}

static inline void setCurrentRange(CurrentCal& cal, CurrentRange range) {
   cal.range = range;
   cal.sens = currentRangeSensitivity(range);
   cal.ready = false;
}

static inline CurrentRange inferCurrentRange(float sens) {
   return (sens >= 0.1425f) ? CUR_RANGE_5A : CUR_RANGE_20A;
}

/* ============================================================
   MEDICIONES (Core 1 escribe, Core 0 lee vía snapshot)
============================================================ */
extern volatile float v_bus5;     // BUS5 (lo que regula/comparte el buckboost)
extern volatile float v_bat2s;    // batería 2S (8.4V) — SoC y protección
extern volatile float v_panel;    // VPV
extern volatile float v_spare;    // canal A2 libre
extern volatile float i_bb_bus5;  // corriente buckboost lado bus CRUDA (i_out) — para protección
extern volatile float i_bb_filt;  // corriente buckboost filtrada (~20 Hz) — para dashboard/telemetría
extern volatile float vBusConsensus; // estimación de consenso distribuido del voltaje de bus (opción B)

/* ── Seguridad: detección de conexión al bus (comparación de voltaje) ── */
extern volatile bool  busDisconnected;  // true = mi v_bus5 no coincide con la red → desconectado
extern volatile bool  busChkEnable;     // habilita la seguridad (bus_chk_en)
extern volatile float busChkTol;        // tolerancia de coincidencia de voltaje (V) (bus_chk_tol)

extern volatile float i_panel;    // corriente del panel
extern volatile float i_bat;      // corriente de batería (+ = carga)
extern volatile float p_panel;    // potencia del panel (V·I)
extern volatile float soc;        // [0,1] estimado de v_bat2s
extern volatile float e_deliv;    // energía ENTREGADA al bus acumulada (Wh) — medida en firmware
extern volatile float e_absorb;   // energía ABSORBIDA del bus acumulada (Wh) — medida en firmware

/* ── Crudos antes de divisor / cero (para auto-calibración) ── */
extern volatile float rawBus5;    // ADS1 A0
extern volatile float rawBat2s;   // ADS1 A1
extern volatile float rawSpare;   // ADS1 A2
extern volatile float rawPanel;   // ADS2 A0
extern volatile float rawIBus;    // ADS1 A3 (cruda, V)
extern volatile float rawIPanel;  // ADS2 A2 (cruda, V)
extern volatile float rawIBat;    // ADS2 A3 (cruda, V)

/* ── Direcciones I2C de los dos ADS1115 ── */
extern volatile uint8_t adsMainAddr;  // ADS1: bus/buckboost
extern volatile uint8_t adsMpptAddr;  // ADS2: PV/batería

/* ── Divisores de voltaje ── */
extern volatile float divFactorBus5;
extern volatile float divFactorPv2s;   // batería 2S
extern volatile float divFactorPanel;
extern volatile float divFactorSpare;

/* ── Calibración de las 3 corrientes ── */
extern CurrentCal calIBus5;
extern CurrentCal calIPanel;
extern CurrentCal calIBat;
extern volatile float boostKclEfficiency;

/* ============================================================
   SALIDAS / REFERENCIAS DEL BUCKBOOST (Core 1 escribe)
============================================================ */
extern volatile float i_ref;
extern volatile float i_ref_v;
extern volatile float i_ref_d;
extern volatile float dutyCmd;
extern volatile float dutyApplied;
extern volatile float tonCmdUs;
extern volatile TopologyMode activeTopology;

/* ============================================================
   PARÁMETROS DEL BUCKBOOST (Core 0 escribe desde CAN)
============================================================ */
extern volatile RunMode  requestedMode;
extern volatile RunMode  runMode;
extern volatile bool     faultLatched;
extern volatile uint8_t  faultCode;
extern volatile bool     pendingFaultWrite;
extern volatile bool     faultsEnabled;

extern volatile float targetV;

/* Referencia común del maestro (droop) */
extern volatile float    iRefVExt;
extern volatile uint32_t iRefVExtMs;

/* Planta del buckboost (hardware) — distinta a la de los agentes 1/2 */
extern volatile float plantL;     // H
extern volatile float plantC;     // F
extern volatile float softStartS; // s

/* Lazos: frecuencias de cruce y ganancias derivadas (recomputeGains) */
extern volatile float omegaV;     // ω_n lazo voltaje (rad/s)
extern volatile float omegaI;     // ω_n lazo corriente (rad/s)
extern volatile float zetaV;      // ζ lazo voltaje
extern volatile float zetaI;      // ζ lazo corriente
extern volatile float plantVdc;   // V que conmuta el medio puente (lineariza I)
extern volatile float kpV, kiV, kpI, kiI;

extern volatile float kdDroop;
extern volatile float iRefMax;
extern volatile float iRefMin;

extern volatile uint32_t pwmPeriodUs;
extern volatile float    tonMaxUs;
extern volatile float    minOffUs;

/* Protecciones del buckboost */
extern volatile float overvoltageTrip;   // bus
extern volatile float undervoltageTrip;  // bus
extern volatile float overcurrentTrip;   // i_bb (abs)
extern volatile float bat2sOvTrip;       // batería 2S alta
extern volatile float bat2sUvTrip;       // batería 2S baja
extern volatile float socEmptyV;         // V de 2S a SoC=0
extern volatile float socFullV;          // V de 2S a SoC=1

/* ============================================================
   PARÁMETROS / ESTADO DEL SUBSISTEMA PV / MPPT
============================================================ */
extern volatile PvBoostMode   pvMode;
extern volatile OperationMode operationMode;
extern volatile bool  pvArmed;
extern volatile bool  pvFaultLatched;
extern volatile uint8_t pvFaultCode;
extern volatile bool  pvPendingFaultWrite;
extern volatile bool  pvArmRequest;       // one-shot recibido por CAN

extern volatile float pvDutyCmd;
extern volatile float pvDutyApplied;
extern volatile float pvDutySoftLimit;

extern volatile float pvFixedDuty;
extern volatile float pvVpvRef;
extern volatile float pvVpvRefMin;
extern volatile float pvVpvRefMax;
extern volatile float pvKpVpv;
extern volatile float pvKiVpv;
extern volatile float pvDutyMin;
extern volatile float pvDutyStart;
extern volatile float pvDutyMax;
extern volatile float pvDutySlewPerStep;
extern volatile uint32_t pvSoftstartMs;
extern volatile uint32_t pvControlPeriodMs;
extern volatile float pvPanelTripLow;
extern volatile float pvBatWarnHigh;
extern volatile float pvBatTripHigh;
extern volatile float pvInputCurrentMax;
extern volatile float pvBatteryChargeCurrentMax;
extern volatile float pvIbatChargeLim;
extern volatile float iBatIdle;              // corriente de idle del sistema (A) — baseline del cero de i_bat
extern volatile float pvInputPowerMax;
extern volatile float pvReverseCurrentMax;
extern volatile float mpptStepV;
extern volatile float mpptEpsPowerW;
extern volatile float mpptMinPowerW;
extern volatile uint32_t mpptPeriodMs;
extern volatile uint32_t mpptReacquireMs;   // sin potencia este tiempo ⇒ re-adquirir (barrido)

/* Inhibición de faults de corriente durante calibración */
extern volatile bool     calibrationMode;
extern volatile uint32_t currentFaultInhibitUntilMs;
extern volatile uint32_t calibrationInhibitMs;

/* Señal one-shot: persistir a NVS (la pone taskCAN, la consume taskControl) */
extern volatile bool savePending;

/* ============================================================
   SNAPSHOT — copia atómica para Core 0 (telemetría)
============================================================ */
struct Snapshot {
  float v_bus5, v_bat2s, v_panel, v_spare;
  float i_bb_bus5, i_bb_filt, i_panel, i_bat, p_panel, soc;
  float e_deliv, e_absorb;
  float i_ref, i_ref_v, i_ref_d, dutyCmd, dutyApplied, tonCmd;
  float pvDutyApplied, pvVpvRef;
  int   runMode, activeTopology, pvMode, operationMode;
  bool  fault, pvFault, pvArmed;
  uint8_t faultCode, pvFaultCode;
  int   hwStatus;
};
Snapshot takeSnapshot();

/* ============================================================
   UTILIDADES
============================================================ */
float clampF(float x, float lo, float hi);
uint8_t clampU8(int x, uint8_t lo, uint8_t hi);
