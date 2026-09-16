/*
 * control.cpp — AGENTE 3
 * ─────────────────────────────────────────────────────────────
 * Core 1. Dos lazos:
 *  (1) BUCKBOOST bidireccional = ESCLAVO droop, MISMO lazo de corriente que
 *      el agente 2: PI voltaje (anti-windup) → i_ref_v ; droop → i_ref_d ;
 *      PI corriente → duty → ton → PWM. Sigue i_ref_v del maestro; si caduca,
 *      corre su propio PI (Opción 3). Soft-start al activar.
 *      Topología INVERTIDA (2S 8.4V > bus 5V):
 *        i_ref>0 (entregar al bus) = BUCK 2S→BUS5
 *        i_ref<0 (cargar batería)  = BOOST BUS5→2S
 *  (2) BOOST PV / MPPT (independiente, portado del firmware base).
 * ─────────────────────────────────────────────────────────────
 */

#include "control.h"
#include "sensors.h"
#include "can_bus.h"
#include "config.h"
#include <math.h>
#include "driver/gpio.h"

/* ============================================================
   PWM LEDC — 3 canales
============================================================ */
static const uint8_t CH_BB_HIGH = 0;
static const uint8_t CH_BB_LOW  = 1;
static const uint8_t CH_PV      = 2;
static const uint8_t PWM_BITS   = 10;
static const uint32_t DUTY_MAX  = (1UL << PWM_BITS) - 1;

static bool bbPwmReady = false;
static bool pvPwmReady = false;

/* ============================================================
   SALIDA PWM SEGURA EN ARRANQUE
   Buckboost OFF: QH=HIGH (activo-LOW ⇒ OFF), QL=LOW (activo-HIGH ⇒ OFF).
   Boost PV OFF: LOW (activo-HIGH, duty 0 ⇒ OFF).
   Se llama como PRIMERA línea de setup(), antes de Serial/tareas, para acortar
   al mínimo la ventana en que los gates flotan tras el reset. Los pull internos
   (~45 kΩ) en la dirección segura son un respaldo: solo actúan si el pin queda
   en alta-Z; NO cubren el arranque previo a setup(). Para cubrir el reset
   completo, reflasheo y brownout hacen falta pull EXTERNOS en el driver.
============================================================ */
void pwmForceSafe() {
  pinMode(PIN_BB_PWM_HIGH, OUTPUT); digitalWrite(PIN_BB_PWM_HIGH, HIGH);   // QH activo-LOW → OFF
  pinMode(PIN_BB_PWM_LOW,  OUTPUT); digitalWrite(PIN_BB_PWM_LOW,  LOW);    // QL activo-HIGH → OFF
  pinMode(PIN_PVBOOST_PWM, OUTPUT); digitalWrite(PIN_PVBOOST_PWM, LOW);    // boost PV → OFF
  pinMode(PIN_PVBOOST_GND, OUTPUT); digitalWrite(PIN_PVBOOST_GND, LOW);    // GND del boost PV
  if (PIN_PVBOOST_EN >= 0) { pinMode(PIN_PVBOOST_EN, OUTPUT); digitalWrite(PIN_PVBOOST_EN, LOW); }  // EN → deshabilitado
  gpio_set_pull_mode((gpio_num_t)PIN_BB_PWM_HIGH, GPIO_PULLUP_ONLY);       // si flota → HIGH (OFF)
  gpio_set_pull_mode((gpio_num_t)PIN_BB_PWM_LOW,  GPIO_PULLDOWN_ONLY);     // si flota → LOW  (OFF)
  gpio_set_pull_mode((gpio_num_t)PIN_PVBOOST_PWM, GPIO_PULLDOWN_ONLY);     // si flota → LOW  (OFF)
  gpio_set_pull_mode((gpio_num_t)PIN_PVBOOST_GND, GPIO_PULLDOWN_ONLY);
  if (PIN_PVBOOST_EN >= 0) gpio_set_pull_mode((gpio_num_t)PIN_PVBOOST_EN, GPIO_PULLDOWN_ONLY);
}

/* ── PV: estado interno de control ── */
static uint32_t pvModeStartMs = 0;
static uint32_t lastMpptMs    = 0;
static uint32_t lastPvSenseMs = 0;
static bool     mpptInitialized = false;
static float    mpptPrevPowerW  = 0.0f;
static float    mpptDirection   = -1.0f;
static float    pvPiVpvInt      = 0.03f;
static uint32_t lastMpptGoodMs  = 0;              // última vez con potencia ≥ mppt_pmin
static float    pvChargeCeil    = 1.0f;           // techo de duty por límite de carga de batería
static bool     pvSweeping      = false;          // re-adquisición: barrido de referencia en curso
static const uint32_t PV_SENSE_PERIOD_MS  = 120;  // cadencia de muestreo del ADS2 (~8 Hz)
static const float    PV_SWEEP_REF_STEP   = 0.03f; // V por ciclo de control durante el barrido de re-adquisición

static void initPwm() {
  uint32_t period;
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    period = pwmPeriodUs; xSemaphoreGive(xMutex);
  } else period = 100;
  uint32_t freq = period ? 1000000UL / period : 20000;
  ledcSetup(CH_BB_HIGH, freq, PWM_BITS); ledcAttachPin(PIN_BB_PWM_HIGH, CH_BB_HIGH);
  ledcSetup(CH_BB_LOW,  freq, PWM_BITS); ledcAttachPin(PIN_BB_PWM_LOW,  CH_BB_LOW);
  bbPwmReady = true;
  ledcSetup(CH_PV, 20000, PWM_BITS);     ledcAttachPin(PIN_PVBOOST_PWM, CH_PV);
  ledcWrite(CH_PV, 0);
  pvPwmReady = true;
}

/* Buckboost apagado: QH HIGH (activo-LOW ⇒ OFF), QL 0 (OFF). */
static void bbAllOff() {
  if (bbPwmReady) { ledcWrite(CH_BB_HIGH, DUTY_MAX); ledcWrite(CH_BB_LOW, 0); }
  else {
    pinMode(PIN_BB_PWM_HIGH, OUTPUT); pinMode(PIN_BB_PWM_LOW, OUTPUT);
    digitalWrite(PIN_BB_PWM_HIGH, HIGH); digitalWrite(PIN_BB_PWM_LOW, LOW);
  }
}

static uint32_t dutyFromUs(float ton, float period) {
  if (ton <= 0.0f || period <= 0.0f) return 0;
  uint32_t d = (uint32_t)lroundf((ton / period) * (float)DUTY_MAX);
  return d > DUTY_MAX ? DUTY_MAX : d;
}
/* BUCK 2S→BUS5: QH PWM activo-bajo, QL OFF. */
static void driveBuck(float ton, float period, float maxTon) {
  ton = clampF(ton, 0.0f, maxTon);
  ledcWrite(CH_BB_HIGH, DUTY_MAX - dutyFromUs(ton, period));
  ledcWrite(CH_BB_LOW, 0);
}
/* BOOST BUS5→2S: QL PWM activo-alto, QH OFF. */
static void driveBoost(float ton, float period, float maxTon) {
  ton = clampF(ton, 0.0f, maxTon);
  ledcWrite(CH_BB_HIGH, DUTY_MAX);
  ledcWrite(CH_BB_LOW, dutyFromUs(ton, period));
}

/* ── PV PWM ── */
static uint32_t dutyToCounts(float duty) {
  return (uint32_t)lroundf(clampF(duty, 0.0f, 1.0f) * (float)DUTY_MAX);
}
static void pvWriteDuty(float duty) {
  if (pvPwmReady) ledcWrite(CH_PV, dutyToCounts(duty));
  pvDutyApplied = clampF(duty, 0.0f, 1.0f);
}
static void pvAllOff() {
  pvWriteDuty(0.0f);
  pvArmed = false;
}

/* ============================================================
   GANANCIAS PI
============================================================ */
void recomputeGains() {
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
    float vdc = plantVdc < 0.5f ? 0.5f : plantVdc;
    /* Ubicación de polos 2.º orden: lazo cerrado = s²+2ζω_n s+ω_n² */
    kpV = 2.0f * zetaV * omegaV * plantC;
    kiV = omegaV * omegaV * plantC;
    kpI = 2.0f * zetaI * omegaI * plantL / vdc;
    kiI = omegaI * omegaI * plantL / vdc;
    xSemaphoreGive(xMutex);
  }
}

/* ============================================================
   FAULTS
============================================================ */
void tripFault(const char* reason, uint8_t code) {
  bbAllOff();
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    faultLatched  = true; faultCode = code;
    runMode = RUN_OFF; requestedMode = RUN_OFF;
    xSemaphoreGive(xMutex);
  }
  pendingFaultWrite = true;
  Serial.printf("[BB][FAULT] %s (code=%u)\n", reason, code);
}

void pvTripFault(const char* reason, uint8_t code) {
  pvAllOff();
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    pvFaultLatched = true; pvFaultCode = code; pvMode = PV_OFF;
    xSemaphoreGive(xMutex);
  }
  pvPendingFaultWrite = true;
  Serial.printf("[PV][FAULT] %s (code=%u)\n", reason, code);
}

static bool currentFaultsInhibited() {
  if (calibrationMode) return true;
  if (currentFaultInhibitUntilMs == 0) return false;
  return ((int32_t)(millis() - currentFaultInhibitUntilMs) < 0);
}

/* ── Protecciones del buckboost ── */
static bool checkBbProtections() {
  if (!faultsEnabled) return true;
  float vb = v_bus5, vbat = v_bat2s, ib = i_bb_bus5;
  if (vb > overvoltageTrip)                     { tripFault("Bus sobre-voltaje", CAN_FAULT_OVERV); return false; }
  if (undervoltageTrip > 0.01f && vb < undervoltageTrip) { tripFault("Bus sub-voltaje", CAN_FAULT_UNDERV); return false; }
  if (!currentFaultsInhibited() && fabsf(ib) > overcurrentTrip) { tripFault("Sobre-corriente i_bb", CAN_FAULT_OVERC); return false; }
  if (vbat > bat2sOvTrip)                        { tripFault("Batería 2S alta", CAN_FAULT_BATOV); return false; }
  if (vbat > 1.0f && vbat < bat2sUvTrip)         { tripFault("Batería 2S baja", CAN_FAULT_BATUV); return false; }
  return true;
}

/* ============================================================
   SUBSISTEMA PV / MPPT  (portado del firmware base)
============================================================ */
static bool pvBoostEnabledByMode() {
  return operationMode == OP_FULL || operationMode == OP_PV_ONLY;
}
static bool bbEnabledByMode() {
  return operationMode == OP_FULL || operationMode == OP_BUCKBOOST_ONLY;
}

static float pvSoftstartDutyLimit() {
  uint32_t elapsed = millis() - pvModeStartMs;
  if (pvSoftstartMs == 0) return pvDutyMax;
  float k = clampF((float)elapsed / (float)pvSoftstartMs, 0.0f, 1.0f);
  return pvDutyStart + k * (pvDutyMax - pvDutyStart);
}
static float pvApplyDutyLimits(float target) {
  pvDutySoftLimit = pvSoftstartDutyLimit();
  float hi = fminf(pvDutyMax, pvDutySoftLimit);
  hi = fminf(hi, pvChargeCeil);                   // techo por límite de corriente de carga
  float limited = clampF(target, pvDutyMin, hi);
  float step = fmaxf(pvDutySlewPerStep, 0.0005f);
  limited = clampF(limited, pvDutyApplied - step, pvDutyApplied + step);
  return clampF(limited, 0.0f, pvDutyMax);
}

/* Punto de arranque seguro del MPP: interior al rango (nunca en los rieles
   Voc/colapso), para que P&O siempre parta de un punto que entrega potencia. */
static float pvMpptSafeStartRef() {
  return clampF(0.5f * (pvVpvRefMin + pvVpvRefMax), pvVpvRefMin, pvVpvRefMax);
}

/* Limitador SOFT de corriente de carga de batería: baja el techo de duty (menos
   potencia del panel) si la carga supera pvIbatChargeLim, y lo libera si hay
   holgura. i_bat<0 = cargando. Integral → sin chattering. */
static void pvUpdateChargeCeiling(float Ts) {
  if (pvIbatChargeLim <= 0.0f) { pvChargeCeil = pvDutyMax; return; }   // desactivado
  float over = (-pvIbatChargeLim) - i_bat;        // >0 si i_bat < -lim (carga excesiva)
  const float kLim = 0.5f;                         // ganancia integral del techo (duty/(A·s))
  pvChargeCeil -= kLim * over * Ts;
  pvChargeCeil = clampF(pvChargeCeil, pvDutyMin, pvDutyMax);
}
static bool pvSafetyOk() {
  if (v_bat2s > pvBatTripHigh)            { pvTripFault("VBAT2S alta", PVF_BAT_HI); return false; }
  if (v_panel < pvPanelTripLow)           { pvTripFault("VPV colapsado", PVF_VPV_LOW); return false; }
  if (!currentFaultsInhibited()) {
    if (i_panel > pvInputCurrentMax)         { pvTripFault("IPV alta", PVF_IPV_HI); return false; }
    if (i_bat > pvBatteryChargeCurrentMax)   { pvTripFault("IBAT carga alta", PVF_IBAT_HI); return false; }
    if (i_panel < -pvReverseCurrentMax)      { pvTripFault("IPV reversa/signo", PVF_IPV_REV); return false; }
    if (p_panel > pvInputPowerMax)           { pvTripFault("PPV alta", PVF_PPV_HI); return false; }
  }
  return true;
}
static void pvApplyDutyCommand(float duty) {
  pvDutyCmd = duty;
  if (!pvArmed || pvMode == PV_OFF || pvMode == PV_SENSORS || pvFaultLatched) {
    pvWriteDuty(0.0f); return;
  }
  if (!pvSafetyOk()) return;
  if (v_bat2s >= pvBatWarnHigh && duty > pvDutyApplied) duty = pvDutyApplied;
  pvWriteDuty(pvApplyDutyLimits(duty));
}
static void pvClearControlStates() {
  pvPiVpvInt = pvDutyStart;
  pvDutyCmd = 0.0f; pvDutySoftLimit = 0.0f;
  mpptInitialized = false; mpptPrevPowerW = 0.0f; mpptDirection = -1.0f;
  pvModeStartMs = millis(); lastMpptMs = millis(); lastPvSenseMs = 0;
  lastMpptGoodMs = millis(); pvChargeCeil = pvDutyMax; pvSweeping = false;
}
static void pvRunVpvPi() {
  float Ts = (float)pvControlPeriodMs / 1000.0f;
  float e = v_panel - pvVpvRef;
  float iCand = pvPiVpvInt + pvKiVpv * e * Ts;
  float uCand = pvKpVpv * e + iCand;
  bool allowI = ((uCand <= pvDutyMax) && (uCand >= pvDutyMin)) ||
                ((uCand > pvDutyMax) && (e < 0.0f)) ||
                ((uCand < pvDutyMin) && (e > 0.0f));
  if (allowI) pvPiVpvInt = iCand;
  pvApplyDutyCommand(pvKpVpv * e + pvPiVpvInt);
}
static void pvUpdateMpptPO() {
  if (!pvArmed) return;
  uint32_t now = millis();

  /* Sin potencia (colapso / punto muerto). Tras mpptReacquireMs arrancamos un
     BARRIDO de re-adquisición (lo ejecuta el dispatcher con pvRunSweep): baja la
     referencia buscando un punto vivo, sin importar dónde quedó el MPP. */
  if (p_panel < mpptMinPowerW) {
    if (!pvSweeping && (uint32_t)(now - lastMpptGoodMs) > mpptReacquireMs) {
      pvSweeping = true;
      Serial.println("[PV][MPPT] sin potencia → barrido de re-adquisición");
    }
    return;
  }
  lastMpptGoodMs = now;
  if (pvSweeping) { pvSweeping = false; mpptInitialized = false; }   // salir del barrido limpio

  /* Mientras el limitador de carga de batería recorta el duty, el punto de
     operación NO refleja el MPP real: congelamos P&O para no derivar. */
  if (pvChargeCeil < pvDutyMax - 0.02f) { mpptPrevPowerW = p_panel; return; }

  if ((now - lastMpptMs) < mpptPeriodMs) return;
  if (!mpptInitialized) { mpptPrevPowerW = p_panel; mpptInitialized = true; lastMpptMs = now; return; }

  float dP = p_panel - mpptPrevPowerW;
  /* Banda muerta de ruido: si el cambio es minúsculo, MANTENEMOS la posición (no
     perturbamos) → no derivamos por la cima plana. Este era el bug: antes seguía
     perturbando en la misma dirección y caminaba hasta Voc hasta colapsar. */
  if (fabsf(dP) <= mpptEpsPowerW) { mpptPrevPowerW = p_panel; lastMpptMs = now; return; }
  /* Cambio significativo: P&O clásico → invertir SIEMPRE que la potencia cayó. */
  if (dP < 0.0f) mpptDirection *= -1.0f;
  float newRef = clampF(pvVpvRef + mpptDirection * mpptStepV, pvVpvRefMin, pvVpvRefMax);
  if ((newRef <= pvVpvRefMin && mpptDirection < 0.0f) ||
      (newRef >= pvVpvRefMax && mpptDirection > 0.0f)) mpptDirection *= -1.0f;
  pvVpvRef = newRef; mpptPrevPowerW = p_panel; lastMpptMs = now;
}

/* Barrido de re-adquisición (lazo abierto sobre la referencia): baja pvVpvRef
   gradualmente forzando más corriente del panel hasta que aparezca potencia;
   ahí entrega el control a P&O en ese punto vivo (bumpless). Garantiza relumbrar
   sin importar las condiciones (reemplaza el salto ciego a un punto fijo). */
static void pvRunSweep() {
  pvVpvRef -= PV_SWEEP_REF_STEP;
  if (pvVpvRef < pvVpvRefMin) pvVpvRef = pvVpvRefMax;   // reinicia el barrido desde arriba (Voc)
  pvRunVpvPi();                                          // el PI regula el panel a pvVpvRef
  if (p_panel >= mpptMinPowerW) {                        // punto vivo encontrado
    pvSweeping = false; mpptInitialized = false; mpptDirection = -1.0f;
    lastMpptGoodMs = millis();
    Serial.println("[PV][MPPT] re-adquirido → P&O");
  }
}
/* Dispatcher PV: lee el ADS2 a cadencia reducida (PV_SENSE_PERIOD_MS) y corre
   el modo activo. El ADS2 se lee SIEMPRE —aunque el PV esté OFF/SENSORS— para
   que lleguen las mediciones de panel/batería a telemetría y haya datos frescos
   para calibrar el cero de i_bat. Solo los modos que ENTREGAN potencia
   (FIXED/VPV/MPPT) tratan una lectura fallida como fault. */
static void updatePvControl() {
  uint32_t now = millis();
  bool active = pvBoostEnabledByMode() && !pvFaultLatched &&
                (pvMode == PV_FIXED || pvMode == PV_VPV_PI || pvMode == PV_MPPT_PO);

  if ((uint32_t)(now - lastPvSenseMs) >= PV_SENSE_PERIOD_MS) {
    lastPvSenseMs = now;
    bool ok = sensorsUpdatePv();                 // monitoreo + calibración
    if (!ok && active) { pvTripFault("lectura ADS2 falló", CAN_FAULT_SENSOR); return; }
  }

  /* Control del boost PV solo en modos activos. */
  if (!pvBoostEnabledByMode() || pvFaultLatched || pvMode == PV_OFF) { pvAllOff(); return; }
  if (pvMode == PV_SENSORS) { pvWriteDuty(0.0f); return; }
  pvUpdateChargeCeiling((float)pvControlPeriodMs / 1000.0f);   // límite de carga de batería
  if (pvMode == PV_FIXED)        pvApplyDutyCommand(pvFixedDuty);
  else if (pvMode == PV_VPV_PI)  pvRunVpvPi();
  else if (pvMode == PV_MPPT_PO){ pvUpdateMpptPO(); if (pvSweeping) pvRunSweep(); else pvRunVpvPi(); }
}

/* ============================================================
   MINI SMART METERING — energía real integrada en firmware (Wh)
   P = v_bus5 · i_bb_filt (potencia del buckboost en el bus). Entregada si P>0,
   absorbida si P<0. Decimado (4 Hz) + banda muerta + guarda contra huecos
   largos. Acumuladores en double: sin pérdida de precisión en corridas largas.
============================================================ */
static void accumulateEnergy(float vbus, float iout) {
  static double   eD = 0.0, eA = 0.0;
  static uint32_t lastMs = 0;
  const uint32_t  PERIOD_MS = 250;
  const float     IDEADBAND = 0.02f;   // A
  uint32_t now = millis();
  if (lastMs == 0) { lastMs = now; return; }
  uint32_t dtm = now - lastMs;
  if (dtm < PERIOD_MS) return;          // decimación
  lastMs = now;
  if (dtm > 2000) return;               // hueco largo → discontinuidad, no integrar
  if (fabsf(iout) < IDEADBAND) return;  // sin flujo neto → no integrar offset
  float dtH = (float)dtm / 3600000.0f;
  float p   = vbus * iout;
  if (p >= 0.0f) eD += (double)p * dtH; else eA += (double)(-p) * dtH;
  e_deliv  = (float)eD;
  e_absorb = (float)eA;
}

/* ============================================================
   TAREA — Core 1
============================================================ */
void taskControl(void* pvParams) {
  /* Cargar config persistida (calibración, params) ANTES de habilitar nada */
  loadConfig();

  bool adsOk = sensorsInit();
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
    hwStatus = adsOk ? HW_OK : HW_NO_ADS; xSemaphoreGive(xMutex);
  }

  initPwm();
  bbAllOff(); pvAllOff();
  recomputeGains();

  float integV = 0.0f, integI = 0.0f, iOutFilt = 0.0f, softStart = 0.0f;
  TopologyMode prevTopo = TOPO_BUCK;       // arranca en modo "entrega al bus"
  PvBoostMode  prevPvMode = PV_OFF;

  static const uint32_t ADS_RETRY_MS = 5000;
  static const uint8_t  ADS_FAIL_THRESH = 5;
  uint8_t adsFailCount = 0; uint32_t adsLastRetry = 0;
  uint32_t lastPvMs = millis();
  uint32_t lastMs = millis();

  for (;;) {
    /* El yield al IDLE (watchdog) lo hace waitReady con vTaskDelay(1) DURANTE
       la conversión del ADS (camino común del buckboost) → el lazo sube a
       ~700 Hz. Las ramas que NO leen sensores ceden explícitamente. */
    uint32_t now = millis();
    float dt = clampF((float)(now - lastMs) * 0.001f, 0.001f, 0.050f);
    lastMs = now;

    /* Metering (usa los valores del ciclo previo; se auto-decima a 4 Hz). */
    accumulateEnergy(v_bus5, i_bb_filt);

    /* Persistencia deshabilitada: AG3 no usa NVS; su config vive en Firebase. */

    /* ── Detectar cambios de modo PV pedidos por CAN ── */
    if (pvMode != prevPvMode) { pvClearControlStates(); prevPvMode = pvMode;
                                if (pvMode == PV_OFF || pvMode == PV_SENSORS) pvArmed = false;
                                if (pvMode == PV_MPPT_PO) pvVpvRef = pvMpptSafeStartRef(); }
    if (pvArmRequest) {
      pvArmRequest = false;
      if (pvBoostEnabledByMode() && !calibrationMode && pvMode != PV_OFF) {
        if (sensorsUpdatePv() && v_panel >= pvPanelTripLow && v_bat2s <= pvBatTripHigh) {
          pvFaultLatched = false; pvClearControlStates();
          if (pvMode == PV_MPPT_PO) pvVpvRef = pvMpptSafeStartRef();   // arranca desde punto vivo
          pvArmed = true;
          Serial.println("[PV] armado");
        } else Serial.println("[PV] arm rechazado (prearm FAIL)");
      }
    }

    /* ===========================================================
       (1) BUCKBOOST — lazo de corriente cascada (esclavo droop)
    =========================================================== */
    if (bbEnabledByMode()) {
      if (!adsOk) {
        if (millis() - adsLastRetry >= ADS_RETRY_MS) {
          adsLastRetry = millis();
          adsOk = sensorsRetry();
          if (adsOk) { adsFailCount = 0;
            if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) { hwStatus = HW_OK; xSemaphoreGive(xMutex); }
            Serial.println("[SENS] ADS1 reconectado"); }
        }
        bbAllOff();
        vTaskDelay(1);   // rama sin lectura de sensores → ceder CPU al IDLE
      } else if (!sensorsUpdateBuckboost()) {
        if (++adsFailCount >= ADS_FAIL_THRESH) {
          adsOk = false; adsFailCount = 0; adsLastRetry = millis();
          if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) { hwStatus = HW_NO_ADS; xSemaphoreGive(xMutex); }
          tripFault("ADS1 desconectado", CAN_FAULT_SENSOR);
        }
        bbAllOff();
        vTaskDelay(1);   // por si el fallo fue antes del yield de readChannel
      } else {
        adsFailCount = 0;
        /* Filtro paso bajo ~20 Hz de la corriente — cada ciclo (para dashboard
           FILTRADO + lazo interno). */
        {
          const float tau = 1.0f / (2.0f * (float)M_PI * 20.0f);
          float alpha = dt / (tau + dt);
          iOutFilt += alpha * (i_bb_bus5 - iOutFilt);
          i_bb_filt = iOutFilt;
        }
        if (!checkBbProtections()) { integV = integI = 0.0f; bbAllOff(); }
        else {
          /* Leer estado/params bajo mutex */
          RunMode rmode; bool fault; float tgtV, kpv, kiv, kpi, kii, kd, irMax, irMin;
          float vBus, iOut, soc_, period, maxTon, minOff;
          if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            rmode = requestedMode; fault = faultLatched; tgtV = targetV;
            kpv = kpV; kiv = kiV; kpi = kpI; kii = kiI; kd = kdDroop;
            irMax = iRefMax; irMin = iRefMin;
            vBus = v_bus5; iOut = i_bb_bus5; soc_ = soc;
            period = (float)pwmPeriodUs; maxTon = tonMaxUs; minOff = minOffUs;
            xSemaphoreGive(xMutex);

            /* busDisconnected (seguridad): si mi v_bus5 no coincide con la red
               asumo que el buckboost no está en el bus → no inyecto (auto-recupera). */
            if (fault || rmode == RUN_OFF || busDisconnected) {
              bbAllOff(); integV = integI = 0.0f; softStart = 0.0f;
              if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                runMode = RUN_OFF; i_ref = i_ref_v = i_ref_d = 0.0f;
                dutyCmd = dutyApplied = tonCmdUs = 0.0f; xSemaphoreGive(xMutex);
              }
            } else {
              /* (La corriente ya viene filtrada en iOutFilt, calculado arriba.) */

              /* Soft-start primero (escala límites) */
              softStart = (softStart < 1.0f) ? (softStart + dt / softStartS) : 1.0f;
              float irMaxEff = irMax * softStart;
              float irMinEff = irMin * softStart;

              /* PI de voltaje con respaldo autónomo + anti-windup */
              float errV = tgtV - vBus;
              float iRefV;
              bool fresh = ((uint32_t)(millis() - iRefVExtMs) <= IREFV_TIMEOUT_MS);
              if (fresh) {
                iRefV = iRefVExt;
                if (kiv > 1e-6f) integV = (iRefV - kpv * errV) / kiv;   // precarga bumpless
              } else {
                integV += errV * dt;
                iRefV = kpv * errV + kiv * integV;
                if (kiv > 1e-6f) {
                  if      (iRefV > irMaxEff) { integV = (irMaxEff - kpv * errV) / kiv; iRefV = irMaxEff; }
                  else if (iRefV < irMinEff) { integV = (irMinEff - kpv * errV) / kiv; iRefV = irMinEff; }
                }
              }

              float iRefD = kd * (soc_ - 0.5f);
              float iRef  = clampF(iRefV + iRefD, irMinEff, irMaxEff);

              /* Topología con histéresis — INVERTIDA respecto a agentes 1/2:
                 entregar al bus (iRef>0) = BUCK ; cargar batería (iRef<0) = BOOST */
              const float HYST = 0.1f;
              TopologyMode topo = prevTopo;
              if (prevTopo == TOPO_BUCK) { if (iRef < -HYST) topo = TOPO_BOOST; }
              else                       { if (iRef >  HYST) topo = TOPO_BUCK;  }
              if (topo != prevTopo) { integI = 0.0f; prevTopo = topo; }

              /* PI interno de corriente → duty */
              float errI = iRef - iOutFilt;
              integI += errI * dt;
              float duty = kpi * errI + kii * integI;

              float maxAllowed = max(0.0f, min(maxTon, period - minOff));
              float ton = clampF(fabsf(duty) * period, 0.0f, maxAllowed);
              float dutyMax = (period > 0.0f) ? maxAllowed / period : 0.0f;
              if (kii > 1e-9f) integI = clampF(integI, -dutyMax / kii, dutyMax / kii);

              if (topo == TOPO_BUCK) driveBuck(ton, period, maxAllowed);
              else                   driveBoost(ton, period, maxAllowed);

              if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                runMode = RUN_ON; i_ref_v = iRefV; i_ref_d = iRefD; i_ref = iRef;
                dutyCmd = fabsf(duty); dutyApplied = ton / period; tonCmdUs = ton;
                activeTopology = topo; xSemaphoreGive(xMutex);
              }
            }
          }
        }
      }
    } else {
      bbAllOff();
      vTaskDelay(1);   // buckboost deshabilitado (PV_ONLY) → ceder CPU al IDLE
    }

    /* ===========================================================
       (2) PV / MPPT — a su propia cadencia
    =========================================================== */
    if (now - lastPvMs >= pvControlPeriodMs) {
      lastPvMs = now;
      updatePvControl();
    }
  }
}
