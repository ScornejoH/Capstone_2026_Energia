/*
 * control.cpp
 * ─────────────────────────────────────────────────────────────
 * Lazos PI de voltaje y corriente + droop SoC + PWM LEDC.
 * Corre íntegramente en Core 1.
 *
 * Parámetros PI derivados de frecuencia natural y amortiguamiento:
 *   Kp = 2 · ζ · ω_n
 *   Ki = ω_n²
 *
 * El lazo de voltaje (externo, lento) genera i_ref.
 * El lazo de corriente (interno, rápido) genera el duty cycle.
 * La topología se selecciona automáticamente por el signo de i_ref.
 * ─────────────────────────────────────────────────────────────
 */

#include "control.h"
#include "shared.h"
#include "sensors.h"
#include <math.h>

/* ============================================================
   PWM LEDC
============================================================ */
static const uint8_t  CH_HIGH  = 0;
static const uint8_t  CH_LOW   = 1;
static const uint8_t  PWM_BITS = 10;
static const uint32_t DUTY_MAX = (1UL << PWM_BITS) - 1;

static bool pwmReady = false;

void initPwm() {
  uint32_t period;
  xSemaphoreTake(xMutex, portMAX_DELAY);
  period = pwmPeriodUs;
  xSemaphoreGive(xMutex);

  uint32_t freq = period ? 1000000UL / period : 10000;
  ledcSetup(CH_HIGH, freq, PWM_BITS);  ledcAttachPin(PIN_PWM_HIGH, CH_HIGH);
  ledcSetup(CH_LOW,  freq, PWM_BITS);  ledcAttachPin(PIN_PWM_LOW,  CH_LOW);
  pwmReady = true;
}

void allOff() {
  if (pwmReady) {
    ledcWrite(CH_HIGH, DUTY_MAX);
    ledcWrite(CH_LOW,  0);
  } else {
    pinMode(PIN_PWM_HIGH, OUTPUT);
    pinMode(PIN_PWM_LOW,  OUTPUT);
    digitalWrite(PIN_PWM_HIGH, HIGH);
    digitalWrite(PIN_PWM_LOW,  LOW);
  }
}

static uint32_t dutyFromUs(float ton, float period) {
  if (ton <= 0.0f || period <= 0.0f) return 0;
  uint32_t d = (uint32_t)lroundf((ton / period) * (float)DUTY_MAX);
  return d > DUTY_MAX ? DUTY_MAX : d;
}

static void driveBoost(float ton, float period, float maxTon) {
  ton = clampF(ton, 0.0f, maxTon);
  ledcWrite(CH_HIGH, DUTY_MAX);
  ledcWrite(CH_LOW,  dutyFromUs(ton, period));
}

static void driveBuck(float ton, float period, float maxTon) {
  ton = clampF(ton, 0.0f, maxTon);
  ledcWrite(CH_HIGH, DUTY_MAX - dutyFromUs(ton, period));
  ledcWrite(CH_LOW,  0);
}

/* ============================================================
   CÁLCULO DE GANANCIAS PI DESDE FRECUENCIAS DE CRUCE
   L y C son constantes de hardware (PLANT_L_H, PLANT_C_F).
   El usuario ajusta omegaV y omegaI (wc en rad/s).

   Lazo voltaje (planta Gv=1/(sC)):
     Kp_v = wc_v · C            Ki_v = Kp_v · wc_v / 10
   Lazo corriente (planta Gi=Vdc/(sL), salida = duty adimensional):
     Kp_i = wc_i · L / Vdc      Ki_i = Kp_i · wc_i / 10
   El /Vdc es imprescindible: sin él Kp_i queda Vdc veces demasiado grande
   y el cruce real se dispara → oscilación.
============================================================ */
void recomputeGains() {
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
    float vdc = plantVdc < 0.5f ? 0.5f : plantVdc;   // piso para evitar ganancia infinita
    kpV = omegaV * PLANT_C_F;
    kiV = kpV * (omegaV / 10.0f);
    kpI = omegaI * PLANT_L_H / vdc;
    kiI = kpI * (omegaI / 10.0f);
    xSemaphoreGive(xMutex);
  }
}

/* ============================================================
   FAULT
============================================================ */
void tripFault(FaultCode code) {
  allOff();
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    faultLatched  = true;
    faultCode     = code;
    runMode       = RUN_OFF;
    requestedMode = RUN_OFF;
    xSemaphoreGive(xMutex);
  }
  pendingFaultWrite = true;
}

/* ============================================================
   PROTECCIONES
============================================================ */
bool checkProtections() {
  float vb, iout, ovTrip, uvTrip, ocTrip;
  bool  enabled;
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) != pdTRUE) return true;
  vb     = v_bus;
  iout   = i_out;
  ovTrip = overvoltageTrip;
  uvTrip = undervoltageTrip;
  ocTrip = overcurrentTrip;
  enabled = faultsEnabled;
  xSemaphoreGive(xMutex);

  if (!enabled) return true;

  if (vb > ovTrip)          { tripFault(FLT_OVERVOLTAGE);  return false; }
  if (vb < uvTrip)          { tripFault(FLT_UNDERVOLTAGE); return false; }
  if (fabsf(iout) > ocTrip) { tripFault(FLT_OVERCURRENT);  return false; }
  return true;
}

/* ============================================================
   TAREA FREERTOS — Core 1
============================================================ */
void taskControl(void* pvParams) {
  /* ── Init sensores ── */
  bool adsOk = sensorsInit();
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
    hwStatus = adsOk ? HW_OK : HW_NO_ADS;
    xSemaphoreGive(xMutex);
  }
  Serial.printf("[SENS] ADS1115: %s\n", adsOk ? "OK" : "NO detectado");

  /* ── Init PWM ── */
  initPwm();
  allOff();

  /* ── Calcular ganancias desde los omega/Vdc por defecto, para que el
        control esté bien sintonizado aunque arranque sin WiFi/Firebase. ── */
  recomputeGains();

  /* ── Integradores PI ── */
  float integV  = 0.0f;
  float integI  = 0.0f;
  float iOutFilt = 0.0f;  // corriente filtrada (paso bajo ~20 Hz)

  /* ── Tiempo de control (µs → s) ── */
  static const float DT = 0.001f;   // 1 ms = periodo de control
  static const uint32_t ADS_RETRY_MS    = 5000;
  static const uint8_t  ADS_FAIL_THRESH = 5;
  uint8_t  adsFailCount = 0;
  uint32_t adsLastRetry = 0;

  /* ── Topología anterior (para detectar cambio de modo) ── */
  TopologyMode prevTopo = TOPO_BOOST;

  uint32_t lastMs    = millis();
  uint32_t lastLogMs = millis();   // muestreo del log de alta frecuencia (50 Hz)

  for (;;) {
    /* ── Esperar al menos 1 ms antes de la siguiente iteración ── */
    vTaskDelay(1);
    uint32_t now = millis();
    float dt = clampF((float)(now - lastMs) * 0.001f, 0.001f, 0.050f);  // s, máx 50 ms
    lastMs = now;

    /* ── Reintento ADS si está offline ── */
    if (!adsOk) {
      if (millis() - adsLastRetry >= ADS_RETRY_MS) {
        adsLastRetry = millis();
        adsOk = sensorsRetry();
        if (adsOk) {
          adsFailCount = 0;
          if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            hwStatus = HW_OK;
            xSemaphoreGive(xMutex);
          }
          Serial.println("[SENS] ADS1115 reconectado");
        }
      }
      allOff();
      continue;
    }

    /* ── Leer sensores ── */
    if (!sensorsUpdate()) {
      adsFailCount++;
      if (adsFailCount >= ADS_FAIL_THRESH) {
        adsOk = false; adsFailCount = 0; adsLastRetry = millis();
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          hwStatus = HW_NO_ADS;
          xSemaphoreGive(xMutex);
        }
        tripFault(FLT_NO_ADS);
      }
      continue;
    }
    adsFailCount = 0;

    /* ── Muestreo del log de alta frecuencia (50 Hz, decimado del lazo) ──
       Se registra aquí (sensores frescos) sin importar run/OFF/fault. */
    if (now - lastLogMs >= 20) {
      lastLogMs = now;
      LogSample smp;
      smp.t_ms = now;
      if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        smp.v_bus = v_bus; smp.v_bat = v_bat;
        smp.i_out = i_out; smp.soc = soc; smp.i_ref = i_ref;
        xSemaphoreGive(xMutex);
        logPush(smp);
      }
    }

    /* ── Protecciones ── */
    if (!checkProtections()) {
      integV = 0.0f; integI = 0.0f;
      continue;
    }

    /* ── Leer estado de control (bajo mutex) ── */
    RunMode rmode;
    bool    fault;
    float   tgtV, kpv, kiv, kpi, kii, kd, irMax, irMin;
    float   vBus, iOut, soc_, period, maxTon, minOff;

    if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) != pdTRUE) continue;
    rmode  = requestedMode;
    fault  = faultLatched;
    tgtV   = targetV;
    kpv    = kpV;  kiv = kiV;
    kpi    = kpI;  kii = kiI;
    kd     = kdDroop;
    irMax  = iRefMax;  irMin = iRefMin;
    vBus   = v_bus;    iOut  = i_out;   soc_ = soc;
    period = (float)pwmPeriodUs;
    maxTon = tonMaxUs;
    minOff = minOffUs;
    xSemaphoreGive(xMutex);

    /* ── Si fault o OFF: apagar y resetear integradores ── */
    if (fault || rmode == RUN_OFF) {
      allOff();
      integV = 0.0f; integI = 0.0f;
      if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        runMode    = RUN_OFF;
        i_ref      = 0.0f;
        i_ref_v    = 0.0f;
        i_ref_d    = 0.0f;
        dutyCmd    = 0.0f;
        dutyApplied = 0.0f;
        xSemaphoreGive(xMutex);
      }
      continue;
    }

    /* ── Filtro paso bajo corriente: fc ≈ 20 Hz → α = dt/(τ+dt), τ=1/(2π·fc) ── */
    {
      const float tau = 1.0f / (2.0f * (float)M_PI * 20.0f);
      float alpha = dt / (tau + dt);
      iOutFilt += alpha * (iOut - iOutFilt);
    }

    /* ── Lazo externo: PI voltaje → i_ref_v ── */
    float errV  = tgtV - vBus;
    integV     += errV * dt;
    float iRefV = kpv * errV + kiv * integV;

    /* ── Droop SoC → i_ref_d ── */
    float iRefD = kd * (soc_ - 0.5f);

    /* ── Referencia de corriente total ── */
    float iRef = clampF(iRefV + iRefD, irMin, irMax);

    /* ── Selección de topología con histéresis ──
       Permanece en BOOST hasta que i_ref < -100 mA, y en BUCK hasta que
       i_ref > +100 mA. Evita el "chattering" de modo cerca de i_ref = 0. */
    const float TOPO_HYST_A = 0.1f;   // ±100 mA
    TopologyMode topo = prevTopo;
    if (prevTopo == TOPO_BOOST) {
      if (iRef < -TOPO_HYST_A) topo = TOPO_BUCK;
    } else { /* TOPO_BUCK */
      if (iRef >  TOPO_HYST_A) topo = TOPO_BOOST;
    }

    /* Al cambiar de modo, resetear integrador interno para evitar windup */
    if (topo != prevTopo) {
      integI = 0.0f;
      prevTopo = topo;
    }

    /* ── Lazo interno: PI corriente → duty ── */
    float errI  = iRef - iOutFilt;   // usar corriente filtrada
    integI     += errI * dt;
    float duty  = kpi * errI + kii * integI;   // [A·s/A = s]... escala libre

    /* Convertir duty (salida PI) a ton en microsegundos.
       duty está en [0,1] si los PI están bien sintonizados — si no,
       se satura con el clamp de tonMaxUs. */
    float maxAllowed = min(maxTon, period - minOff);
    maxAllowed = max(0.0f, maxAllowed);
    float ton = clampF(fabsf(duty) * period, 0.0f, maxAllowed);

    /* Saturar integrador (anti-windup simple por saturación) */
    float dutyMax = maxAllowed / period;
    integI = clampF(integI, -dutyMax / kii, dutyMax / kii);

    /* ── Aplicar PWM ── */
    if (topo == TOPO_BOOST) {
      driveBoost(ton, period, maxAllowed);
    } else {
      driveBuck(ton, period, maxAllowed);
    }

    float applied = ton / period;

    /* ── Actualizar variables compartidas ── */
    if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      runMode        = RUN_ON;
      i_ref_v        = iRefV;
      i_ref_d        = iRefD;
      i_ref          = iRef;
      dutyCmd        = fabsf(duty);
      dutyApplied    = applied;
      activeTopology = topo;
      xSemaphoreGive(xMutex);
    }
  }
}
