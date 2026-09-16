/*
 * control.cpp
 * ─────────────────────────────────────────────────────────────
 * Control PWM buck-boost (FF+PI) + lectura ADS1115 vía I2C.
 * Corre íntegramente en Core 1.
 *
 * Comportamiento ante fallas de hardware:
 *   ADS OK  → operación normal (control activo)
 *   ADS FAIL → control inhibido, PWM apagado, hwStatus actualizado
 *
 * hwStatus lo combina taskComms (Core 0) con el estado del CAN
 * para producir el estado final reportado a Firebase.
 * ─────────────────────────────────────────────────────────────
 */

#include "control.h"
#include "shared.h"
#include <Wire.h>
#include <math.h>

/* ============================================================
   CONSTANTES ADS1115
============================================================ */
static const uint8_t  ADS_ADDR         = 0x48;
static const uint8_t  REG_CONVERSION   = 0x00;
static const uint8_t  REG_CONFIG       = 0x01;
static const uint16_t ADS_PGA_4096     = 0x0200;
static const uint16_t ADS_RATE_860     = 0x00E0;
static const uint16_t ADS_COMP_DISABLE = 0x0003;

/* Número de fallos consecutivos antes de declarar ADS perdido */
#define ADS_FAIL_THRESHOLD   5
/* Intervalo de reintento de reconexión ADS (ms) */
#define ADS_RETRY_MS         5000

/* ============================================================
   CONSTANTES PWM LEDC
============================================================ */
static const uint8_t  CH_HIGH  = 0;
static const uint8_t  CH_LOW   = 1;
static const uint8_t  PWM_BITS = 10;
static const uint32_t DUTY_MAX = (1UL << PWM_BITS) - 1;

/* ============================================================
   ESTADO PRIVADO DEL MÓDULO
============================================================ */
static float    piIntegral         = 0.0f;
static float    tonFracAcc         = 0.0f;
static uint8_t  fastControlChannel = 0;
static bool     pwmHwReady         = false;

// Estado ADS (privado, solo Core 1 lo escribe)
static bool     adsAvailable       = false;   // true = ADS respondió en init
static uint8_t  adsFailCount       = 0;       // fallos consecutivos de lectura
static bool     adsOnline          = false;   // estado actual de operación
static uint32_t adsLastRetryMs     = 0;

/* ============================================================
   PWM HARDWARE
============================================================ */
void configHwPwm() {
  uint32_t period;
  xSemaphoreTake(xMutex, portMAX_DELAY);
  period = pwmPeriodUs;
  xSemaphoreGive(xMutex);

  uint32_t freq = period ? 1000000UL / period : 0;
  if (!freq) return;

  ledcSetup(CH_HIGH, freq, PWM_BITS);  ledcAttachPin(PIN_PWM_HIGH, CH_HIGH);
  ledcSetup(CH_LOW,  freq, PWM_BITS);  ledcAttachPin(PIN_PWM_LOW,  CH_LOW);
  pwmHwReady = true;
}

static uint32_t tonToDuty(float ton, float period) {
  if (ton <= 0.0f || period <= 0.0f) return 0;
  uint32_t d = (uint32_t)lroundf((ton / period) * (float)DUTY_MAX);
  return d > DUTY_MAX ? DUTY_MAX : d;
}

void allOff() {
  if (pwmHwReady) {
    ledcWrite(CH_HIGH, DUTY_MAX);
    ledcWrite(CH_LOW,  0);
  } else {
    pinMode(PIN_PWM_HIGH, OUTPUT);  pinMode(PIN_PWM_LOW, OUTPUT);
    digitalWrite(PIN_PWM_HIGH, HIGH);  digitalWrite(PIN_PWM_LOW, LOW);
  }
}

void driveBoost(float ton, float allowed, float period) {
  ton = clampF(ton, 0.0f, allowed);
  ledcWrite(CH_HIGH, DUTY_MAX);
  ledcWrite(CH_LOW,  tonToDuty(ton, period));
}

void driveBuck(float ton, float allowed, float period) {
  ton = clampF(ton, 0.0f, allowed);
  ledcWrite(CH_HIGH, DUTY_MAX - tonToDuty(ton, period));
  ledcWrite(CH_LOW,  0);
}

float getAllowedTonMax(float period, float tMax, float tHard, float minOff) {
  float lim = min(tMax, tHard);
  float pl  = period - minOff;
  if (pl < 0) pl = 0;
  return max(0.0f, min(lim, pl));
}

/* ============================================================
   ADS1115  (driver manual I2C)
============================================================ */
static bool adsWriteReg(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(ADS_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  return Wire.endTransmission() == 0;
}

static bool adsReadConversion(int16_t& val) {
  Wire.beginTransmission(ADS_ADDR);
  Wire.write(REG_CONVERSION);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)ADS_ADDR, 2) < 2) return false;
  val = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

static uint16_t adsContinuousCfg(uint8_t ch) {
  static const uint16_t mux[] = {0x4000, 0x5000, 0x6000, 0x7000};
  return 0x8000 | mux[ch & 3] | ADS_PGA_4096 | ADS_RATE_860 | ADS_COMP_DISABLE;
}

static uint16_t adsSingleCfg(uint8_t ch) {
  static const uint16_t mux[] = {0x4000, 0x5000, 0x6000, 0x7000};
  return 0x8000 | mux[ch & 3] | 0x0200 | 0x0100 | 0x00E0 | 0x0003;
}

static float rawToVolts(int16_t raw) {
  float v = raw * 0.000125f;
  return v < 0.0f ? 0.0f : v;
}

static float adsSingleShot(uint8_t ch) {
  if (!adsWriteReg(REG_CONFIG, adsSingleCfg(ch))) return NAN;
  delayMicroseconds(1300);
  Wire.beginTransmission(ADS_ADDR);  Wire.write(REG_CONVERSION);
  if (Wire.endTransmission(false) != 0) return NAN;
  if (Wire.requestFrom((int)ADS_ADDR, 2) < 2) return NAN;
  int16_t raw = (int16_t)((Wire.read() << 8) | Wire.read());
  return rawToVolts(raw);
}

/* ── Intentar inicializar / reconectar el ADS ── */
static bool adsTryInit() {
  // Enviar un write de prueba al registro de config
  Wire.beginTransmission(ADS_ADDR);
  Wire.write(REG_CONFIG);
  bool ok = (Wire.endTransmission() == 0);
  return ok;
}

/* ── Actualizar hwStatus según disponibilidad de ADS y CAN ── */
static void updateHwStatus(bool adsOk) {
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    bool canOk = (hwStatus == HW_OK || hwStatus == HW_NO_ADS);
    if (adsOk && canOk)   hwStatus = HW_OK;
    else if (!adsOk && canOk)  hwStatus = HW_NO_ADS;
    else if (adsOk && !canOk)  hwStatus = HW_NO_CAN;
    else                        hwStatus = HW_NO_CAN_NO_ADS;
    xSemaphoreGive(xMutex);
  }
}

bool configFastCh() {
  if (!adsOnline) return false;
  uint8_t ch;
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    ch = (topology == TOPO_BOOST) ? 2 : 1;
    xSemaphoreGive(xMutex);
  } else return false;

  fastControlChannel = ch;
  if (!adsWriteReg(REG_CONFIG, adsContinuousCfg(ch))) return false;
  delayMicroseconds(1300);
  return true;
}

bool updateFast() {
  if (!adsOnline) return false;
  int16_t raw = 0;
  if (!adsReadConversion(raw)) {
    adsFailCount++;
    if (adsFailCount >= ADS_FAIL_THRESHOLD) {
      adsOnline = false;
      adsFailCount = 0;
      Serial.println("[ADS] Perdido — control inhibido");
      updateHwStatus(false);
      allOff();
      if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        runMode = RUN_OFF;
        xSemaphoreGive(xMutex);
      }
    }
    return false;
  }
  adsFailCount = 0;
  float v = rawToVolts(raw);
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    if (fastControlChannel == 0 || fastControlChannel == 2) {
      rawVoltsA0 = v;
      v_bus = v * divFactorA0;
    } else {
      rawVoltsA1 = v;
      v_ag1 = v * divFactorA1;
    }
    xSemaphoreGive(xMutex);
  }
  return true;
}

bool updateTelemetry() {
  if (!adsOnline) return false;
  float a2 = adsSingleShot(2);
  float a1 = adsSingleShot(1);
  if (isnan(a2) || isnan(a1)) {
    adsFailCount++;
    if (adsFailCount >= ADS_FAIL_THRESHOLD) {
      adsOnline = false;
      adsFailCount = 0;
      Serial.println("[ADS] Perdido en telemetría — control inhibido");
      updateHwStatus(false);
      allOff();
      if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        runMode = RUN_OFF;
        xSemaphoreGive(xMutex);
      }
    }
    return false;
  }
  adsFailCount = 0;
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    rawVoltsA0 = a2;
    rawVoltsA1 = a1;
    v_bus = a2 * divFactorA0;
    v_ag1 = a1 * divFactorA1;
    xSemaphoreGive(xMutex);
  }
  configFastCh();
  return true;
}

/* ============================================================
   PROTECCIONES
============================================================ */
void tripFault(const char* reason) {
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    faultLatched  = true;
    runMode       = RUN_OFF;
    requestedMode = RUN_OFF;   // sincronizar modo deseado
    xSemaphoreGive(xMutex);
  }
  tonFracAcc        = 0;
  pendingFaultWrite = true;    // pedir a Core 0 que escriba en Firebase
  allOff();
  Serial.printf("[FAULT] %s\n", reason);
}

bool checkProtections() {
  float vb, va, bBT, bAT, ukBTL, ukBTH, ukOT;
  int   topo;
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) != pdTRUE) return true;
  vb    = v_bus;  va    = v_ag1;  topo  = (int)topology;
  bBT   = boostBusTripHigh;  bAT   = boostAg1TripLow;
  ukBTL = buckBusTripLow;    ukBTH = buckBusTripHigh;  ukOT = buckOutTripHigh;
  xSemaphoreGive(xMutex);

  if (topo == (int)TOPO_BOOST) {
    if (vb > bBT)  { tripFault("BOOST BUS alto");   return false; }
    if (va < bAT)  { tripFault("BOOST AG1 bajo");   return false; }
  } else {
    if (vb < ukBTL) { tripFault("BUCK BUS bajo");   return false; }
    if (vb > ukBTH) { tripFault("BUCK BUS alto");   return false; }
    if (va > ukOT)  { tripFault("BUCK VOUT alto");  return false; }
  }
  return true;
}

/* ============================================================
   FEEDFORWARD + PI
============================================================ */
void updateFFPI() {
  float vOut, vIn, tgt, _kp, _ki, _ff, period, tMax, tHard, minOff;
  float stepUp, stepDown, _buckVd;
  uint32_t ctrlUs;
  int topo;

  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) != pdTRUE) return;
  topo   = (int)topology;
  vOut   = (topo == (int)TOPO_BOOST) ? v_bus : v_ag1;
  vIn    = (topo == (int)TOPO_BOOST) ? v_ag1 : v_bus;
  tgt    = targetV;  _kp = kp;  _ki = ki;  _ff = ffScale;
  period = (float)pwmPeriodUs;
  tMax   = tonMaxUs;  tHard = tonHardMaxUs;  minOff = minOffUs;
  stepUp = maxTonStepUpUs;  stepDown = maxTonStepDownUs;
  _buckVd = buckDiodeDrop;  ctrlUs = controlPeriodUs;
  xSemaphoreGive(xMutex);

  if (!checkProtections()) return;

  float error   = tgt - vOut;
  float dt      = (float)ctrlUs / 1e6f;
  float allowed = getAllowedTonMax(period, tMax, tHard, minOff);

  float newTon;
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    newTon = tonCmdUs;  xSemaphoreGive(xMutex);
  } else return;

  if (error <= -1.0f) {
    piIntegral *= 0.5f;
    newTon = slewLimitAsym(newTon, 0.0f, stepUp, stepDown);
  } else if (error <= 0.0f) {
    piIntegral *= 0.92f;
    newTon = slewLimitAsym(newTon, newTon * 0.95f, stepUp, stepDown);
  } else {
    piIntegral += error * dt;
    piIntegral  = clampF(piIntegral, -20.0f, 20.0f);

    float tonFF = 0.0f;
    if (topo == (int)TOPO_BOOST && vIn > 0.1f && tgt > vIn) {
      float d = clampF(1.0f - (vIn / tgt), 0.0f, 0.85f);
      tonFF = _ff * d * period;
    } else if (topo == (int)TOPO_BUCK && vIn > 0.5f) {
      float d = (tgt >= vIn) ? 0.9f : clampF((tgt + _buckVd) / (vIn + _buckVd), 0.0f, 0.9f);
      tonFF = _ff * d * period;
    }

    float tonTarget = tonFF + _kp * error + _ki * piIntegral;
    if (topo == (int)TOPO_BOOST) {
      if      (error > 1.00f) tonTarget = fmaxf(tonTarget, 28.0f);
      else if (error > 0.70f) tonTarget = fmaxf(tonTarget, 24.0f);
      else if (error > 0.45f) tonTarget = fmaxf(tonTarget, 20.0f);
    }
    tonTarget = clampF(tonTarget, 0.0f, allowed);
    newTon = slewLimitAsym(newTon, tonTarget, stepUp, stepDown);
  }

  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    tonCmdUs = newTon;  xSemaphoreGive(xMutex);
  }
}

/* ============================================================
   TAREA FREERTOS — Core 1
============================================================ */
void taskControl(void* pvParams) {
  delay(200);   // esperar que Serial esté listo antes del primer print
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  configHwPwm();
  allOff();

  /* ── Intentar inicializar ADS ── */
  adsAvailable = adsTryInit();
  adsOnline    = adsAvailable;
  if (adsOnline) {
    configFastCh();
    Serial.println("[Core1] ADS1115 OK — control activo");
  } else {
    Serial.println("[Core1] ADS1115 NO detectado — control inhibido");
    Serial.println("[Core1] Parametros PI modificables. Control activo cuando conecte sensor.");
    if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      hwStatus = HW_NO_ADS;
      xSemaphoreGive(xMutex);
    }
  }

  unsigned long lastCtrlUs     = micros();
  unsigned long lastFullReadMs = millis();

  for (;;) {
    unsigned long nowUs = micros();
    unsigned long nowMs = millis();

    /* ── Reintento periódico de reconexión del ADS ── */
    if (!adsOnline && (nowMs - adsLastRetryMs >= ADS_RETRY_MS)) {
      adsLastRetryMs = nowMs;
      if (adsTryInit()) {
        adsOnline    = true;
        adsFailCount = 0;
        configFastCh();
        Serial.println("[ADS] Reconectado — control restaurado");
        updateHwStatus(true);
      }
    }

    uint32_t ctrlP, fullP;
    if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      ctrlP = controlPeriodUs;  fullP = fullReadPeriodMs;
      xSemaphoreGive(xMutex);
    } else { ctrlP = 1163;  fullP = 200; }

    /* ── Lazo rápido (solo si ADS está online) ── */
    if (adsOnline && (nowUs - lastCtrlUs >= ctrlP)) {
      lastCtrlUs += ctrlP;
      if (updateFast()) {
        bool fault, doAuto;
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
          fault  = faultLatched;
          doAuto = (runMode == RUN_AUTO_FFPI);
          xSemaphoreGive(xMutex);
        } else { fault = false;  doAuto = false; }
        checkProtections();
        if (doAuto && !fault) updateFFPI();
      }
    }

    /* ── Lectura completa periódica ── */
    if (adsOnline && (nowMs - lastFullReadMs >= fullP)) {
      lastFullReadMs = nowMs;
      updateTelemetry();
    }

    /* ── Aplicar salida PWM ── */
    float ton, allowed, period, mton;
    int   topo, rmode, reqmode;
    bool  fault;
    if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      ton     = tonCmdUs;  mton    = manualTonUs;
      topo    = (int)topology;
      reqmode = (int)requestedMode;
      fault   = faultLatched;   period = (float)pwmPeriodUs;
      allowed = getAllowedTonMax(period, tonMaxUs, tonHardMaxUs, minOffUs);

      // runMode refleja el estado efectivo:
      // - FAULT         → siempre OFF
      // - Sin ADS + AUTO → OFF (no hay medición para el PI)
      // - Sin ADS + MANUAL → permitido (el usuario fija ton directamente)
      // - Sin ADS + OFF  → OFF
      if (fault) {
        runMode = RUN_OFF;
      } else if (!adsOnline && reqmode == (int)RUN_AUTO_FFPI) {
        runMode = RUN_OFF;
      } else {
        runMode = (RunMode)reqmode;
      }
      rmode = (int)runMode;
      xSemaphoreGive(xMutex);
    } else { allOff();  vTaskDelay(1);  continue; }

    if (rmode == (int)RUN_OFF) {
      allOff();
      if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        tonAppliedUs = 0.0f;  xSemaphoreGive(xMutex);
      }
    } else if (rmode == (int)RUN_MANUAL) {
      float clamped = clampF(mton, 0.0f, allowed);
      if (topo == (int)TOPO_BOOST) driveBoost(mton, allowed, period);
      else                         driveBuck (mton, allowed, period);
      if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        tonAppliedUs = clamped;  xSemaphoreGive(xMutex);
      }
    } else {
      float clamped = clampF(ton, 0.0f, allowed);
      if (topo == (int)TOPO_BOOST) driveBoost(ton, allowed, period);
      else                         driveBuck (ton, allowed, period);
      if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        tonAppliedUs = clamped;  xSemaphoreGive(xMutex);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
