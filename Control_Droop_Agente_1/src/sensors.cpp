/*
 * sensors.cpp
 * ─────────────────────────────────────────────────────────────
 * Driver ADS1115 manual (I2C directo, sin librería Adafruit).
 * Estimación SoC por LUT interpolada linealmente.
 * ─────────────────────────────────────────────────────────────
 */

#include "sensors.h"
#include "shared.h"
#include <Wire.h>
#include <math.h>

/* ── ADS1115 ── */
static const uint8_t  ADS_ADDR         = 0x48;
static const uint8_t  REG_CONVERSION   = 0x00;
static const uint8_t  REG_CONFIG       = 0x01;
/* Valores DR para ADS1115 (¡distintos al ADS1015!):
 *  8=0x0000  16=0x0020  32=0x0040  64=0x0060
 * 128=0x0080 250=0x00A0 475=0x00C0 860=0x00E0  */
static const uint16_t ADS_PGA_4096     = 0x0200;  // ±4.096 V → LSB = 125 µV
static const uint16_t ADS_MODE_SINGLE  = 0x0100;  // single-shot (sin esto el MUX no cambia entre canales)
static const uint16_t ADS_RATE_860     = 0x00E0;  // 860 SPS → conversión ~1.16 ms
static const uint16_t ADS_COMP_DISABLE = 0x0003;

static const float LSB_V = 0.000125f;  // 125 µV por bit

/* Sensibilidad ACS712 — usar acsSensitivity (variable compartida) para calibración */

/* ============================================================
   LUT SoC — voltaje (V) vs SoC [0,1]
   Rellena esta tabla con los valores medidos en tu batería.
   La interpolación es lineal entre puntos adyacentes.
   Los voltajes DEBEN estar en orden ascendente.
============================================================ */
static const float SOC_V[]   = { 3.00f, 3.20f, 3.50f, 3.70f, 3.80f,
                                  3.90f, 4.00f, 4.10f, 4.20f };
static const float SOC_PCT[] = { 0.00f, 0.05f, 0.15f, 0.35f, 0.50f,
                                  0.65f, 0.78f, 0.90f, 1.00f };
static const int   SOC_LEN   = sizeof(SOC_V) / sizeof(SOC_V[0]);

/* ============================================================
   UTILIDADES ADS
============================================================ */
static bool adsWrite(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(ADS_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  return Wire.endTransmission() == 0;
}

static bool adsRead(int16_t& out) {
  Wire.beginTransmission(ADS_ADDR);
  Wire.write(REG_CONVERSION);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)ADS_ADDR, 2) < 2) return false;
  out = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

static uint16_t singleShotCfg(uint8_t ch) {
  static const uint16_t mux[] = {0x4000, 0x5000, 0x6000, 0x7000};
  return 0x8000 | mux[ch & 3] | ADS_PGA_4096 | ADS_MODE_SINGLE | ADS_RATE_860 | ADS_COMP_DISABLE;
}

/* Lee el bit OS del registro de config. Retorna true si la conversión terminó. */
static bool conversionComplete() {
  Wire.beginTransmission(ADS_ADDR);
  Wire.write(REG_CONFIG);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)ADS_ADDR, 2) < 2) return false;
  uint16_t cfg = ((uint16_t)Wire.read() << 8) | Wire.read();
  return (cfg & 0x8000) != 0;
}

static float adsSingleShot(uint8_t ch) {
  if (!adsWrite(REG_CONFIG, singleShotCfg(ch))) return NAN;
  /* Solapar el yield con la conversión: vTaskDelay(1) cede CPU al IDLE
     (alimenta el watchdog) MIENTRAS el ADS convierte (~1.16 ms), así el lazo
     no paga 1 ms de yield APARTE del tiempo de conversión → sube de ~400 a
     ~700 Hz. Luego se sondea el bit OS por si la conversión no terminó
     (vTaskDelay(1) puede durar < 1.16 ms). Sin modo continuo no hay carrera
     del OS: tras el vTaskDelay la conversión ya arrancó (OS=0). */
  vTaskDelay(1);
  uint32_t t0 = micros();
  while (!conversionComplete()) {
    if ((uint32_t)(micros() - t0) > 3000UL) return NAN;   // timeout ~3 ms
    delayMicroseconds(50);
  }
  int16_t raw;
  if (!adsRead(raw)) return NAN;
  return raw < 0 ? 0.0f : raw * LSB_V;
}

/* ============================================================
   LUT SoC — interpolación lineal
============================================================ */
float estimateSoC(float vb) {
  if (vb <= SOC_V[0])         return SOC_PCT[0];
  if (vb >= SOC_V[SOC_LEN-1]) return SOC_PCT[SOC_LEN-1];
  for (int i = 1; i < SOC_LEN; i++) {
    if (vb <= SOC_V[i]) {
      float t = (vb - SOC_V[i-1]) / (SOC_V[i] - SOC_V[i-1]);
      return SOC_PCT[i-1] + t * (SOC_PCT[i] - SOC_PCT[i-1]);
    }
  }
  return SOC_PCT[SOC_LEN-1];
}

/* ============================================================
   INIT
============================================================ */
bool sensorsInit() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  /* Verificar presencia enviando un write de prueba */
  Wire.beginTransmission(ADS_ADDR);
  Wire.write(REG_CONFIG);
  return Wire.endTransmission() == 0;
}

bool sensorsRetry() {
  return sensorsInit();
}

/* ============================================================
   UPDATE — single-shot decimado (~400 Hz). La corriente (ACS, A3, lazo
   interno) se lee CADA ciclo; el bus (A2, lazo de voltaje lento) y la
   batería/SoC se DECIMAN y se retienen (ZOH) entre lecturas.
     N_BUS=16 → bus ~25 Hz  |  N_BAT=64 → batería ~6 Hz
   Si subes mucho omega_v del lazo de voltaje, baja N_BUS.
============================================================ */
static const uint16_t N_BUS = 10;   // bus ≈ loop/10 ≈ 70 Hz (2× Nyquist del lazo de voltaje a 35 Hz)
static const uint16_t N_BAT = 64;

bool sensorsUpdate() {
  static uint16_t cnt    = 0;
  static float    lastVa2 = NAN;   // bus (retenido, ZOH)
  static float    lastVa1 = NAN;   // batería
  cnt++;

  float va3 = adsSingleShot(3);    // ACS712 — cada ciclo (lazo de corriente)
  if (isnan(va3)) return false;    // el ACS es el indicador de salud del ADS

  if (cnt % N_BUS == 0 || isnan(lastVa2)) {   // bus decimado
    float v = adsSingleShot(2);
    if (!isnan(v)) lastVa2 = v;
  }
  if (cnt % N_BAT == 0 || isnan(lastVa1)) {   // batería/SoC decimados
    float v = adsSingleShot(1);
    if (!isnan(v)) lastVa1 = v;
  }
  float va2 = lastVa2, va1 = lastVa1;
  if (isnan(va2) || isnan(va1)) return true;  // aún sin primera lectura de bus/bat

  float dBus, dBat, offset, sign, sens;
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    dBus   = divFactorBus;
    dBat   = divFactorBat;
    offset = acsOffset;
    sign   = acsSign;
    sens   = acsSensitivity;
    xSemaphoreGive(xMutex);
  } else return false;

  if (sens < 1e-4f) sens = 0.185f;  // guardia contra división por cero

  float vBus = va2 * dBus;
  float vBat = va1 * dBat;
  float iOut = sign * (va3 - offset) / sens;
  float soc_ = estimateSoC(vBat);

  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    rawVoltsA0 = va2;   // rawVoltsA0 almacena el canal de bus (ahora A2)
    rawVoltsA1 = va1;
    rawVoltsA3 = va3;
    v_bus = vBus;
    v_bat = vBat;
    i_out = iOut;
    soc   = soc_;
    xSemaphoreGive(xMutex);
  }

  return true;
}
