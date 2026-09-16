/*
 * sensors.cpp — AGENTE 3
 * Driver ADS1115 a bajo nivel (portado del firmware base) + escalado.
 */

#include "sensors.h"
#include <Wire.h>
#include <math.h>

/* ── Registros / config ADS1115 ── */
static const uint8_t REG_CONVERSION = 0x00;
static const uint8_t REG_CONFIG     = 0x01;
static const uint16_t ADS_PGA_4096     = 0x0200;  // ±4.096 V, 125 µV/LSB
static const uint16_t ADS_RATE_860     = 0x00E0;
static const uint16_t ADS_COMP_DISABLE = 0x0003;
static const uint16_t ADS_MODE_SINGLE  = 0x0100;
static const uint16_t ADS_OS_START     = 0x8000;

static uint16_t muxBits(uint8_t ch) {
  switch (ch) { case 0: return 0x4000; case 1: return 0x5000;
                case 2: return 0x6000; default: return 0x7000; }
}
static uint8_t chFromMux(uint16_t cfg) {
  switch (cfg & 0x7000) { case 0x4000: return 0; case 0x5000: return 1;
                          case 0x6000: return 2; case 0x7000: return 3;
                          default: return 255; }
}
static uint16_t singleShotCfg(uint8_t ch) {
  return ADS_OS_START | muxBits(ch) | ADS_PGA_4096 | ADS_MODE_SINGLE |
         ADS_RATE_860 | ADS_COMP_DISABLE;
}

static bool writeReg(uint8_t addr, uint8_t reg, uint16_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  return Wire.endTransmission() == 0;
}
static bool readReg(uint8_t addr, uint8_t reg, int16_t& val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 2) < 2) return false;
  uint8_t msb = Wire.read(), lsb = Wire.read();
  val = (int16_t)((msb << 8) | lsb);
  return true;
}
static bool adsPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}
static bool waitReady(uint8_t addr, uint32_t timeoutUs = 8000) {
  /* Solapar el yield con la conversión: vTaskDelay(1) cede CPU al IDLE
     (alimenta el watchdog) MIENTRAS el ADS convierte (~1.16 ms), así el lazo
     no paga 1 ms de yield aparte → sube de ~400 a ~700 Hz. Luego se sondea. */
  vTaskDelay(1);
  uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < timeoutUs) {
    int16_t c = 0;
    if (!readReg(addr, REG_CONFIG, c)) return false;
    if (((uint16_t)c & 0x8000) != 0) return true;   // OS=1 → listo
    delayMicroseconds(80);
  }
  return false;
}
static float rawToVolts(int16_t raw) {
  float v = raw * 0.000125f;
  return v < 0.0f ? 0.0f : v;
}

/* Lectura single-shot de un canal (verifica que el mux coincide). */
static bool readChannel(uint8_t addr, uint8_t ch, float& volts) {
  if (!writeReg(addr, REG_CONFIG, singleShotCfg(ch))) return false;
  if (!waitReady(addr)) return false;
  int16_t cfg = 0;
  if (!readReg(addr, REG_CONFIG, cfg)) return false;
  if (chFromMux((uint16_t)cfg) != ch) return false;
  int16_t raw = 0;
  if (!readReg(addr, REG_CONVERSION, raw)) return false;
  volts = rawToVolts(raw);
  return true;
}

/* ── Calibración de corriente ── */
static float applyCurrentCal(CurrentCal& cal, float adcVolts) {
  if (!isfinite(adcVolts)) return 0.0f;
  float raw = (float)cal.sign * (adcVolts - cal.zero) / cal.sens;
  if (!cal.ready) { cal.filtered = raw; cal.ready = true; }
  else {
    float a = clampF(cal.alpha, 0.0f, 1.0f);
    cal.filtered = a * raw + (1.0f - a) * cal.filtered;
  }
  return (fabsf(cal.filtered) < cal.deadband) ? 0.0f : cal.filtered;
}

/* ── SoC desde v_bat2s (lineal entre socEmptyV y socFullV) ── */
static float estimateSoc(float vbat) {
  float lo = socEmptyV, hi = socFullV;
  if (hi - lo < 0.1f) return 0.5f;
  return clampF((vbat - lo) / (hi - lo), 0.0f, 1.0f);
}

/* ============================================================
   API
============================================================ */
/* Reloj I2C: 100 kHz. Con DOS ADS1115 colgados del mismo bus (main + mppt) y
   cableado de prueba, 400 kHz dejaba al segundo ADS sin responder (NAK/timeout).
   100 kHz es mucho más tolerante. Si necesitas más velocidad de lazo y el bus
   está limpio, se puede subir a 200000. */
#define I2C_CLOCK_HZ 100000

bool sensorsInit() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  delay(20);
  bool ads1Ok = adsPresent(adsMainAddr);
  bool ads2Ok = adsPresent(adsMpptAddr);
  Serial.printf("[SENS] ADS1/main: %s\n", ads1Ok ? "OK" : "NO detectado");
  Serial.printf("[SENS] ADS2/mppt: %s\n", ads2Ok ? "OK" : "NO detectado");
  return ads1Ok;
}

bool sensorsRetry() {
  return adsPresent(adsMainAddr);
}

/* Para subir el lazo del buckboost a ~400 Hz: la corriente i_bb (A3, lazo
   interno) se lee CADA ciclo; bus5 (A0, lazo de voltaje lento) y bat2s (A1,
   SoC) se DECIMAN y se retienen (ZOH). N_BUS=16 → bus ~25 Hz; N_BAT=64 → ~6 Hz. */
static const uint16_t N_BUS = 10;   // bus ≈ loop/10 ≈ 70 Hz (2× Nyquist del lazo de voltaje a 35 Hz)
static const uint16_t N_BAT = 64;

bool sensorsUpdateBuckboost() {
  static uint16_t cnt = 0;
  static float lastA0 = NAN, lastA1 = NAN, lastA2 = NAN;   // bus5, bat2s, spare (retenidos)
  cnt++;

  float a3;
  if (!readChannel(adsMainAddr, ADS1_CH_IBUS5, a3)) return false;  // corriente cada ciclo
  rawIBus  = a3;
  i_bb_bus5 = applyCurrentCal(calIBus5, a3);

  float v;
  if (cnt % N_BUS == 0 || isnan(lastA0)) { if (readChannel(adsMainAddr, ADS1_CH_BUS5,  v)) lastA0 = v; }
  if (cnt % N_BAT == 0 || isnan(lastA1)) { if (readChannel(adsMainAddr, ADS1_CH_BAT2S, v)) lastA1 = v; }
  /* A2 (spare) = punto medio de la 2S (nodo entre celda 1 y celda 2). Voltaje
     lento → se lee decimado y se retiene, como bat2s. */
  if (cnt % N_BAT == 0 || isnan(lastA2)) { if (readChannel(adsMainAddr, ADS1_CH_SPARE, v)) lastA2 = v; }
  if (isnan(lastA0) || isnan(lastA1)) return true;   // i_bb ya actualizado; falta 1.ª lectura bus/bat

  rawBus5  = lastA0;
  rawBat2s = lastA1;
  v_bus5   = lastA0 * divFactorBus5;
  v_bat2s  = lastA1 * divFactorPv2s;
  if (!isnan(lastA2)) { rawSpare = lastA2; v_spare = lastA2 * divFactorSpare; }  // punto medio 2S
  soc      = estimateSoc(v_bat2s);
  return true;
}

bool sensorsUpdatePv() {
  float a0, a2, a3;
  bool ok = readChannel(adsMpptAddr, ADS2_CH_PANEL,  a0);
  ok = readChannel(adsMpptAddr, ADS2_CH_IPANEL, a2) && ok;
  ok = readChannel(adsMpptAddr, ADS2_CH_IBAT,   a3) && ok;
  if (!ok) return false;

  rawPanel  = a0;
  rawIPanel = a2;
  rawIBat   = a3;
  v_panel  = a0 * divFactorPanel;
  i_panel  = applyCurrentCal(calIPanel, a2);
  i_bat    = applyCurrentCal(calIBat,   a3);
  p_panel  = v_panel * i_panel;

#if I2C_DEBUG
  Serial.printf("[I2C][ADS2] rawVPV=%.4fV rawIPV=%.4fV rawIBAT=%.4fV | VPV=%.3fV IPV=%.3fA IBAT=%.3fA Ppv=%.3fW\n",
                rawPanel, rawIPanel, rawIBat, v_panel, i_panel, i_bat, p_panel);
#endif
  return true;
}
