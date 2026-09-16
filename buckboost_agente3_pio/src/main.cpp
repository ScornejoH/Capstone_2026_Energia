#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <math.h>

// ============================================================
// Agente 3 integrado: buckboost bidireccional + boost PV MPPT
// ESP32 + dos ADS1115 sobre el mismo bus I2C aislado por ISO1540.
//
// Buckboost bidireccional:
//   GPIO25 -> QH/high-side, activo LOW.
//   GPIO26 -> QL/low-side, activo HIGH.
//   BUCK : bateria 2S/PV2S -> BUS5, QH PWM, QL OFF.
//   BOOST: BUS5 -> bateria 2S/PV2S, QL PWM, QH OFF.
//
// Boost PV MPPT:
//   Extrae potencia del panel y entrega al nodo bateria 2S/PV2S.
//   El proyecto MPPT original usaba GPIO25/26, pero esos pines ya
//   pertenecen al buckboost en agente 3. Por eso se dejan pines
//   dedicados y faciles de cambiar aqui abajo.
//
// Medicion ADS principal (ADS1, default 0x48):
//   A0: BUS5.
//   A1: Vbat_2S / PV2S total.
//   A2: spare / sin uso por ahora.
//   A3: corriente buckboost hacia/desde BUS5.
//
// Medicion ADS PV/MPPT (ADS2, default 0x49):
//   A0: VPV / panel.
//   A1: Vbat_2S / PV2S opcional/duplicado.
//   A2: IPV / corriente del panel.
//   A3: Ibat / corriente de bateria, positiva si carga la bateria.
// ============================================================

// ------------------------------ Pines ------------------------------
static const int PIN_BB_PWM_HIGH = 25;
static const int PIN_BB_PWM_LOW  = 26;

// Confirmar en el cableado final. No usar 25/26 porque son del buckboost.
static const int PIN_PVBOOST_PWM = 27;
static const int PIN_PVBOOST_EN  = -1;  // Sin enable dedicado en la PCB boost PV.
static const int PIN_PVBOOST_GND = 14;  // Salida LOW fija para referencia auxiliar de baja corriente.

static const int PIN_I2C_SDA = 21;
static const int PIN_I2C_SCL = 22;

// ------------------------------ ADS1115 -----------------------------
static const uint8_t ADS_MAIN_DEFAULT = 0x48;  // ADS1: buckboost/bus.
static const uint8_t ADS_MPPT_DEFAULT = 0x49;  // ADS2: PV/bateria.

static uint8_t adsMainAddr = ADS_MAIN_DEFAULT;
static uint8_t adsMpptAddr = ADS_MPPT_DEFAULT;

static const uint8_t REG_CONVERSION = 0x00;
static const uint8_t REG_CONFIG     = 0x01;

static const uint16_t ADS_PGA_4096      = 0x0200; // +/-4.096 V, 125 uV/LSB.
static const uint16_t ADS_RATE_860      = 0x00E0;
static const uint16_t ADS_COMP_DISABLE  = 0x0003;
static const uint16_t ADS_MODE_SINGLE   = 0x0100;
static const uint16_t ADS_OS_START      = 0x8000;

static const uint8_t ADS1_CH_BUS5    = 0;
static const uint8_t ADS1_CH_BAT2S   = 1;
static const uint8_t ADS1_CH_SPARE   = 2;
static const uint8_t ADS1_CH_IBUS5   = 3;

static const uint8_t ADS2_CH_PANEL   = 0;
static const uint8_t ADS2_CH_BAT2S   = 1;
static const uint8_t ADS2_CH_IPANEL  = 2;
static const uint8_t ADS2_CH_IBAT    = 3;

// --------------------------- Persistencia ---------------------------
static const char* CFG_NAMESPACE = "ag3_full";
static const uint32_t CFG_MAGIC = 0xA6030003UL;
static const uint16_t CFG_VERSION = 3;

// ------------------------ Topologia buckboost -----------------------
enum TopologyMode {
  TOPO_BUCK,
  TOPO_BOOST
};

enum RunMode {
  RUN_OFF,
  RUN_MANUAL,
  RUN_AUTO_FFPI
};

enum PvBoostMode {
  PV_OFF,
  PV_SENSORS,
  PV_FIXED,
  PV_VPV_PI,
  PV_MPPT_PO
};

enum OperationMode {
  OP_FULL,
  OP_BUCKBOOST_ONLY,
  OP_PV_ONLY
};

TopologyMode topology = TOPO_BUCK;
RunMode runMode = RUN_OFF;
PvBoostMode pvMode = PV_OFF;
OperationMode operationMode = OP_FULL;

bool faultLatched = false;
bool pvFaultLatched = false;
bool pvArmed = false;
bool configSavePending = false;
bool calibrationMode = false;
uint32_t currentFaultInhibitUntilMs = 0;

// -------------------------- Mediciones ------------------------------
struct Measurements {
  float ads1[4] = {NAN, NAN, NAN, NAN};
  float ads2[4] = {NAN, NAN, NAN, NAN};
  int16_t raw1[4] = {0, 0, 0, 0};
  int16_t raw2[4] = {0, 0, 0, 0};
  bool ads1Ok = false;
  bool ads2Ok = false;

  float vBatMid = 0.0f;
  float vSpare = 0.0f;
  float vBus5 = 0.0f;
  float vBat2s = 0.0f;
  float vPanel = 0.0f;

  float iBbBus5 = 0.0f;
  float iPanel = 0.0f;
  float iBat = 0.0f;
  float pPanel = 0.0f;
  float pBat = 0.0f;
  float iBoostNodeEst = 0.0f;
  float iBbIn2sKcl = 0.0f;

  uint32_t lastAds1Ms = 0;
  uint32_t lastAds2Ms = 0;
};

Measurements meas;

// Alias usados por el controlador buckboost.
float v_pv2s = 0.0f; // bateria total 2S/PV2S: ADS1 A1 primario.
float v_bus5 = 0.0f; // BUS5: ADS1 A0.

float adcPv2sVolts = NAN;
float adcBus5Volts = NAN;
unsigned long lastAdcPv2sMs = 0;
unsigned long lastAdcBus5Ms = 0;

// Divisores. Para 33k/16k ideal es 3.0625; se deja 3.12 por lo ya calibrado.
float divFactorBatMid = 3.12f;
float divFactorSpare  = 3.12f;
float divFactorBus5   = 3.12f;
float divFactorPv2s   = 3.12f;
float divFactorPanel  = 3.12f;

struct CurrentCal {
  float zero = 2.50f;
  float sens = 0.185f; // ACS712-05B: 185 mV/A.
  int8_t sign = 1;
  float alpha = 0.30f;
  float deadband = 0.04f;
  float filtered = 0.0f;
  bool ready = false;
};

CurrentCal calIBus5;
CurrentCal calIPanel;
CurrentCal calIBat;

float boostKclEfficiency = 0.90f;
uint32_t calibrationInhibitMs = 60000;

// ---------------------- Control buckboost ---------------------------
float targetV = 5.00f;

float tonMinUs = 0.0f;
float tonMaxUs = 85.0f;
float tonHardMaxUs = 92.0f;
float minOffUs = 8.0f;

float kp = 10.0f;
float ki = 5.0f;
float ffScale = 1.50f;
float buckDiodeDrop = 0.65f;

float piIntegral = 0.0f;
float tonCmdUs = 0.0f;
float manualTonUs = 10.0f;
float controlOutputFilteredV = 0.0f;
bool controlOutputFilterReady = false;

float maxTonStepUpUs = 0.5f;
float maxTonStepDownUs = 3.0f;
float controlFilterAlpha = 0.25f;
float overshootDeadbandV = 0.03f;
float smallOvershootDecay = 0.995f;

uint32_t pwmPeriodUs = 100;       // 10 kHz.
uint32_t controlPeriodUs = 1163;  // ADS1115 860 SPS.
uint32_t fullReadPeriodMs = 200;
uint32_t printPeriodMs = 1000;
bool autoLog = true;

float tonFracAcc = 0.0f;
uint8_t bbSafeStopSteps = 12;
uint16_t bbInductorDischargeUs = 1500;
uint8_t pvSafeStopSteps = 12;
uint16_t pvInductorDischargeUs = 1500;

// Protecciones buckboost.
float boostPv2sTripHigh = 8.70f;
float boostPv2sWarnHigh = 8.45f;
float boostBus5TripLow  = 4.40f;
float boostBus5WarnLow  = 4.70f;

float buckPv2sTripLow   = 6.00f;
float buckPv2sTripHigh  = 8.90f;
float buckBus5TripHigh  = 5.60f;
float buckBus5WarnHigh  = 5.30f;

// ------------------------ Control boost PV --------------------------
float pvFixedDuty = 0.05f;
float pvDutyCmd = 0.0f;
float pvDutyApplied = 0.0f;
float pvDutySoftLimit = 0.0f;
float pvPiVpvInt = 0.03f;

float pvVpvRef = 5.00f;
float pvVpvRefMin = 4.20f;
float pvVpvRefMax = 5.40f;
float pvKpVpv = 0.060f;
float pvKiVpv = 0.900f;

float pvDutyMin = 0.00f;
float pvDutyStart = 0.03f;
float pvDutyMax = 0.48f;
float pvDutySlewPerStep = 0.010f;
uint32_t pvSoftstartMs = 3000;
uint32_t pvControlPeriodMs = 50;

float pvPanelTripLow = 3.00f;
float pvBatWarnHigh = 8.45f;
float pvBatTripHigh = 8.70f;
float pvInputCurrentMax = 3.00f;
float pvBatteryChargeCurrentMax = 1.00f;
float pvInputPowerMax = 16.0f;
float pvReverseCurrentMax = 0.30f;

float mpptStepV = 0.025f;
float mpptEpsPowerW = 0.08f;
float mpptMinPowerW = 1.0f;
uint32_t mpptPeriodMs = 1000;
bool mpptInitialized = false;
float mpptPrevPowerW = 0.0f;
float mpptDirection = -1.0f;

// ------------------------- Tiempos internos -------------------------
unsigned long lastControlUs = 0;
unsigned long lastFullReadMs = 0;
unsigned long lastPrintMs = 0;
unsigned long lastPvControlMs = 0;
unsigned long pvModeStartMs = 0;
unsigned long lastMpptMs = 0;

// ----------------------------- PWM ----------------------------------
static const uint8_t PWM_CH_HIGH = 0;
static const uint8_t PWM_CH_LOW  = 1;
static const uint8_t PWM_CH_PV   = 2;
static const uint8_t PWM_RES_BITS = 10;
static const uint32_t PWM_DUTY_MAX = (1UL << PWM_RES_BITS) - 1;

bool pwmHwReady = false;
bool pvPwmReady = false;
uint8_t fastControlChannel = ADS1_CH_BUS5;
uint8_t fastControlAddr = ADS_MAIN_DEFAULT;
uint32_t lastPwmHighDuty = UINT32_MAX;
uint32_t lastPwmLowDuty = UINT32_MAX;
uint32_t lastPvPwmDuty = UINT32_MAX;

// ------------------------- Prototipos breves ------------------------
float clampFloat(float x, float xmin, float xmax);
float getAllowedTonMax();
bool saveConfig(bool verbose = true);
bool loadConfig(bool verbose = false);
void sanitizeConfig();
void configureFastControlChannel();
void pvOutputsSafeOff();
void bbAllOff();
bool prearmCheck(bool verbose);
void stopAllPower(const char* reason);
void runPowerCycle(float tonUs);

// ============================================================
// Utilidades
// ============================================================
float clampFloat(float x, float xmin, float xmax) {
  if (x < xmin) return xmin;
  if (x > xmax) return xmax;
  return x;
}

uint8_t clampU8(int x, uint8_t lo, uint8_t hi) {
  if (x < (int)lo) return lo;
  if (x > (int)hi) return hi;
  return (uint8_t)x;
}

float slewLimitAsym(float current, float target, float maxUp, float maxDown) {
  if (target > current + maxUp) return current + maxUp;
  if (target < current - maxDown) return current - maxDown;
  return target;
}

const char* topologyName() {
  return (topology == TOPO_BUCK) ? "BUCK" : "BOOST";
}

const char* runName() {
  if (runMode == RUN_OFF) return "OFF";
  if (runMode == RUN_MANUAL) return "MANUAL";
  if (runMode == RUN_AUTO_FFPI) return "AUTO_FFPI";
  return "?";
}

const char* pvModeName() {
  if (pvMode == PV_OFF) return "PV_OFF";
  if (pvMode == PV_SENSORS) return "PV_SENSORS";
  if (pvMode == PV_FIXED) return "PV_FIXED";
  if (pvMode == PV_VPV_PI) return "PV_VPV_PI";
  if (pvMode == PV_MPPT_PO) return "PV_MPPT_PO";
  return "?";
}

const char* operationModeName() {
  if (operationMode == OP_FULL) return "FULL";
  if (operationMode == OP_BUCKBOOST_ONLY) return "BUCKBOOST_ONLY";
  if (operationMode == OP_PV_ONLY) return "PV_ONLY";
  return "?";
}

bool buckboostEnabledByMode() {
  return operationMode == OP_FULL || operationMode == OP_BUCKBOOST_ONLY;
}

bool pvBoostEnabledByMode() {
  return operationMode == OP_FULL || operationMode == OP_PV_ONLY;
}

float getInputV() {
  return (topology == TOPO_BUCK) ? v_pv2s : v_bus5;
}

float getOutputV() {
  return (topology == TOPO_BUCK) ? v_bus5 : v_pv2s;
}

bool currentFaultsInhibited() {
  if (calibrationMode) return true;
  if (currentFaultInhibitUntilMs == 0) return false;
  return ((int32_t)(millis() - currentFaultInhibitUntilMs) < 0);
}

float getAllowedTonMax() {
  float periodLimit = (float)pwmPeriodUs - minOffUs;
  if (periodLimit < 0.0f) periodLimit = 0.0f;

  float limit = tonMaxUs;
  if (limit > tonHardMaxUs) limit = tonHardMaxUs;
  if (limit > periodLimit) limit = periodLimit;
  if (limit < 0.0f) limit = 0.0f;
  return limit;
}

uint32_t pwmFrequencyHz() {
  if (pwmPeriodUs == 0) return 0;
  return 1000000UL / pwmPeriodUs;
}

uint32_t dutyToCounts(float duty) {
  return (uint32_t)lroundf(clampFloat(duty, 0.0f, 1.0f) * (float)PWM_DUTY_MAX);
}

uint32_t tonUsToDuty(float tonUs) {
  if (tonUs <= 0.0f || pwmPeriodUs == 0) return 0;
  uint32_t duty = (uint32_t)lroundf((tonUs / (float)pwmPeriodUs) * (float)PWM_DUTY_MAX);
  return (duty > PWM_DUTY_MAX) ? PWM_DUTY_MAX : duty;
}

uint8_t parseI2cAddress(const String& rawIn, uint8_t fallback) {
  String raw = rawIn;
  raw.trim();
  raw.toLowerCase();
  if (raw.length() == 0) return fallback;

  char* endPtr = nullptr;
  uint32_t parsed = strtoul(raw.c_str(), &endPtr, raw.startsWith("0x") ? 16 : 10);
  if (endPtr == raw.c_str() || parsed < 0x03 || parsed > 0x77) return fallback;
  return (uint8_t)parsed;
}

void printHexAddr(uint8_t addr) {
  Serial.print("0x");
  if (addr < 16) Serial.print('0');
  Serial.print(addr, HEX);
}

// ============================================================
// PWM y salidas de potencia
// ============================================================
void invalidateBbPwmCache() {
  lastPwmHighDuty = UINT32_MAX;
  lastPwmLowDuty = UINT32_MAX;
}

void invalidatePvPwmCache() {
  lastPvPwmDuty = UINT32_MAX;
}

void writeBbPwmChannels(uint32_t highDuty, uint32_t lowDuty) {
  if (highDuty != lastPwmHighDuty) {
    ledcWrite(PWM_CH_HIGH, highDuty);
    lastPwmHighDuty = highDuty;
  }

  if (lowDuty != lastPwmLowDuty) {
    ledcWrite(PWM_CH_LOW, lowDuty);
    lastPwmLowDuty = lowDuty;
  }
}

void configureHardwarePwm() {
  uint32_t freqHz = pwmFrequencyHz();
  if (freqHz == 0) return;

  ledcSetup(PWM_CH_HIGH, freqHz, PWM_RES_BITS);
  ledcAttachPin(PIN_BB_PWM_HIGH, PWM_CH_HIGH);
  ledcSetup(PWM_CH_LOW, freqHz, PWM_RES_BITS);
  ledcAttachPin(PIN_BB_PWM_LOW, PWM_CH_LOW);
  pwmHwReady = true;
  invalidateBbPwmCache();
}

void configurePvBoostPwm() {
  ledcSetup(PWM_CH_PV, 10000, PWM_RES_BITS);
  ledcAttachPin(PIN_PVBOOST_PWM, PWM_CH_PV);
  ledcWrite(PWM_CH_PV, 0);
  pvPwmReady = true;
  invalidatePvPwmCache();
}

void bbAllOff() {
  if (pwmHwReady) {
    writeBbPwmChannels(PWM_DUTY_MAX, 0);
  } else {
    pinMode(PIN_BB_PWM_HIGH, OUTPUT);
    pinMode(PIN_BB_PWM_LOW, OUTPUT);
    digitalWrite(PIN_BB_PWM_HIGH, HIGH);
    digitalWrite(PIN_BB_PWM_LOW, LOW);
  }
}

void bbSafeStop(bool rampDown) {
  float startTon = fmaxf(tonCmdUs, manualTonUs);
  startTon = clampFloat(startTon, 0.0f, getAllowedTonMax());

  if (rampDown && pwmHwReady && startTon > 0.01f && bbSafeStopSteps > 0) {
    for (uint8_t i = bbSafeStopSteps; i > 0; --i) {
      float t = startTon * ((float)(i - 1) / (float)bbSafeStopSteps);
      runPowerCycle(t);
      delayMicroseconds((pwmPeriodUs > 0) ? pwmPeriodUs : 100);
    }
  }

  bbAllOff();
  if (bbInductorDischargeUs > 0) delayMicroseconds(bbInductorDischargeUs);
}

void pvEnableWrite(bool enable) {
  if (PIN_PVBOOST_EN >= 0) {
    digitalWrite(PIN_PVBOOST_EN, enable ? HIGH : LOW);
  }
}

void pvWriteDuty(float duty) {
  if (!pvPwmReady) configurePvBoostPwm();
  uint32_t counts = dutyToCounts(duty);
  if (counts != lastPvPwmDuty) {
    ledcWrite(PWM_CH_PV, counts);
    lastPvPwmDuty = counts;
  }
  pvDutyApplied = clampFloat(duty, 0.0f, 1.0f);
}

void pvOutputsSafeOff() {
  float startDuty = pvDutyApplied;
  if (pvPwmReady && startDuty > 0.0005f && pvSafeStopSteps > 0) {
    for (uint8_t i = pvSafeStopSteps; i > 0; --i) {
      float d = startDuty * ((float)(i - 1) / (float)pvSafeStopSteps);
      pvWriteDuty(d);
      delayMicroseconds(100);
    }
  } else {
    pvWriteDuty(0.0f);
  }
  if (pvInductorDischargeUs > 0) delayMicroseconds(pvInductorDischargeUs);
  pvEnableWrite(false);
  pvArmed = false;
}

void driveBoostHardware(float tonUs) {
  tonUs = clampFloat(tonUs, 0.0f, getAllowedTonMax());
  uint32_t duty = tonUsToDuty(tonUs);
  // BOOST 5 V -> 2S: QH OFF (GPIO25 HIGH), QL PWM activo-alto.
  writeBbPwmChannels(PWM_DUTY_MAX, duty);
}

void driveBuckHardware(float tonUs) {
  tonUs = clampFloat(tonUs, 0.0f, getAllowedTonMax());
  uint32_t duty = tonUsToDuty(tonUs);
  // BUCK 2S -> 5 V: QH PWM activo-bajo, QL OFF.
  writeBbPwmChannels(PWM_DUTY_MAX - duty, 0);
}

void runPowerCycle(float tonUs) {
  if (topology == TOPO_BOOST) driveBoostHardware(tonUs);
  else driveBuckHardware(tonUs);
}

void refreshBuckboostOutput() {
  if (!buckboostEnabledByMode()) {
    bbAllOff();
    return;
  }

  if (runMode == RUN_OFF || faultLatched) {
    bbAllOff();
  } else if (runMode == RUN_MANUAL) {
    runPowerCycle(manualTonUs);
  } else if (runMode == RUN_AUTO_FFPI) {
    runPowerCycle(tonCmdUs);
  }
}

// ============================================================
// ADS1115 bajo nivel
// ============================================================
uint16_t adsMuxBitsForChannel(uint8_t channel) {
  if (channel == 0) return 0x4000;
  if (channel == 1) return 0x5000;
  if (channel == 2) return 0x6000;
  return 0x7000;
}

uint8_t adsChannelFromMuxBits(uint16_t config) {
  switch (config & 0x7000) {
    case 0x4000: return 0;
    case 0x5000: return 1;
    case 0x6000: return 2;
    case 0x7000: return 3;
    default: return 255;
  }
}

uint16_t makeContinuousConfig(uint8_t channel) {
  uint16_t config = 0;
  config |= adsMuxBitsForChannel(channel);
  config |= ADS_PGA_4096;
  config |= ADS_RATE_860;
  config |= ADS_COMP_DISABLE;
  return config;
}

uint16_t makeSingleShotConfig(uint8_t channel) {
  uint16_t config = 0;
  config |= ADS_OS_START;
  config |= adsMuxBitsForChannel(channel);
  config |= ADS_PGA_4096;
  config |= ADS_MODE_SINGLE;
  config |= ADS_RATE_860;
  config |= ADS_COMP_DISABLE;
  return config;
}

bool writeRegister(uint8_t addr, uint8_t reg, uint16_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write((uint8_t)(value >> 8));
  Wire.write((uint8_t)(value & 0xFF));
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    Serial.print("ADS write error @");
    printHexAddr(addr);
    Serial.print(" err=");
    Serial.println(err);
    return false;
  }
  return true;
}

bool readRegister(uint8_t addr, uint8_t reg, int16_t& value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) {
    Serial.print("ADS pointer error @");
    printHexAddr(addr);
    Serial.print(" err=");
    Serial.println(err);
    return false;
  }

  uint8_t n = Wire.requestFrom((int)addr, 2);
  if (n < 2) {
    Serial.print("ADS requestFrom error @");
    printHexAddr(addr);
    Serial.println();
    return false;
  }

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  value = (int16_t)((msb << 8) | lsb);
  return true;
}

bool adsPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool waitAdsSingleShotReady(uint8_t addr, uint32_t timeoutUs = 10000) {
  unsigned long startUs = micros();
  while ((uint32_t)(micros() - startUs) < timeoutUs) {
    int16_t cfgRaw = 0;
    if (!readRegister(addr, REG_CONFIG, cfgRaw)) return false;
    if (((uint16_t)cfgRaw & 0x8000) != 0) return true;
    delayMicroseconds(100);
  }
  Serial.print("ADS timeout single-shot @");
  printHexAddr(addr);
  Serial.println();
  return false;
}

bool adsConfigMatchesChannel(uint16_t config, uint8_t channel) {
  return adsChannelFromMuxBits(config) == channel;
}

float rawToSignedVolts(int16_t raw) {
  return raw * 0.000125f;
}

float rawToVolts(int16_t raw) {
  float volts = rawToSignedVolts(raw);
  return (volts < 0.0f) ? 0.0f : volts;
}

bool readAdsSingleShotRaw(uint8_t addr, uint8_t channel, int16_t& raw, uint16_t* cfgReadback = nullptr) {
  if (!writeRegister(addr, REG_CONFIG, makeSingleShotConfig(channel))) return false;
  if (!waitAdsSingleShotReady(addr)) return false;

  uint16_t cfg = 0;
  int16_t cfgRaw = 0;
  if (!readRegister(addr, REG_CONFIG, cfgRaw)) return false;
  cfg = (uint16_t)cfgRaw;
  if (!adsConfigMatchesChannel(cfg, channel)) {
    Serial.print("ADS mux mismatch @");
    printHexAddr(addr);
    Serial.print(": esperado A");
    Serial.print(channel);
    Serial.print(" leido A");
    Serial.println((int)adsChannelFromMuxBits(cfg));
    return false;
  }

  if (cfgReadback != nullptr) {
    *cfgReadback = cfg;
  }

  return readRegister(addr, REG_CONVERSION, raw);
}

float readAdsSingleShot(uint8_t addr, uint8_t channel) {
  int16_t raw = 0;
  if (!readAdsSingleShotRaw(addr, channel, raw)) return NAN;
  return rawToVolts(raw);
}

float readAdsSettledSingleShot(uint8_t addr, uint8_t channel) {
  int16_t raw = 0;
  (void)readAdsSingleShotRaw(addr, channel, raw);
  delayMicroseconds(250);
  if (!readAdsSingleShotRaw(addr, channel, raw)) return NAN;
  return rawToVolts(raw);
}

bool readAdsAverageVolts(uint8_t addr, uint8_t channel, uint8_t samples, float& volts) {
  float acc = 0.0f;
  uint8_t ok = 0;

  (void)readAdsSingleShot(addr, channel);
  delayMicroseconds(300);

  for (uint8_t i = 0; i < samples; ++i) {
    float v = readAdsSingleShot(addr, channel);
    if (isfinite(v)) {
      acc += v;
      ok++;
    }
    delayMicroseconds(300);
  }

  if (ok == 0) return false;
  volts = acc / (float)ok;
  return true;
}

void scanI2CAtBoot() {
  bool foundAny = false;
  Serial.print("I2C scan SDA=");
  Serial.print(PIN_I2C_SDA);
  Serial.print(" SCL=");
  Serial.println(PIN_I2C_SCL);

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      foundAny = true;
      Serial.print("I2C device @ ");
      printHexAddr(addr);
      Serial.println();
    }
  }

  if (!foundAny) {
    Serial.println("I2C: no se detectaron dispositivos.");
  }
}

void configureFastControlChannel() {
  if (!buckboostEnabledByMode()) return;

  if (topology == TOPO_BUCK) {
    fastControlAddr = adsMainAddr;
    fastControlChannel = ADS1_CH_BUS5;
  } else {
    fastControlAddr = adsMainAddr;
    fastControlChannel = ADS1_CH_BAT2S;
  }

  if (writeRegister(fastControlAddr, REG_CONFIG, makeContinuousConfig(fastControlChannel))) {
    delayMicroseconds(1300);
  }
}

// ============================================================
// Procesamiento de mediciones
// ============================================================
float applyCurrentCal(CurrentCal& cal, float adcVolts) {
  if (!isfinite(adcVolts)) return 0.0f;
  float raw = (float)cal.sign * (adcVolts - cal.zero) / cal.sens;

  if (!cal.ready) {
    cal.filtered = raw;
    cal.ready = true;
  } else {
    float a = clampFloat(cal.alpha, 0.0f, 1.0f);
    cal.filtered = a * raw + (1.0f - a) * cal.filtered;
  }

  return (fabsf(cal.filtered) < cal.deadband) ? 0.0f : cal.filtered;
}

void applyMeasurementScale() {
  meas.vBatMid = 0.0f;
  meas.vSpare  = isfinite(meas.ads1[ADS1_CH_SPARE])   ? meas.ads1[ADS1_CH_SPARE]   * divFactorSpare  : 0.0f;
  meas.vBus5   = isfinite(meas.ads1[ADS1_CH_BUS5])    ? meas.ads1[ADS1_CH_BUS5]    * divFactorBus5   : ((operationMode == OP_PV_ONLY) ? 0.0f : v_bus5);

  meas.vPanel  = isfinite(meas.ads2[ADS2_CH_PANEL])   ? meas.ads2[ADS2_CH_PANEL]   * divFactorPanel  : 0.0f;
  if (isfinite(meas.ads1[ADS1_CH_BAT2S])) {
    meas.vBat2s = meas.ads1[ADS1_CH_BAT2S] * divFactorPv2s;
  } else if (isfinite(meas.ads2[ADS2_CH_BAT2S])) {
    meas.vBat2s = meas.ads2[ADS2_CH_BAT2S] * divFactorPv2s;
  } else {
    meas.vBat2s = v_pv2s;
  }

  meas.iBbBus5 = applyCurrentCal(calIBus5,  meas.ads1[ADS1_CH_IBUS5]);
  meas.iPanel  = applyCurrentCal(calIPanel, meas.ads2[ADS2_CH_IPANEL]);
  meas.iBat    = applyCurrentCal(calIBat,   meas.ads2[ADS2_CH_IBAT]);

  meas.pPanel = meas.vPanel * meas.iPanel;
  meas.pBat = meas.vBat2s * meas.iBat;

  // Convencion KCL:
  //   Ipanel boost inyecta potencia al nodo 2S con eficiencia boostKclEfficiency.
  //   Ibat es positiva cuando la bateria se carga desde el nodo.
  //   IbbIn2sKcl positiva significa corriente que entra al buckboost desde el nodo 2S.
  meas.iBoostNodeEst = (meas.vBat2s > 0.20f) ? (boostKclEfficiency * meas.pPanel / meas.vBat2s) : 0.0f;
  meas.iBbIn2sKcl = meas.iBoostNodeEst - meas.iBat;

  v_bus5 = meas.vBus5;
  v_pv2s = meas.vBat2s;
}

bool updateFastMeasurement() {
  int16_t cfgRaw = 0;
  if (!readRegister(fastControlAddr, REG_CONFIG, cfgRaw)) return false;
  if (!adsConfigMatchesChannel((uint16_t)cfgRaw, fastControlChannel)) {
    configureFastControlChannel();
    return false;
  }

  int16_t raw = 0;
  if (!readRegister(fastControlAddr, REG_CONVERSION, raw)) return false;

  float volts = rawToVolts(raw);
  unsigned long nowMs = millis();

  if (fastControlAddr == adsMainAddr && fastControlChannel == ADS1_CH_BUS5) {
    meas.ads1[ADS1_CH_BUS5] = volts;
    adcBus5Volts = volts;
    lastAdcBus5Ms = nowMs;
    meas.vBus5 = volts * divFactorBus5;
    v_bus5 = meas.vBus5;
  } else if (fastControlAddr == adsMainAddr && fastControlChannel == ADS1_CH_BAT2S) {
    meas.ads1[ADS1_CH_BAT2S] = volts;
    adcPv2sVolts = volts;
    lastAdcPv2sMs = nowMs;
    meas.vBat2s = volts * divFactorPv2s;
    v_pv2s = meas.vBat2s;
  }

  return true;
}

bool readAdsAllChannels(uint8_t addr, float outVolts[4], int16_t outRaw[4]) {
  bool ok = true;
  for (uint8_t ch = 0; ch < 4; ++ch) {
    int16_t discard = 0;
    int16_t raw = 0;
    bool okDiscard = readAdsSingleShotRaw(addr, ch, discard);
    delayMicroseconds(250);
    bool okRead = readAdsSingleShotRaw(addr, ch, raw);
    if (!okDiscard || !okRead) {
      ok = false;
      outVolts[ch] = NAN;
      outRaw[ch] = 0;
    } else {
      outVolts[ch] = rawToVolts(raw);
      outRaw[ch] = raw;
    }
  }
  return ok;
}

bool updateTelemetryMeasurements() {
  bool ok1 = readAdsAllChannels(adsMainAddr, meas.ads1, meas.raw1);
  bool ok2 = readAdsAllChannels(adsMpptAddr, meas.ads2, meas.raw2);
  meas.ads1Ok = ok1;
  meas.ads2Ok = ok2;

  uint32_t now = millis();
  if (ok1) {
    meas.lastAds1Ms = now;
    adcBus5Volts = meas.ads1[ADS1_CH_BUS5];
    lastAdcBus5Ms = now;
  }
  if (ok2) {
    meas.lastAds2Ms = now;
    adcPv2sVolts = meas.ads1[ADS1_CH_BAT2S];
    lastAdcPv2sMs = now;
  }

  applyMeasurementScale();
  configureFastControlChannel();

  return ok1 && ok2;
}

bool updatePvBoostControlMeasurements() {
  bool ok = true;
  float v = NAN;

  if (readAdsAverageVolts(adsMpptAddr, ADS2_CH_PANEL, 1, v)) meas.ads2[ADS2_CH_PANEL] = v;
  else ok = false;

  if (readAdsAverageVolts(adsMainAddr, ADS1_CH_BAT2S, 1, v)) {
    meas.ads1[ADS1_CH_BAT2S] = v;
    adcPv2sVolts = v;
    lastAdcPv2sMs = millis();
  } else {
    ok = false;
  }

  if (readAdsAverageVolts(adsMpptAddr, ADS2_CH_IPANEL, 1, v)) meas.ads2[ADS2_CH_IPANEL] = v;
  else ok = false;

  if (readAdsAverageVolts(adsMpptAddr, ADS2_CH_IBAT, 1, v)) meas.ads2[ADS2_CH_IBAT] = v;
  else ok = false;

  applyMeasurementScale();
  configureFastControlChannel();
  return ok;
}

void printAdsRawDevice(uint8_t addr, const char* label) {
  Serial.print(label);
  Serial.print(" @");
  printHexAddr(addr);
  Serial.println(" single-ended sin divisores:");

  for (uint8_t ch = 0; ch < 4; ++ch) {
    int16_t rawDiscard = 0;
    int16_t raw1 = 0;
    int16_t raw2 = 0;
    uint16_t cfg1 = 0;
    uint16_t cfg2 = 0;
    bool okD = readAdsSingleShotRaw(addr, ch, rawDiscard);
    delayMicroseconds(250);
    bool ok1 = readAdsSingleShotRaw(addr, ch, raw1, &cfg1);
    delayMicroseconds(250);
    bool ok2 = readAdsSingleShotRaw(addr, ch, raw2, &cfg2);

    Serial.print("  A");
    Serial.print(ch);
    Serial.print(": ");
    if (!okD || !ok1 || !ok2) {
      Serial.println("ERR");
    } else {
      Serial.print("mux_s1=A");
      Serial.print((int)adsChannelFromMuxBits(cfg1));
      Serial.print(" mux_s2=A");
      Serial.print((int)adsChannelFromMuxBits(cfg2));
      Serial.print(" d=");
      Serial.print(rawToSignedVolts(rawDiscard), 6);
      Serial.print(" s1=");
      Serial.print(rawToSignedVolts(raw1), 6);
      Serial.print(" s2=");
      Serial.print(rawToSignedVolts(raw2), 6);
      Serial.print(" V raw=");
      Serial.print(raw1);
      Serial.print("/");
      Serial.println(raw2);
    }
  }
}

void printAdsRawChannels() {
  Serial.println("Mapa ADS agente 3 completo:");
  Serial.println("  ADS1 A0=BUS5, A1=Vbat_2S/PV2S, A2=spare, A3=I_buckboost_bus5");
  Serial.println("  ADS2 A0=VPV, A1=Vbat_2S opcional/duplicado, A2=IPV, A3=Ibat");
  printAdsRawDevice(adsMainAddr, "ADS1/main");
  printAdsRawDevice(adsMpptAddr, "ADS2/mppt");
  configureFastControlChannel();
}

// ============================================================
// Protecciones buckboost
// ============================================================
void tripFault(const char* reason) {
  if (faultLatched) return;
  faultLatched = true;
  runMode = RUN_OFF;
  tonFracAcc = 0.0f;
  bbSafeStop(true);

  Serial.print("FAULT buckboost: ");
  Serial.print(reason);
  Serial.print(" | Topo=");
  Serial.print(topologyName());
  Serial.print(" | PV2S=");
  Serial.print(v_pv2s, 3);
  Serial.print(" V | BUS5=");
  Serial.print(v_bus5, 3);
  Serial.print(" V | Ton=");
  Serial.print(tonCmdUs, 3);
  Serial.println(" us");
}

bool checkProtections(bool verbose) {
  if (topology == TOPO_BOOST) {
    if (v_pv2s > boostPv2sTripHigh) {
      tripFault("BOOST PV2S/VOUT alto");
      return false;
    }
    if (v_bus5 < boostBus5TripLow) {
      tripFault("BOOST BUS5/VIN bajo");
      return false;
    }
  } else {
    if (v_pv2s < buckPv2sTripLow) {
      tripFault("BUCK PV2S/VIN bajo");
      return false;
    }
    if (v_pv2s > buckPv2sTripHigh) {
      tripFault("BUCK PV2S/VIN alto");
      return false;
    }
    if (v_bus5 > buckBus5TripHigh) {
      tripFault("BUCK BUS5/VOUT alto");
      return false;
    }
  }

  if (verbose) Serial.println("Protecciones buckboost OK");
  return true;
}

// ============================================================
// Control buckboost FF + PI
// ============================================================
float computeBoostFeedforwardTon() {
  if (v_bus5 <= 0.1f || targetV <= v_bus5) return 0.0f;
  float dutyIdeal = 1.0f - (v_bus5 / targetV);
  dutyIdeal = clampFloat(dutyIdeal, 0.0f, 0.85f);
  return ffScale * dutyIdeal * (float)pwmPeriodUs;
}

float computeBuckFeedforwardTon() {
  if (v_pv2s <= 0.5f) return 0.0f;
  if (targetV >= v_pv2s) return getAllowedTonMax();
  float dutyIdeal = (targetV + buckDiodeDrop) / (v_pv2s + buckDiodeDrop);
  dutyIdeal = clampFloat(dutyIdeal, 0.0f, 0.90f);
  return ffScale * dutyIdeal * (float)pwmPeriodUs;
}

float computeFeedforwardTon() {
  return (topology == TOPO_BOOST) ? computeBoostFeedforwardTon() : computeBuckFeedforwardTon();
}

void resetController() {
  piIntegral = 0.0f;
  tonCmdUs = 0.0f;
  tonFracAcc = 0.0f;
  controlOutputFilteredV = getOutputV();
  controlOutputFilterReady = false;
}

void updateFFPI() {
  float measuredOutputV = getOutputV();
  if (!controlOutputFilterReady) {
    controlOutputFilteredV = measuredOutputV;
    controlOutputFilterReady = true;
  } else {
    float alpha = clampFloat(controlFilterAlpha, 0.01f, 1.0f);
    controlOutputFilteredV += alpha * (measuredOutputV - controlOutputFilteredV);
  }

  float error = targetV - controlOutputFilteredV;
  float dt = (float)controlPeriodUs / 1000000.0f;

  if (!checkProtections(false)) return;

  if (error <= -1.0f) {
    piIntegral *= 0.5f;
    tonCmdUs = slewLimitAsym(tonCmdUs, 0.0f, maxTonStepUpUs, maxTonStepDownUs);
    return;
  }

  if (error <= 0.0f) {
    if (error > -overshootDeadbandV) {
      piIntegral *= 0.995f;
      tonCmdUs = slewLimitAsym(tonCmdUs, tonCmdUs * smallOvershootDecay, maxTonStepUpUs, maxTonStepDownUs);
    } else {
      piIntegral *= 0.92f;
      tonCmdUs = slewLimitAsym(tonCmdUs, tonCmdUs * 0.95f, maxTonStepUpUs, maxTonStepDownUs);
    }
    return;
  }

  piIntegral += error * dt;
  piIntegral = clampFloat(piIntegral, -20.0f, 20.0f);

  float tonTarget = computeFeedforwardTon() + kp * error + ki * piIntegral;

  if (topology == TOPO_BOOST) {
    if (error > 1.00f) tonTarget = fmaxf(tonTarget, 28.0f);
    else if (error > 0.70f) tonTarget = fmaxf(tonTarget, 24.0f);
    else if (error > 0.45f) tonTarget = fmaxf(tonTarget, 20.0f);
  }

  tonTarget = clampFloat(tonTarget, tonMinUs, getAllowedTonMax());
  tonCmdUs = slewLimitAsym(tonCmdUs, tonTarget, maxTonStepUpUs, maxTonStepDownUs);
}

// ============================================================
// Control boost PV / MPPT
// ============================================================
void pvTripFault(const char* reason) {
  if (pvFaultLatched) return;
  pvFaultLatched = true;
  pvMode = PV_OFF;
  pvOutputsSafeOff();
  Serial.print("FAULT pvboost: ");
  Serial.print(reason);
  Serial.print(" | VPV=");
  Serial.print(meas.vPanel, 3);
  Serial.print(" V | VBAT2S=");
  Serial.print(meas.vBat2s, 3);
  Serial.print(" V | IPV=");
  Serial.print(meas.iPanel, 3);
  Serial.println(" A");
}

bool pvSafetyOkBeforePower() {
  if (!meas.ads2Ok && !adsPresent(adsMpptAddr)) {
    pvTripFault("ADS2 no responde");
    return false;
  }
  if (meas.vBat2s > pvBatTripHigh) {
    pvTripFault("VBAT2S alta");
    return false;
  }
  if (meas.vPanel < pvPanelTripLow) {
    pvTripFault("VPV colapsado");
    return false;
  }
  if (!currentFaultsInhibited()) {
    if (meas.iPanel > pvInputCurrentMax) {
      pvTripFault("IPV alta");
      return false;
    }
    if (meas.iBat > pvBatteryChargeCurrentMax) {
      pvTripFault("IBAT carga alta");
      return false;
    }
    if (meas.iPanel < -pvReverseCurrentMax) {
      pvTripFault("IPV reversa o signo malo");
      return false;
    }
    if (meas.pPanel > pvInputPowerMax) {
      pvTripFault("PPV alta");
      return false;
    }
  }
  return true;
}

float pvSoftstartDutyLimit() {
  uint32_t elapsed = millis() - pvModeStartMs;
  if (pvSoftstartMs == 0) return pvDutyMax;
  float k = clampFloat((float)elapsed / (float)pvSoftstartMs, 0.0f, 1.0f);
  return pvDutyStart + k * (pvDutyMax - pvDutyStart);
}

float pvApplyDutyLimits(float target) {
  pvDutySoftLimit = pvSoftstartDutyLimit();
  float limited = clampFloat(target, pvDutyMin, fminf(pvDutyMax, pvDutySoftLimit));
  float step = fmaxf(pvDutySlewPerStep, 0.0005f);
  limited = clampFloat(limited, pvDutyApplied - step, pvDutyApplied + step);
  return clampFloat(limited, 0.0f, pvDutyMax);
}

void pvApplyDutyCommand(float duty) {
  pvDutyCmd = duty;
  if (!pvArmed || pvMode == PV_OFF || pvMode == PV_SENSORS || pvFaultLatched) {
    pvWriteDuty(0.0f);
    pvEnableWrite(false);
    return;
  }

  if (!pvSafetyOkBeforePower()) return;

  if (meas.vBat2s >= pvBatWarnHigh && duty > pvDutyApplied) {
    duty = pvDutyApplied;
  }

  float safeDuty = pvApplyDutyLimits(duty);
  pvEnableWrite(true);
  pvWriteDuty(safeDuty);
}

void pvClearControlStates() {
  pvPiVpvInt = pvDutyStart;
  pvDutyCmd = 0.0f;
  pvDutySoftLimit = 0.0f;
  mpptInitialized = false;
  mpptPrevPowerW = 0.0f;
  mpptDirection = -1.0f;
  pvModeStartMs = millis();
  lastMpptMs = millis();
}

void pvEnterMode(PvBoostMode newMode) {
  if (!pvBoostEnabledByMode() && newMode != PV_OFF) {
    pvOutputsSafeOff();
    Serial.println("PV mode rechazado: modo BUCKBOOST_ONLY activo. Usa testmode pv o testmode full.");
    return;
  }
  pvMode = newMode;
  pvClearControlStates();
  pvWriteDuty(0.0f);
  if (newMode == PV_OFF || newMode == PV_SENSORS) pvEnableWrite(false);

  Serial.print("pv_mode=");
  Serial.println(pvModeName());
}

void pvRunFixedDuty() {
  pvApplyDutyCommand(pvFixedDuty);
}

void pvRunVpvPi() {
  // En boost PV: duty arriba -> mas corriente de panel -> VPV baja.
  float Ts = (float)pvControlPeriodMs / 1000.0f;
  float e = meas.vPanel - pvVpvRef;
  float iCandidate = pvPiVpvInt + pvKiVpv * e * Ts;
  float uCandidate = pvKpVpv * e + iCandidate;

  bool allowIntegrate =
    ((uCandidate <= pvDutyMax) && (uCandidate >= pvDutyMin)) ||
    ((uCandidate > pvDutyMax) && (e < 0.0f)) ||
    ((uCandidate < pvDutyMin) && (e > 0.0f));

  if (allowIntegrate) pvPiVpvInt = iCandidate;

  float u = pvKpVpv * e + pvPiVpvInt;
  pvApplyDutyCommand(u);
}

void pvUpdateMpptPO() {
  if (!pvArmed) return;
  if (meas.pPanel < mpptMinPowerW) return;

  uint32_t now = millis();
  if ((now - lastMpptMs) < mpptPeriodMs) return;

  if (!mpptInitialized) {
    mpptPrevPowerW = meas.pPanel;
    mpptInitialized = true;
    lastMpptMs = now;
    return;
  }

  float dP = meas.pPanel - mpptPrevPowerW;
  if (fabsf(dP) > mpptEpsPowerW && dP < 0.0f) {
    mpptDirection *= -1.0f;
  }

  float newRef = pvVpvRef + mpptDirection * mpptStepV;
  newRef = clampFloat(newRef, pvVpvRefMin, pvVpvRefMax);

  if ((newRef <= pvVpvRefMin && mpptDirection < 0.0f) ||
      (newRef >= pvVpvRefMax && mpptDirection > 0.0f)) {
    mpptDirection *= -1.0f;
  }

  pvVpvRef = newRef;
  mpptPrevPowerW = meas.pPanel;
  lastMpptMs = now;
}

void pvRunMpptPO() {
  pvUpdateMpptPO();
  pvRunVpvPi();
}

void updatePvBoostControl() {
  if (!pvBoostEnabledByMode()) {
    pvMode = PV_OFF;
    pvOutputsSafeOff();
    return;
  }

  if (pvMode == PV_OFF || pvFaultLatched) {
    pvOutputsSafeOff();
    return;
  }

  if (pvMode == PV_SENSORS) {
    pvWriteDuty(0.0f);
    pvEnableWrite(false);
    return;
  }

  if (!updatePvBoostControlMeasurements()) {
    pvTripFault("lectura ADS2 fallo");
    return;
  }

  if (pvMode == PV_FIXED) pvRunFixedDuty();
  else if (pvMode == PV_VPV_PI) pvRunVpvPi();
  else if (pvMode == PV_MPPT_PO) pvRunMpptPO();
}

// ============================================================
// Arranque/parada y presets
// ============================================================
void stopBuckboost(const char* reason = "OFF") {
  bool wasRunning = (runMode != RUN_OFF);
  runMode = RUN_OFF;
  tonFracAcc = 0.0f;
  bbSafeStop(wasRunning);
  Serial.print("Buckboost: ");
  Serial.println(reason);
}

void stopAllPower(const char* reason = "OFF") {
  stopBuckboost(reason);
  pvMode = PV_OFF;
  pvOutputsSafeOff();
  Serial.print("PV boost: ");
  Serial.println(reason);
}

void setOperationMode(OperationMode newMode, bool persist = true) {
  if (newMode != operationMode) {
    stopAllPower("cambio testmode");
  } else {
    stopAllPower("testmode reaplicado");
  }

  operationMode = newMode;
  if (operationMode == OP_BUCKBOOST_ONLY) {
    pvMode = PV_OFF;
    pvOutputsSafeOff();
  } else if (operationMode == OP_PV_ONLY) {
    stopBuckboost("PV_ONLY");
  }

  configureFastControlChannel();
  Serial.print("testmode=");
  Serial.println(operationModeName());
  if (persist) saveConfig();
}

void clearFaults() {
  faultLatched = false;
  pvFaultLatched = false;
  runMode = RUN_OFF;
  pvMode = PV_OFF;
  pvArmed = false;
  resetController();
  pvClearControlStates();
  bbAllOff();
  pvOutputsSafeOff();
  Serial.println("Faults limpiadas. Estado: buckboost OFF, pvboost OFF");
}

bool canStartBuckboost() {
  if (!buckboostEnabledByMode()) {
    Serial.println("No se puede partir buckboost: modo PV_ONLY activo. Usa testmode bb o testmode full.");
    return false;
  }
  if (calibrationMode) {
    Serial.println("No se puede partir buckboost: calmode activo.");
    return false;
  }
  if (faultLatched) {
    Serial.println("No se puede partir buckboost: fault latched. Usa clear_fault.");
    return false;
  }
  if (!updateTelemetryMeasurements()) {
    Serial.println("No se puede partir buckboost: lectura ADS fallida.");
    return false;
  }
  return checkProtections(false);
}

void startManual(float tonUs) {
  if (!canStartBuckboost()) return;
  tonUs = clampFloat(tonUs, 0.0f, getAllowedTonMax());
  runMode = RUN_MANUAL;
  manualTonUs = tonUs;
  tonCmdUs = tonUs;
  tonFracAcc = 0.0f;
  Serial.print("Modo manual ");
  Serial.print(topologyName());
  Serial.print(" Ton=");
  Serial.print(manualTonUs, 2);
  Serial.println(" us");
}

void startAutoFFPI() {
  if (!canStartBuckboost()) return;
  runMode = RUN_AUTO_FFPI;
  resetController();
  tonCmdUs = (topology == TOPO_BOOST) ? clampFloat(computeFeedforwardTon(), 0.0f, getAllowedTonMax()) : 0.0f;
  Serial.print("AUTO_FFPI iniciado en ");
  Serial.print(topologyName());
  Serial.print(" | Target=");
  Serial.print(targetV, 2);
  Serial.print(" V | Ton inicial=");
  Serial.print(tonCmdUs, 2);
  Serial.println(" us");
}

void setTopology(TopologyMode newTopology) {
  stopBuckboost("cambio de topologia");
  topology = newTopology;
  resetController();
  configureFastControlChannel();
  Serial.print("Topologia seleccionada: ");
  Serial.println(topologyName());
}

void applyBuckPreset() {
  stopBuckboost("preset BUCK agente 3 aplicado");
  topology = TOPO_BUCK;
  targetV = 5.00f;
  tonMaxUs = 85.0f;
  tonHardMaxUs = 92.0f;
  minOffUs = 8.0f;
  kp = 6.0f;
  ki = 1.0f;
  ffScale = 1.00f;
  buckDiodeDrop = 0.65f;
  maxTonStepUpUs = 0.5f;
  maxTonStepDownUs = 3.0f;
  resetController();
  configureFastControlChannel();
}

void applyBoostPreset() {
  stopBuckboost("preset BOOST agente 3 aplicado");
  topology = TOPO_BOOST;
  targetV = 8.20f;
  tonMaxUs = 55.0f;
  tonHardMaxUs = 65.0f;
  minOffUs = 8.0f;
  kp = 10.0f;
  ki = 2.0f;
  ffScale = 1.10f;
  maxTonStepUpUs = 1.0f;
  maxTonStepDownUs = 2.0f;
  resetController();
  configureFastControlChannel();
}

void applyPvMpptSafePreset() {
  if (!pvBoostEnabledByMode()) {
    pvOutputsSafeOff();
    Serial.println("preset PV MPPT rechazado: modo BUCKBOOST_ONLY activo.");
    return;
  }
  pvEnterMode(PV_MPPT_PO);
  pvInputCurrentMax = 3.0f;
  pvBatteryChargeCurrentMax = 1.0f;
  pvInputPowerMax = 16.0f;
  pvDutyMax = 0.48f;
  pvVpvRefMin = 4.20f;
  pvVpvRefMax = 5.40f;
  pvVpvRef = 5.00f;
  mpptStepV = 0.025f;
  mpptPeriodMs = 1000;
  Serial.println("preset PV MPPT safe aplicado");
}

bool pvPrearmCheck(bool verbose) {
  if (!pvBoostEnabledByMode()) {
    pvOutputsSafeOff();
    if (verbose) Serial.println("PV prearm rechazado: modo BUCKBOOST_ONLY activo.");
    return false;
  }
  pvOutputsSafeOff();
  bool ok = updateTelemetryMeasurements();
  ok = ok && adsPresent(adsMpptAddr);

  bool sensorsOk = ok &&
    isfinite(meas.ads2[ADS2_CH_PANEL]) &&
    isfinite(meas.ads1[ADS1_CH_BAT2S]) &&
    isfinite(meas.ads2[ADS2_CH_IPANEL]) &&
    meas.vBat2s <= pvBatTripHigh &&
    meas.vPanel >= pvPanelTripLow;

  if (verbose) {
    Serial.println();
    Serial.println("=== PV BOOST PREARM ===");
    Serial.print("ADS2/read: ");
    Serial.println(ok ? "PASS" : "FAIL");
    Serial.print("VPV/VBAT rango: ");
    Serial.println(sensorsOk ? "PASS" : "FAIL");
    Serial.print("PWM pin asumido GPIO");
    Serial.print(PIN_PVBOOST_PWM);
    Serial.print(" | EN GPIO");
    Serial.println(PIN_PVBOOST_EN);
  }

  return ok && sensorsOk && !pvFaultLatched;
}

void pvArm() {
  if (!pvBoostEnabledByMode()) {
    pvOutputsSafeOff();
    Serial.println("PV ARM rechazado: modo BUCKBOOST_ONLY activo. Usa testmode pv o testmode full.");
    return;
  }
  if (calibrationMode) {
    Serial.println("PV ARM rechazado: calmode activo.");
    return;
  }
  if (!pvPrearmCheck(true)) {
    Serial.println("PV ARM rechazado.");
    return;
  }
  pvFaultLatched = false;
  pvClearControlStates();
  pvWriteDuty(0.0f);
  pvEnableWrite(true);
  pvArmed = true;
  Serial.println("pv_armed=YES");
}

void pvDisarm() {
  pvOutputsSafeOff();
  Serial.println("pv_armed=NO");
}

// ============================================================
// Configuracion persistente
// ============================================================
void restoreFirmwareDefaults() {
  operationMode = OP_FULL;
  topology = TOPO_BUCK;
  runMode = RUN_OFF;
  pvMode = PV_OFF;
  faultLatched = false;
  pvFaultLatched = false;
  pvArmed = false;

  adsMainAddr = ADS_MAIN_DEFAULT;
  adsMpptAddr = ADS_MPPT_DEFAULT;

  divFactorBatMid = 3.12f;
  divFactorSpare = 3.12f;
  divFactorBus5 = 3.12f;
  divFactorPv2s = 3.12f;
  divFactorPanel = 3.12f;

  calIBus5 = CurrentCal();
  calIPanel = CurrentCal();
  calIBat = CurrentCal();
  calIPanel.sign = -1; // Como el firmware MPPT original.

  boostKclEfficiency = 0.90f;

  targetV = 5.00f;
  tonMinUs = 0.0f;
  tonMaxUs = 85.0f;
  tonHardMaxUs = 92.0f;
  minOffUs = 8.0f;
  kp = 10.0f;
  ki = 5.0f;
  ffScale = 1.50f;
  buckDiodeDrop = 0.65f;
  piIntegral = 0.0f;
  tonCmdUs = 0.0f;
  manualTonUs = 10.0f;
  tonFracAcc = 0.0f;
  maxTonStepUpUs = 0.5f;
  maxTonStepDownUs = 3.0f;
  controlFilterAlpha = 0.25f;
  overshootDeadbandV = 0.03f;
  smallOvershootDecay = 0.995f;
  pwmPeriodUs = 100;
  controlPeriodUs = 1163;
  fullReadPeriodMs = 200;
  printPeriodMs = 1000;
  autoLog = true;
  bbSafeStopSteps = 12;
  bbInductorDischargeUs = 1500;
  pvSafeStopSteps = 12;
  pvInductorDischargeUs = 1500;
  calibrationMode = false;
  currentFaultInhibitUntilMs = 0;
  calibrationInhibitMs = 60000;

  boostPv2sTripHigh = 8.70f;
  boostPv2sWarnHigh = 8.45f;
  boostBus5TripLow = 4.40f;
  boostBus5WarnLow = 4.70f;
  buckPv2sTripLow = 6.00f;
  buckPv2sTripHigh = 8.90f;
  buckBus5TripHigh = 5.60f;
  buckBus5WarnHigh = 5.30f;

  pvFixedDuty = 0.05f;
  pvDutyCmd = 0.0f;
  pvDutyApplied = 0.0f;
  pvVpvRef = 5.00f;
  pvVpvRefMin = 4.20f;
  pvVpvRefMax = 5.40f;
  pvKpVpv = 0.060f;
  pvKiVpv = 0.900f;
  pvDutyMin = 0.0f;
  pvDutyStart = 0.03f;
  pvDutyMax = 0.48f;
  pvDutySlewPerStep = 0.010f;
  pvSoftstartMs = 3000;
  pvControlPeriodMs = 50;
  pvPanelTripLow = 3.00f;
  pvBatWarnHigh = 8.45f;
  pvBatTripHigh = 8.70f;
  pvInputCurrentMax = 3.00f;
  pvBatteryChargeCurrentMax = 1.00f;
  pvInputPowerMax = 16.0f;
  pvReverseCurrentMax = 0.30f;
  mpptStepV = 0.025f;
  mpptEpsPowerW = 0.08f;
  mpptMinPowerW = 1.0f;
  mpptPeriodMs = 1000;
}

void sanitizeCurrentCal(CurrentCal& cal) {
  cal.sens = clampFloat(cal.sens, 0.020f, 0.300f);
  cal.alpha = clampFloat(cal.alpha, 0.0f, 1.0f);
  cal.deadband = clampFloat(cal.deadband, 0.0f, 1.0f);
  cal.sign = (cal.sign < 0) ? -1 : 1;
}

void sanitizeConfig() {
  if (operationMode != OP_FULL &&
      operationMode != OP_BUCKBOOST_ONLY &&
      operationMode != OP_PV_ONLY) {
    operationMode = OP_FULL;
  }

  adsMainAddr = clampU8(adsMainAddr, 0x03, 0x77);
  adsMpptAddr = clampU8(adsMpptAddr, 0x03, 0x77);
  if (adsMainAddr == adsMpptAddr) adsMpptAddr = ADS_MPPT_DEFAULT;

  divFactorBatMid = clampFloat(divFactorBatMid, 0.10f, 50.0f);
  divFactorSpare = clampFloat(divFactorSpare, 0.10f, 50.0f);
  divFactorBus5 = clampFloat(divFactorBus5, 0.10f, 50.0f);
  divFactorPv2s = clampFloat(divFactorPv2s, 0.10f, 50.0f);
  divFactorPanel = clampFloat(divFactorPanel, 0.10f, 50.0f);
  sanitizeCurrentCal(calIBus5);
  sanitizeCurrentCal(calIPanel);
  sanitizeCurrentCal(calIBat);
  boostKclEfficiency = clampFloat(boostKclEfficiency, 0.20f, 1.05f);

  targetV = clampFloat(targetV, 0.50f, 9.0f);
  tonMinUs = clampFloat(tonMinUs, 0.0f, 500.0f);
  minOffUs = clampFloat(minOffUs, 0.0f, 40.0f);
  tonHardMaxUs = clampFloat(tonHardMaxUs, 0.0f, 500.0f);
  tonMaxUs = clampFloat(tonMaxUs, 0.0f, tonHardMaxUs);
  kp = clampFloat(kp, 0.0f, 40.0f);
  ki = clampFloat(ki, 0.0f, 40.0f);
  ffScale = clampFloat(ffScale, 0.0f, 2.0f);
  buckDiodeDrop = clampFloat(buckDiodeDrop, 0.0f, 1.2f);
  maxTonStepUpUs = clampFloat(maxTonStepUpUs, 0.01f, 10.0f);
  maxTonStepDownUs = clampFloat(maxTonStepDownUs, 0.01f, 20.0f);
  controlFilterAlpha = clampFloat(controlFilterAlpha, 0.01f, 1.0f);
  overshootDeadbandV = clampFloat(overshootDeadbandV, 0.0f, 0.25f);
  smallOvershootDecay = clampFloat(smallOvershootDecay, 0.90f, 1.0f);
  pwmPeriodUs = (uint32_t)clampFloat((float)pwmPeriodUs, 50.0f, 500.0f);
  controlPeriodUs = (uint32_t)clampFloat((float)controlPeriodUs, 1163.0f, 200000.0f);
  fullReadPeriodMs = (uint32_t)clampFloat((float)fullReadPeriodMs, 20.0f, 5000.0f);
  printPeriodMs = (uint32_t)clampFloat((float)printPeriodMs, 50.0f, 60000.0f);
  bbSafeStopSteps = clampU8(bbSafeStopSteps, 0, 40);
  bbInductorDischargeUs = (uint16_t)clampFloat((float)bbInductorDischargeUs, 0.0f, 20000.0f);
  pvSafeStopSteps = clampU8(pvSafeStopSteps, 0, 40);
  pvInductorDischargeUs = (uint16_t)clampFloat((float)pvInductorDischargeUs, 0.0f, 20000.0f);
  calibrationInhibitMs = (uint32_t)clampFloat((float)calibrationInhibitMs, 1000.0f, 600000.0f);

  boostPv2sTripHigh = clampFloat(boostPv2sTripHigh, 3.0f, 12.0f);
  boostPv2sWarnHigh = clampFloat(boostPv2sWarnHigh, 3.0f, 12.0f);
  boostBus5TripLow = clampFloat(boostBus5TripLow, 0.0f, 6.0f);
  boostBus5WarnLow = clampFloat(boostBus5WarnLow, 0.0f, 6.0f);
  buckPv2sTripLow = clampFloat(buckPv2sTripLow, 0.0f, 12.0f);
  buckPv2sTripHigh = clampFloat(buckPv2sTripHigh, 0.0f, 12.0f);
  buckBus5TripHigh = clampFloat(buckBus5TripHigh, 0.0f, 8.0f);
  buckBus5WarnHigh = clampFloat(buckBus5WarnHigh, 0.0f, 8.0f);

  pvFixedDuty = clampFloat(pvFixedDuty, 0.0f, pvDutyMax);
  pvVpvRefMin = clampFloat(pvVpvRefMin, 0.0f, pvVpvRefMax);
  pvVpvRefMax = fmaxf(pvVpvRefMax, pvVpvRefMin);
  pvVpvRef = clampFloat(pvVpvRef, pvVpvRefMin, pvVpvRefMax);
  pvKpVpv = clampFloat(pvKpVpv, 0.0f, 5.0f);
  pvKiVpv = clampFloat(pvKiVpv, 0.0f, 20.0f);
  pvDutyMax = clampFloat(pvDutyMax, 0.02f, 0.80f);
  pvDutyStart = clampFloat(pvDutyStart, 0.0f, pvDutyMax);
  pvDutySlewPerStep = clampFloat(pvDutySlewPerStep, 0.0005f, 0.10f);
  pvControlPeriodMs = (uint32_t)clampFloat((float)pvControlPeriodMs, 10.0f, 1000.0f);
  pvPanelTripLow = clampFloat(pvPanelTripLow, 0.0f, 10.0f);
  pvBatTripHigh = clampFloat(pvBatTripHigh, 0.0f, 12.0f);
  pvBatWarnHigh = clampFloat(pvBatWarnHigh, 0.0f, pvBatTripHigh);
  pvInputCurrentMax = clampFloat(pvInputCurrentMax, 0.05f, 20.0f);
  pvBatteryChargeCurrentMax = clampFloat(pvBatteryChargeCurrentMax, 0.05f, 20.0f);
  pvInputPowerMax = clampFloat(pvInputPowerMax, 0.5f, 100.0f);
  pvReverseCurrentMax = clampFloat(pvReverseCurrentMax, 0.0f, 5.0f);
  mpptStepV = clampFloat(mpptStepV, 0.001f, 0.25f);
  mpptEpsPowerW = clampFloat(mpptEpsPowerW, 0.0f, 5.0f);
  mpptMinPowerW = clampFloat(mpptMinPowerW, 0.0f, pvInputPowerMax);
  mpptPeriodMs = max<uint32_t>(mpptPeriodMs, 100);

  manualTonUs = clampFloat(manualTonUs, 0.0f, getAllowedTonMax());
  tonCmdUs = clampFloat(tonCmdUs, 0.0f, getAllowedTonMax());
}

bool saveConfig(bool verbose) {
  sanitizeConfig();
  Preferences prefs;
  if (!prefs.begin(CFG_NAMESPACE, false)) {
    if (verbose) Serial.println("cfg_saved=NO");
    return false;
  }

  bool ok = prefs.clear();
#define CFG_PUT(expr) do { if ((expr) == 0) ok = false; } while (0)

  CFG_PUT(prefs.putUShort("ver", CFG_VERSION));
  CFG_PUT(prefs.putUChar("opmode", (uint8_t)operationMode));
  CFG_PUT(prefs.putUChar("topo", (uint8_t)topology));
  CFG_PUT(prefs.putUChar("ads1", adsMainAddr));
  CFG_PUT(prefs.putUChar("ads2", adsMpptAddr));

  CFG_PUT(prefs.putFloat("divmid", divFactorBatMid));
  CFG_PUT(prefs.putFloat("divspare", divFactorSpare));
  CFG_PUT(prefs.putFloat("divbus5", divFactorBus5));
  CFG_PUT(prefs.putFloat("divbat2s", divFactorPv2s));
  CFG_PUT(prefs.putFloat("divpanel", divFactorPanel));
  CFG_PUT(prefs.putFloat("eta", boostKclEfficiency));

  CFG_PUT(prefs.putFloat("zibus", calIBus5.zero));
  CFG_PUT(prefs.putFloat("sibus", calIBus5.sens));
  CFG_PUT(prefs.putChar("gibus", calIBus5.sign));
  CFG_PUT(prefs.putFloat("aibus", calIBus5.alpha));
  CFG_PUT(prefs.putFloat("dibus", calIBus5.deadband));
  CFG_PUT(prefs.putFloat("zipv", calIPanel.zero));
  CFG_PUT(prefs.putFloat("sipv", calIPanel.sens));
  CFG_PUT(prefs.putChar("gipv", calIPanel.sign));
  CFG_PUT(prefs.putFloat("aipv", calIPanel.alpha));
  CFG_PUT(prefs.putFloat("dipv", calIPanel.deadband));
  CFG_PUT(prefs.putFloat("zibat", calIBat.zero));
  CFG_PUT(prefs.putFloat("sibat", calIBat.sens));
  CFG_PUT(prefs.putChar("gibat", calIBat.sign));
  CFG_PUT(prefs.putFloat("aibat", calIBat.alpha));
  CFG_PUT(prefs.putFloat("dibat", calIBat.deadband));

  CFG_PUT(prefs.putFloat("target", targetV));
  CFG_PUT(prefs.putFloat("tonmax", tonMaxUs));
  CFG_PUT(prefs.putFloat("tonhard", tonHardMaxUs));
  CFG_PUT(prefs.putFloat("minoff", minOffUs));
  CFG_PUT(prefs.putFloat("kp", kp));
  CFG_PUT(prefs.putFloat("ki", ki));
  CFG_PUT(prefs.putFloat("ff", ffScale));
  CFG_PUT(prefs.putFloat("buckvd", buckDiodeDrop));
  CFG_PUT(prefs.putFloat("slewup", maxTonStepUpUs));
  CFG_PUT(prefs.putFloat("slewdn", maxTonStepDownUs));
  CFG_PUT(prefs.putFloat("vfilt", controlFilterAlpha));
  CFG_PUT(prefs.putFloat("ovdead", overshootDeadbandV));
  CFG_PUT(prefs.putFloat("ovdecay", smallOvershootDecay));
  CFG_PUT(prefs.putUInt("pwmus", pwmPeriodUs));
  CFG_PUT(prefs.putUInt("ctrlus", controlPeriodUs));
  CFG_PUT(prefs.putUInt("fullms", fullReadPeriodMs));
  CFG_PUT(prefs.putUInt("printms", printPeriodMs));
  CFG_PUT(prefs.putBool("autolog", autoLog));
  CFG_PUT(prefs.putUChar("safebb", bbSafeStopSteps));
  CFG_PUT(prefs.putUShort("bbdisus", bbInductorDischargeUs));
  CFG_PUT(prefs.putUChar("safepv", pvSafeStopSteps));
  CFG_PUT(prefs.putUShort("pvdisus", pvInductorDischargeUs));
  CFG_PUT(prefs.putUInt("calinhms", calibrationInhibitMs));

  CFG_PUT(prefs.putFloat("bpv2shi", boostPv2sTripHigh));
  CFG_PUT(prefs.putFloat("bpv2swrn", boostPv2sWarnHigh));
  CFG_PUT(prefs.putFloat("bbus5lo", boostBus5TripLow));
  CFG_PUT(prefs.putFloat("bbus5warn", boostBus5WarnLow));
  CFG_PUT(prefs.putFloat("bkpv2slo", buckPv2sTripLow));
  CFG_PUT(prefs.putFloat("bkpv2shi", buckPv2sTripHigh));
  CFG_PUT(prefs.putFloat("bkbus5hi", buckBus5TripHigh));
  CFG_PUT(prefs.putFloat("bkbus5wr", buckBus5WarnHigh));

  CFG_PUT(prefs.putFloat("pvduty", pvFixedDuty));
  CFG_PUT(prefs.putFloat("pvref", pvVpvRef));
  CFG_PUT(prefs.putFloat("pvrefmin", pvVpvRefMin));
  CFG_PUT(prefs.putFloat("pvrefmax", pvVpvRefMax));
  CFG_PUT(prefs.putFloat("pvkp", pvKpVpv));
  CFG_PUT(prefs.putFloat("pvki", pvKiVpv));
  CFG_PUT(prefs.putFloat("pvdmax", pvDutyMax));
  CFG_PUT(prefs.putFloat("pvdstart", pvDutyStart));
  CFG_PUT(prefs.putFloat("pvslew", pvDutySlewPerStep));
  CFG_PUT(prefs.putUInt("pvssms", pvSoftstartMs));
  CFG_PUT(prefs.putUInt("pvctrlms", pvControlPeriodMs));
  CFG_PUT(prefs.putFloat("pvmin", pvPanelTripLow));
  CFG_PUT(prefs.putFloat("pvbwarn", pvBatWarnHigh));
  CFG_PUT(prefs.putFloat("pvbtrip", pvBatTripHigh));
  CFG_PUT(prefs.putFloat("pvimax", pvInputCurrentMax));
  CFG_PUT(prefs.putFloat("pvibatmax", pvBatteryChargeCurrentMax));
  CFG_PUT(prefs.putFloat("pvpmax", pvInputPowerMax));
  CFG_PUT(prefs.putFloat("pvirev", pvReverseCurrentMax));
  CFG_PUT(prefs.putFloat("mstep", mpptStepV));
  CFG_PUT(prefs.putFloat("meps", mpptEpsPowerW));
  CFG_PUT(prefs.putFloat("mpmin", mpptMinPowerW));
  CFG_PUT(prefs.putUInt("mms", mpptPeriodMs));
  if (ok) {
    CFG_PUT(prefs.putUInt("magic", CFG_MAGIC));
  } else {
    prefs.remove("magic");
  }
#undef CFG_PUT
  prefs.end();

  configSavePending = !ok;
  if (verbose) Serial.println(ok ? "cfg_saved=YES" : "cfg_saved=NO");
  return ok;
}

bool loadConfig(bool verbose) {
  Preferences prefs;
  if (!prefs.begin(CFG_NAMESPACE, true)) {
    if (verbose) Serial.println("cfg_loaded=NO");
    return false;
  }

  bool ok = prefs.getUInt("magic", 0) == CFG_MAGIC;
  if (ok) {
    uint8_t op = prefs.getUChar("opmode", (uint8_t)operationMode);
    if (op == (uint8_t)OP_BUCKBOOST_ONLY) operationMode = OP_BUCKBOOST_ONLY;
    else if (op == (uint8_t)OP_PV_ONLY) operationMode = OP_PV_ONLY;
    else operationMode = OP_FULL;

    uint8_t topo = prefs.getUChar("topo", (uint8_t)topology);
    topology = (topo == (uint8_t)TOPO_BOOST) ? TOPO_BOOST : TOPO_BUCK;
    adsMainAddr = prefs.getUChar("ads1", adsMainAddr);
    adsMpptAddr = prefs.getUChar("ads2", adsMpptAddr);

    divFactorBatMid = prefs.getFloat("divmid", divFactorBatMid);
    divFactorSpare = prefs.getFloat("divspare", divFactorSpare);
    divFactorBus5 = prefs.getFloat("divbus5", divFactorBus5);
    divFactorPv2s = prefs.getFloat("divbat2s", divFactorPv2s);
    divFactorPanel = prefs.getFloat("divpanel", divFactorPanel);
    boostKclEfficiency = prefs.getFloat("eta", boostKclEfficiency);

    calIBus5.zero = prefs.getFloat("zibus", calIBus5.zero);
    calIBus5.sens = prefs.getFloat("sibus", calIBus5.sens);
    calIBus5.sign = prefs.getChar("gibus", calIBus5.sign);
    calIBus5.alpha = prefs.getFloat("aibus", calIBus5.alpha);
    calIBus5.deadband = prefs.getFloat("dibus", calIBus5.deadband);
    calIPanel.zero = prefs.getFloat("zipv", calIPanel.zero);
    calIPanel.sens = prefs.getFloat("sipv", calIPanel.sens);
    calIPanel.sign = prefs.getChar("gipv", calIPanel.sign);
    calIPanel.alpha = prefs.getFloat("aipv", calIPanel.alpha);
    calIPanel.deadband = prefs.getFloat("dipv", calIPanel.deadband);
    calIBat.zero = prefs.getFloat("zibat", calIBat.zero);
    calIBat.sens = prefs.getFloat("sibat", calIBat.sens);
    calIBat.sign = prefs.getChar("gibat", calIBat.sign);
    calIBat.alpha = prefs.getFloat("aibat", calIBat.alpha);
    calIBat.deadband = prefs.getFloat("dibat", calIBat.deadband);

    targetV = prefs.getFloat("target", targetV);
    tonMaxUs = prefs.getFloat("tonmax", tonMaxUs);
    tonHardMaxUs = prefs.getFloat("tonhard", tonHardMaxUs);
    minOffUs = prefs.getFloat("minoff", minOffUs);
    kp = prefs.getFloat("kp", kp);
    ki = prefs.getFloat("ki", ki);
    ffScale = prefs.getFloat("ff", ffScale);
    buckDiodeDrop = prefs.getFloat("buckvd", buckDiodeDrop);
    maxTonStepUpUs = prefs.getFloat("slewup", maxTonStepUpUs);
    maxTonStepDownUs = prefs.getFloat("slewdn", maxTonStepDownUs);
    controlFilterAlpha = prefs.getFloat("vfilt", controlFilterAlpha);
    overshootDeadbandV = prefs.getFloat("ovdead", overshootDeadbandV);
    smallOvershootDecay = prefs.getFloat("ovdecay", smallOvershootDecay);
    pwmPeriodUs = prefs.getUInt("pwmus", pwmPeriodUs);
    controlPeriodUs = prefs.getUInt("ctrlus", controlPeriodUs);
    fullReadPeriodMs = prefs.getUInt("fullms", fullReadPeriodMs);
    printPeriodMs = prefs.getUInt("printms", printPeriodMs);
    autoLog = prefs.getBool("autolog", autoLog);
    bbSafeStopSteps = prefs.getUChar("safebb", bbSafeStopSteps);
    bbInductorDischargeUs = prefs.getUShort("bbdisus", bbInductorDischargeUs);
    pvSafeStopSteps = prefs.getUChar("safepv", pvSafeStopSteps);
    pvInductorDischargeUs = prefs.getUShort("pvdisus", pvInductorDischargeUs);
    calibrationInhibitMs = prefs.getUInt("calinhms", calibrationInhibitMs);

    boostPv2sTripHigh = prefs.getFloat("bpv2shi", boostPv2sTripHigh);
    boostPv2sWarnHigh = prefs.getFloat("bpv2swrn", boostPv2sWarnHigh);
    boostBus5TripLow = prefs.getFloat("bbus5lo", boostBus5TripLow);
    boostBus5WarnLow = prefs.getFloat("bbus5warn", boostBus5WarnLow);
    buckPv2sTripLow = prefs.getFloat("bkpv2slo", buckPv2sTripLow);
    buckPv2sTripHigh = prefs.getFloat("bkpv2shi", buckPv2sTripHigh);
    buckBus5TripHigh = prefs.getFloat("bkbus5hi", buckBus5TripHigh);
    buckBus5WarnHigh = prefs.getFloat("bkbus5wr", buckBus5WarnHigh);

    pvFixedDuty = prefs.getFloat("pvduty", pvFixedDuty);
    pvVpvRef = prefs.getFloat("pvref", pvVpvRef);
    pvVpvRefMin = prefs.getFloat("pvrefmin", pvVpvRefMin);
    pvVpvRefMax = prefs.getFloat("pvrefmax", pvVpvRefMax);
    pvKpVpv = prefs.getFloat("pvkp", pvKpVpv);
    pvKiVpv = prefs.getFloat("pvki", pvKiVpv);
    pvDutyMax = prefs.getFloat("pvdmax", pvDutyMax);
    pvDutyStart = prefs.getFloat("pvdstart", pvDutyStart);
    pvDutySlewPerStep = prefs.getFloat("pvslew", pvDutySlewPerStep);
    pvSoftstartMs = prefs.getUInt("pvssms", pvSoftstartMs);
    pvControlPeriodMs = prefs.getUInt("pvctrlms", pvControlPeriodMs);
    pvPanelTripLow = prefs.getFloat("pvmin", pvPanelTripLow);
    pvBatWarnHigh = prefs.getFloat("pvbwarn", pvBatWarnHigh);
    pvBatTripHigh = prefs.getFloat("pvbtrip", pvBatTripHigh);
    pvInputCurrentMax = prefs.getFloat("pvimax", pvInputCurrentMax);
    pvBatteryChargeCurrentMax = prefs.getFloat("pvibatmax", pvBatteryChargeCurrentMax);
    pvInputPowerMax = prefs.getFloat("pvpmax", pvInputPowerMax);
    pvReverseCurrentMax = prefs.getFloat("pvirev", pvReverseCurrentMax);
    mpptStepV = prefs.getFloat("mstep", mpptStepV);
    mpptEpsPowerW = prefs.getFloat("meps", mpptEpsPowerW);
    mpptMinPowerW = prefs.getFloat("mpmin", mpptMinPowerW);
    mpptPeriodMs = prefs.getUInt("mms", mpptPeriodMs);
  }
  prefs.end();

  if (ok) {
    sanitizeConfig();
    resetController();
    pvClearControlStates();
    runMode = RUN_OFF;
    pvMode = PV_OFF;
    pvArmed = false;
    configSavePending = false;
  }

  if (verbose) Serial.println(ok ? "cfg_loaded=YES" : "cfg_loaded=NO");
  return ok;
}

void factoryResetConfig() {
  stopAllPower("factoryreset");
  Preferences prefs;
  if (!prefs.begin(CFG_NAMESPACE, false)) {
    Serial.println("factoryreset=NO");
    return;
  }
  prefs.clear();
  prefs.end();

  restoreFirmwareDefaults();
  sanitizeConfig();
  configureHardwarePwm();
  configurePvBoostPwm();
  configureFastControlChannel();
  bbAllOff();
  pvOutputsSafeOff();
  Serial.println("factoryreset=YES");
}

// ============================================================
// Calibracion
// ============================================================
bool parseFloatAfterPrefix(const String& cmd, const char* prefix, float& value) {
  String p(prefix);
  if (!cmd.startsWith(p)) return false;

  String raw = cmd.substring(p.length());
  raw.trim();
  raw.replace(',', '.');
  if (raw.length() == 0) return false;

  char* endPtr = nullptr;
  float parsed = strtof(raw.c_str(), &endPtr);
  if (endPtr == raw.c_str()) return false;
  while (*endPtr == ' ' || *endPtr == '\t') ++endPtr;
  if (*endPtr != '\0') return false;

  value = parsed;
  return true;
}

float parseValueAfterSpace(const String& cmd) {
  int p = cmd.indexOf(' ');
  if (p < 0) return 0.0f;
  return cmd.substring(p + 1).toFloat();
}

void extendCurrentFaultInhibit(uint32_t durationMs) {
  currentFaultInhibitUntilMs = millis() + durationMs;
}

uint32_t currentFaultInhibitRemainingMs() {
  if (!currentFaultsInhibited()) return 0;
  int32_t remain = (int32_t)(currentFaultInhibitUntilMs - millis());
  return (remain > 0) ? (uint32_t)remain : 0;
}

void enterCalibrationMode(uint32_t durationMs = 0) {
  stopAllPower("calmode");
  calibrationMode = true;
  extendCurrentFaultInhibit(durationMs ? durationMs : calibrationInhibitMs);
  Serial.print("calmode=ON current_faults_inhibited_ms=");
  Serial.println(currentFaultInhibitRemainingMs());
}

void exitCalibrationMode() {
  calibrationMode = false;
  currentFaultInhibitUntilMs = 0;
  configureFastControlChannel();
  Serial.println("calmode=OFF");
}

bool parseCurrentSettingCommand(const String& cmd, const char* prefix, String& sensor, float& value) {
  String p(prefix);
  if (!cmd.startsWith(p)) return false;

  String rest = cmd.substring(p.length());
  rest.trim();
  int sp = rest.indexOf(' ');
  if (sp < 0) return false;

  sensor = rest.substring(0, sp);
  String raw = rest.substring(sp + 1);
  raw.trim();
  raw.replace(',', '.');
  if (raw.length() == 0) return false;

  char* endPtr = nullptr;
  float parsed = strtof(raw.c_str(), &endPtr);
  if (endPtr == raw.c_str()) return false;
  while (*endPtr == ' ' || *endPtr == '\t') ++endPtr;
  if (*endPtr != '\0') return false;

  value = parsed;
  return true;
}

CurrentCal* selectCurrentCal(const String& sensor, const char** labelOut = nullptr) {
  if (sensor == "ibus" || sensor == "ibus5" || sensor == "ibb") {
    if (labelOut) *labelOut = "ibus";
    return &calIBus5;
  }
  if (sensor == "ipv" || sensor == "ipanel") {
    if (labelOut) *labelOut = "ipv";
    return &calIPanel;
  }
  if (sensor == "ibat" || sensor == "bat") {
    if (labelOut) *labelOut = "ibat";
    return &calIBat;
  }
  return nullptr;
}

void printCurrentCal(const char* label, const CurrentCal& cal) {
  Serial.print("  ");
  Serial.print(label);
  Serial.print(": zero=");
  Serial.print(cal.zero, 6);
  Serial.print(" V sens=");
  Serial.print(cal.sens, 6);
  Serial.print(" V/A sign=");
  Serial.print(cal.sign);
  Serial.print(" alpha=");
  Serial.print(cal.alpha, 3);
  Serial.print(" deadband=");
  Serial.print(cal.deadband, 3);
  Serial.println(" A");
}

void printCalibrationStatus() {
  Serial.println();
  Serial.println("Mapa/calibracion mediciones:");
  Serial.print("  ADS1/main @");
  printHexAddr(adsMainAddr);
  Serial.println(": A0=BUS5, A1=Vbat_2S/PV2S, A2=spare, A3=Ibb_BUS5");
  Serial.print("  ADS2/mppt @");
  printHexAddr(adsMpptAddr);
  Serial.println(": A0=VPV, A1=Vbat_2S opcional/duplicado, A2=IPV, A3=Ibat");
  Serial.print("  Divisores: bus5=");
  Serial.print(divFactorBus5, 6);
  Serial.print(" bat2s=");
  Serial.print(divFactorPv2s, 6);
  Serial.print(" spare=");
  Serial.print(divFactorSpare, 6);
  Serial.print(" panel=");
  Serial.println(divFactorPanel, 6);
  Serial.println("  Corrientes:");
  printCurrentCal("ibus", calIBus5);
  printCurrentCal("ipv", calIPanel);
  printCurrentCal("ibat", calIBat);
  Serial.print("  calmode=");
  Serial.print(calibrationMode ? "ON" : "OFF");
  Serial.print(" current_faults_inhibited=");
  Serial.print(currentFaultsInhibited() ? "YES" : "NO");
  Serial.print(" remain_ms=");
  Serial.println(currentFaultInhibitRemainingMs());
}

bool handleCurrentCalCommand(const String& cmd) {
  String sensor;
  float value = 0.0f;
  const char* label = nullptr;
  CurrentCal* cal = nullptr;

  if (parseCurrentSettingCommand(cmd, "izero ", sensor, value)) {
    cal = selectCurrentCal(sensor, &label);
    if (cal != nullptr && isfinite(value)) {
      extendCurrentFaultInhibit(calibrationInhibitMs);
      cal->zero = clampFloat(value, 0.0f, 4.096f);
      cal->ready = false;
      Serial.print("izero_");
      Serial.print(label);
      Serial.print("=");
      Serial.println(cal->zero, 6);
      saveConfig();
    } else {
      Serial.println("izero fallo: usa ibus|ipv|ibat y Vadc valido.");
    }
    return true;
  }

  if (parseCurrentSettingCommand(cmd, "isens ", sensor, value)) {
    cal = selectCurrentCal(sensor, &label);
    if (cal != nullptr && isfinite(value)) {
      cal->sens = clampFloat(fabsf(value), 0.020f, 0.300f);
      cal->ready = false;
      Serial.print("isens_");
      Serial.print(label);
      Serial.print("=");
      Serial.println(cal->sens, 6);
      saveConfig();
    } else {
      Serial.println("isens fallo: usa ibus|ipv|ibat y V/A valido.");
    }
    return true;
  }

  if (parseCurrentSettingCommand(cmd, "ialpha ", sensor, value)) {
    value = clampFloat(value, 0.0f, 1.0f);
    if (sensor == "all") {
      calIBus5.alpha = value;
      calIPanel.alpha = value;
      calIBat.alpha = value;
      calIBus5.ready = calIPanel.ready = calIBat.ready = false;
      Serial.print("ialpha_all=");
      Serial.println(value, 3);
      saveConfig();
    } else {
      cal = selectCurrentCal(sensor, &label);
      if (cal != nullptr) {
        cal->alpha = value;
        cal->ready = false;
        Serial.print("ialpha_");
        Serial.print(label);
        Serial.print("=");
        Serial.println(cal->alpha, 3);
        saveConfig();
      } else {
        Serial.println("ialpha fallo: usa ibus|ipv|ibat|all.");
      }
    }
    return true;
  }

  if (parseCurrentSettingCommand(cmd, "idead ", sensor, value)) {
    value = clampFloat(value, 0.0f, 1.0f);
    if (sensor == "all") {
      calIBus5.deadband = value;
      calIPanel.deadband = value;
      calIBat.deadband = value;
      calIBus5.ready = calIPanel.ready = calIBat.ready = false;
      Serial.print("idead_all=");
      Serial.println(value, 3);
      saveConfig();
    } else {
      cal = selectCurrentCal(sensor, &label);
      if (cal != nullptr) {
        cal->deadband = value;
        cal->ready = false;
        Serial.print("idead_");
        Serial.print(label);
        Serial.print("=");
        Serial.println(cal->deadband, 3);
        saveConfig();
      } else {
        Serial.println("idead fallo: usa ibus|ipv|ibat|all.");
      }
    }
    return true;
  }

  return false;
}

bool calibrateVoltage(uint8_t addr, uint8_t ch, float& factor, float realV, const char* label) {
  if (!isfinite(realV) || realV <= 0.01f) {
    Serial.println("CAL fallo: usa un voltaje real mayor a 0.01 V.");
    return false;
  }

  if (runMode != RUN_OFF || pvMode != PV_OFF) {
    Serial.println("CAL aviso: calibrando voltaje con potencia activa; usa una condicion estable.");
  }

  float adc = 0.0f;
  if (!readAdsAverageVolts(addr, ch, 16, adc) || adc <= 0.005f) {
    Serial.println("CAL fallo: lectura ADS invalida o demasiado baja.");
    configureFastControlChannel();
    return false;
  }

  factor = clampFloat(realV / adc, 0.10f, 50.0f);
  controlOutputFilterReady = false;
  updateTelemetryMeasurements();
  configureFastControlChannel();

  Serial.print("cal_");
  Serial.print(label);
  Serial.print(": adc=");
  Serial.print(adc, 6);
  Serial.print(" V, real=");
  Serial.print(realV, 5);
  Serial.print(" V, factor=");
  Serial.println(factor, 6);
  saveConfig();
  return true;
}

void zeroCurrentSensor(uint8_t addr, uint8_t ch, CurrentCal& cal, const char* label) {
  extendCurrentFaultInhibit(calibrationInhibitMs);
  if (runMode != RUN_OFF || pvMode != PV_OFF) {
    Serial.println("ZERO bloqueo: deja buckboost y pvboost en OFF, sin corriente real.");
    return;
  }

  float adc = 0.0f;
  if (!readAdsAverageVolts(addr, ch, 64, adc)) {
    Serial.println("ZERO fallo: lectura ADS invalida.");
    configureFastControlChannel();
    return;
  }
  cal.zero = adc;
  cal.ready = false;
  configureFastControlChannel();

  Serial.print("zero_");
  Serial.print(label);
  Serial.print("=");
  Serial.println(cal.zero, 6);
  saveConfig();
}

void calibrateCurrentSensor(uint8_t addr, uint8_t ch, CurrentCal& cal, float realA, const char* label) {
  extendCurrentFaultInhibit(calibrationInhibitMs);
  if (!isfinite(realA) || fabsf(realA) < 0.02f) {
    Serial.println("CAL corriente fallo: usa una corriente conocida sobre 0.02 A.");
    return;
  }

  float adc = 0.0f;
  if (!readAdsAverageVolts(addr, ch, 32, adc)) {
    Serial.println("CAL corriente fallo: lectura ADS invalida.");
    configureFastControlChannel();
    return;
  }

  float delta = adc - cal.zero;
  if (fabsf(delta) < 0.001f) {
    Serial.println("CAL corriente fallo: delta respecto al cero demasiado pequeno.");
    configureFastControlChannel();
    return;
  }

  float signedSens = delta / realA;
  cal.sign = (signedSens >= 0.0f) ? 1 : -1;
  cal.sens = clampFloat(fabsf(signedSens), 0.020f, 0.300f);
  cal.ready = false;
  configureFastControlChannel();

  Serial.print("cal_");
  Serial.print(label);
  Serial.print(": zero=");
  Serial.print(cal.zero, 6);
  Serial.print(" sens=");
  Serial.print(cal.sens, 6);
  Serial.print(" sign=");
  Serial.println(cal.sign);
  saveConfig();
}

// ============================================================
// Serial UI
// ============================================================
void printSwitchingMap() {
  Serial.println();
  Serial.println("Mapa de conmutacion agente 3:");
  Serial.println("  Buckboost:");
  Serial.println("    GPIO25 -> QH/high-side, activo LOW.");
  Serial.println("    GPIO26 -> QL/low-side, activo HIGH.");
  Serial.println("    BUCK  2S->BUS5: QH PWM, QL OFF.");
  Serial.println("    BOOST BUS5->2S: QL PWM, QH OFF.");
  Serial.println("  Boost PV MPPT:");
  Serial.print("    GPIO");
  Serial.print(PIN_PVBOOST_PWM);
  Serial.println(" -> PWM boost PV low-side.");
  Serial.print("    GPIO");
  Serial.print(PIN_PVBOOST_EN);
  Serial.println(" -> enable boost PV (si esta cableado).");
}

void printStatus() {
  Serial.println();
  Serial.print("TestMode=");
  Serial.println(operationModeName());

  Serial.print("Buckboost Topo=");
  Serial.print(topologyName());
  Serial.print(" | Run=");
  Serial.print(runName());
  Serial.print(" | Fault=");
  Serial.print(faultLatched ? "SI" : "NO");
  Serial.print(" | PV2S/ADS1A1=");
  Serial.print(v_pv2s, 3);
  Serial.print(" V | BUS5/ADS1A0=");
  Serial.print(v_bus5, 3);
  Serial.print(" V | Vin=");
  Serial.print(getInputV(), 3);
  Serial.print(" V | Vout=");
  Serial.print(getOutputV(), 3);
  Serial.print(" V | Target=");
  Serial.print(targetV, 2);
  Serial.print(" V | Ton=");
  Serial.print(tonCmdUs, 3);
  Serial.print(" us | Allowed=");
  Serial.print(getAllowedTonMax(), 2);
  Serial.print(" us | Kp=");
  Serial.print(kp, 3);
  Serial.print(" | Ki=");
  Serial.print(ki, 3);
  Serial.print(" | ff=");
  Serial.println(ffScale, 3);

  Serial.print("Mediciones: Vspare/ADS1A2=");
  Serial.print(meas.vSpare, 3);
  Serial.print(" V | Ibb_BUS5/ADS1A3=");
  Serial.print(meas.iBbBus5, 3);
  Serial.println(" A");

  Serial.print("PV boost mode=");
  Serial.print(pvModeName());
  Serial.print(" | armed=");
  Serial.print(pvArmed ? "YES" : "NO");
  Serial.print(" | fault=");
  Serial.print(pvFaultLatched ? "SI" : "NO");
  Serial.print(" | VPV/ADS2A0=");
  Serial.print(meas.vPanel, 3);
  Serial.print(" V | IPV/ADS2A2=");
  Serial.print(meas.iPanel, 3);
  Serial.print(" A | Ibat/ADS2A3=");
  Serial.print(meas.iBat, 3);
  Serial.print(" A | PPV=");
  Serial.print(meas.pPanel, 3);
  Serial.println(" W");

  Serial.print("PV duty applied/cmd/ss=");
  Serial.print(pvDutyApplied, 4);
  Serial.print(" / ");
  Serial.print(pvDutyCmd, 4);
  Serial.print(" / ");
  Serial.print(pvDutySoftLimit, 4);
  Serial.print(" | VPVref=");
  Serial.print(pvVpvRef, 3);
  Serial.print(" | dmax=");
  Serial.print(pvDutyMax, 3);
  Serial.print(" | IbatMax=");
  Serial.print(pvBatteryChargeCurrentMax, 3);
  Serial.print(" A");
  Serial.print(" | Kp/Ki=");
  Serial.print(pvKpVpv, 4);
  Serial.print("/");
  Serial.println(pvKiVpv, 4);

  Serial.print("KCL nodo 2S: Iboost_est=");
  Serial.print(meas.iBoostNodeEst, 3);
  Serial.print(" A | Ibat=");
  Serial.print(meas.iBat, 3);
  Serial.print(" A | Ibb_in_2S=");
  Serial.print(meas.iBbIn2sKcl, 3);
  Serial.print(" A | eta=");
  Serial.println(boostKclEfficiency, 3);

  Serial.print("ADS1=");
  printHexAddr(adsMainAddr);
  Serial.print(" ok=");
  Serial.print(meas.ads1Ok ? "YES" : "NO");
  Serial.print(" | ADS2=");
  printHexAddr(adsMpptAddr);
  Serial.print(" ok=");
  Serial.print(meas.ads2Ok ? "YES" : "NO");
  Serial.print(" | cfg_pending=");
  Serial.print(configSavePending ? "YES" : "NO");
  Serial.print(" | calmode=");
  Serial.print(calibrationMode ? "ON" : "OFF");
  Serial.print(" | current_fault_inhibit=");
  Serial.print(currentFaultsInhibited() ? "YES" : "NO");
  Serial.print(" | safeoff BB/PV steps=");
  Serial.print(bbSafeStopSteps);
  Serial.print("/");
  Serial.println(pvSafeStopSteps);
}

void printHelp() {
  Serial.println();
  Serial.println("Comandos principales:");
  Serial.println("  help; status; read; adsraw; adsmap; calstatus; switching; prearm");
  Serial.println("  off; clear_fault; savecfg; loadcfg; factoryreset");
  Serial.println("  testmode full; testmode bb; testmode pv");
  Serial.println("  Los comandos se pueden encadenar con ';'");
  Serial.println();
  Serial.println("Buckboost:");
  Serial.println("  mode buck; mode boost; preset_buck; preset_boost");
  Serial.println("  ton 10; buck_ton 20; boost_ton 20; auto");
  Serial.println("  target 5.0; kp 6; ki 1; ff 1.0; period 100; ctrlms 10");
  Serial.println();
  Serial.println("PV boost / MPPT:");
  Serial.println("  pv_prearm; pv_arm; pv_disarm; pv_off");
  Serial.println("  pv_sensors; pv_fixed 0.05; pv_vpv; pv_mppt; preset_mppt");
  Serial.println("  pv_dmax 0.48; pv_duty 0.05; pv_vpvref 5.0");
  Serial.println("  pv_kp 0.06; pv_ki 0.9; pv_slew 0.01; pv_ctrlms 50");
  Serial.println("  pv_imax 3.0; pv_ibatmax 1.0; pv_pmax 16; pv_batwarn 8.45; pv_battrip 8.70");
  Serial.println("  mppt_step 0.025; mppt_ms 1000; mppt_epsp 0.08");
  Serial.println();
  Serial.println("Calibracion voltajes:");
  Serial.println("  cal bus5 <V>     -> ADS1 A0");
  Serial.println("  cal bat2s <V>    -> ADS1 A1");
  Serial.println("  cal spare <V>    -> ADS1 A2");
  Serial.println("  cal panel <V>    -> ADS2 A0");
  Serial.println();
  Serial.println("Calibracion corrientes:");
  Serial.println("  calmode on; calmode off; calhold 60");
  Serial.println("  zeroacs          -> cero en ibus, ipv, ibat");
  Serial.println("  zero ibus|ipv|ibat");
  Serial.println("  flip ibus|ipv|ibat");
  Serial.println("  cal ibus <A>; cal ipv <A>; cal ibat <A>");
  Serial.println("  izero ibus|ipv|ibat <Vadc>; isens ibus|ipv|ibat <V/A>");
  Serial.println("  ialpha ibus|ipv|ibat|all <0..1>; idead ibus|ipv|ibat|all <A>");
  Serial.println();
  Serial.println("Apagado seguro:");
  Serial.println("  safe_bb_steps 12; safe_pv_steps 12; bb_discharge_us 1500; pv_discharge_us 1500");
  Serial.println();
  Serial.println("ADS:");
  Serial.println("  adsaddr main 0x48; adsaddr mppt 0x49; adsswap");
  Serial.println("  divbus5 3.12; divbat2s 3.12; divspare 3.12; divpanel 3.12");
}

void setConfigPendingOrSave() {
  if (runMode != RUN_OFF || pvMode != PV_OFF) {
    configSavePending = true;
    Serial.println("cfg_save_deferred=YES: usa off; savecfg para guardar sin perturbar.");
  } else {
    saveConfig();
  }
}

void handleSerialCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd.length() == 0) return;
  if (cmd.startsWith("set ")) {
    cmd = cmd.substring(4);
    cmd.trim();
  }

  Serial.print("Comando recibido: ");
  Serial.println(cmd);

  float value = 0.0f;

  if (cmd == "help") printHelp();
  else if (cmd == "status") printStatus();
  else if (cmd == "switching" || cmd == "gatemap") printSwitchingMap();
  else if (cmd == "off") stopAllPower("OFF");
  else if (cmd == "clear_fault") clearFaults();
  else if (cmd == "read" || cmd == "sensors") {
    if (updateTelemetryMeasurements()) printStatus();
    else Serial.println("Lectura ADS fallida");
  }
  else if (cmd == "adsraw") printAdsRawChannels();
  else if (cmd == "adsmap" || cmd == "calstatus") printCalibrationStatus();
  else if (cmd == "prearm" || cmd == "selftest") prearmCheck(true);
  else if (cmd == "calmode on" || cmd == "cal on") enterCalibrationMode();
  else if (cmd == "calmode off" || cmd == "cal off") exitCalibrationMode();
  else if (parseFloatAfterPrefix(cmd, "calhold ", value) ||
           parseFloatAfterPrefix(cmd, "calmode ", value)) {
    calibrationInhibitMs = (uint32_t)(clampFloat(value, 1.0f, 600.0f) * 1000.0f);
    enterCalibrationMode(calibrationInhibitMs);
  }
  else if (cmd == "savecfg" || cmd == "savecal") setConfigPendingOrSave();
  else if (cmd == "loadcfg") {
    stopAllPower("loadcfg");
    if (loadConfig(true)) {
      configureHardwarePwm();
      configurePvBoostPwm();
      configureFastControlChannel();
      bbAllOff();
      pvOutputsSafeOff();
      updateTelemetryMeasurements();
      printStatus();
    }
  }
  else if (cmd == "factoryreset" || cmd == "clearcal") {
    factoryResetConfig();
    updateTelemetryMeasurements();
    printStatus();
  }
  else if (cmd == "testmode full" || cmd == "sysmode full" || cmd == "mode full" || cmd == "full") {
    setOperationMode(OP_FULL);
  }
  else if (cmd == "testmode bb" || cmd == "testmode buckboost" ||
           cmd == "sysmode bb" || cmd == "sysmode buckboost" ||
           cmd == "only_bb" || cmd == "bb_only") {
    setOperationMode(OP_BUCKBOOST_ONLY);
  }
  else if (cmd == "testmode pv" || cmd == "testmode mppt" ||
           cmd == "sysmode pv" || cmd == "sysmode mppt" ||
           cmd == "only_pv" || cmd == "pv_only" || cmd == "mppt_only") {
    setOperationMode(OP_PV_ONLY);
  }
  else if (cmd == "mode buck") setTopology(TOPO_BUCK);
  else if (cmd == "mode boost") setTopology(TOPO_BOOST);
  else if (cmd == "preset_buck" || cmd == "preset buck") {
    applyBuckPreset();
    printStatus();
  }
  else if (cmd == "preset_boost" || cmd == "preset boost") {
    applyBoostPreset();
    printStatus();
  }
  else if (cmd == "auto" || cmd == "auto_ffpi") startAutoFFPI();
  else if (cmd.startsWith("ton ")) startManual(parseValueAfterSpace(cmd));
  else if (cmd.startsWith("buck_ton ")) {
    setTopology(TOPO_BUCK);
    startManual(parseValueAfterSpace(cmd));
  }
  else if (cmd.startsWith("boost_ton ")) {
    setTopology(TOPO_BOOST);
    startManual(parseValueAfterSpace(cmd));
  }
  else if (cmd == "pv_prearm" || cmd == "mppt_prearm") pvPrearmCheck(true);
  else if (cmd == "pv_arm" || cmd == "mppt_arm") pvArm();
  else if (cmd == "pv_disarm" || cmd == "mppt_disarm") pvDisarm();
  else if (cmd == "pv_off" || cmd == "mppt_off") {
    pvMode = PV_OFF;
    pvOutputsSafeOff();
    Serial.println("pvboost OFF");
  }
  else if (cmd == "pv_sensors" || cmd == "mppt_sensors") pvEnterMode(PV_SENSORS);
  else if (cmd == "pv_vpv" || cmd == "mppt_vpv") pvEnterMode(PV_VPV_PI);
  else if (cmd == "pv_mppt" || cmd == "mppt_auto" || cmd == "mode mppt") pvEnterMode(PV_MPPT_PO);
  else if (cmd == "preset_mppt" || cmd == "preset mppt" || cmd == "preset mpptsafe") applyPvMpptSafePreset();
  else if (parseFloatAfterPrefix(cmd, "pv_fixed ", value)) {
    pvFixedDuty = clampFloat(value, 0.0f, pvDutyMax);
    pvEnterMode(PV_FIXED);
  }
  else if (parseFloatAfterPrefix(cmd, "pv_duty ", value)) {
    pvFixedDuty = clampFloat(value, 0.0f, pvDutyMax);
    Serial.print("pv_fixed_duty=");
    Serial.println(pvFixedDuty, 4);
  }
  else if (parseFloatAfterPrefix(cmd, "target ", value)) {
    targetV = clampFloat(value, 0.5f, 9.0f);
    resetController();
    Serial.print("Target actualizado a ");
    Serial.println(targetV, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "kp ", value)) {
    kp = clampFloat(value, 0.0f, 40.0f);
    Serial.print("Kp=");
    Serial.println(kp, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "ki ", value)) {
    ki = clampFloat(value, 0.0f, 40.0f);
    Serial.print("Ki=");
    Serial.println(ki, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "ff ", value)) {
    ffScale = clampFloat(value, 0.0f, 2.0f);
    Serial.print("ff=");
    Serial.println(ffScale, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "tonmax ", value)) {
    tonMaxUs = clampFloat(value, 0.0f, 99.0f);
    Serial.print("TonMax=");
    Serial.println(tonMaxUs, 2);
  }
  else if (parseFloatAfterPrefix(cmd, "tonhard ", value)) {
    tonHardMaxUs = clampFloat(value, 0.0f, 99.0f);
    if (tonMaxUs > tonHardMaxUs) tonMaxUs = tonHardMaxUs;
    Serial.print("TonHard=");
    Serial.println(tonHardMaxUs, 2);
  }
  else if (parseFloatAfterPrefix(cmd, "minoff ", value)) {
    minOffUs = clampFloat(value, 0.0f, 40.0f);
    Serial.print("MinOff=");
    Serial.println(minOffUs, 2);
  }
  else if (parseFloatAfterPrefix(cmd, "slewup ", value)) {
    maxTonStepUpUs = clampFloat(value, 0.01f, 10.0f);
    Serial.print("SlewUp=");
    Serial.println(maxTonStepUpUs, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "slewdown ", value)) {
    maxTonStepDownUs = clampFloat(value, 0.01f, 20.0f);
    Serial.print("SlewDown=");
    Serial.println(maxTonStepDownUs, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "vfilt ", value)) {
    controlFilterAlpha = clampFloat(value, 0.01f, 1.0f);
    controlOutputFilterReady = false;
    Serial.print("vfilt=");
    Serial.println(controlFilterAlpha, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "ovdead ", value)) {
    overshootDeadbandV = clampFloat(value, 0.0f, 0.25f);
    Serial.print("ovdead=");
    Serial.println(overshootDeadbandV, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "ovdecay ", value)) {
    smallOvershootDecay = clampFloat(value, 0.90f, 1.0f);
    Serial.print("ovdecay=");
    Serial.println(smallOvershootDecay, 4);
  }
  else if (parseFloatAfterPrefix(cmd, "buckvd ", value)) {
    buckDiodeDrop = clampFloat(value, 0.0f, 1.2f);
    Serial.print("buckvd=");
    Serial.println(buckDiodeDrop, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "period ", value)) {
    pwmPeriodUs = (uint32_t)clampFloat(value, 50.0f, 500.0f);
    sanitizeConfig();
    configureHardwarePwm();
    refreshBuckboostOutput();
    Serial.print("period=");
    Serial.println(pwmPeriodUs);
  }
  else if (parseFloatAfterPrefix(cmd, "ctrlms ", value)) {
    controlPeriodUs = (uint32_t)(clampFloat(value, 1.0f, 200.0f) * 1000.0f);
    if (controlPeriodUs < 1163UL) controlPeriodUs = 1163UL;
    Serial.print("ctrlus=");
    Serial.println(controlPeriodUs);
  }
  else if (parseFloatAfterPrefix(cmd, "fullms ", value)) {
    fullReadPeriodMs = (uint32_t)clampFloat(value, 20.0f, 5000.0f);
    Serial.print("fullms=");
    Serial.println(fullReadPeriodMs);
  }
  else if (parseFloatAfterPrefix(cmd, "printms ", value) || parseFloatAfterPrefix(cmd, "telms ", value)) {
    printPeriodMs = (uint32_t)clampFloat(value, 50.0f, 60000.0f);
    Serial.print("printms=");
    Serial.println(printPeriodMs);
  }
  else if (cmd == "log on") {
    autoLog = true;
    Serial.println("auto_log=ON");
  }
  else if (cmd == "log off") {
    autoLog = false;
    Serial.println("auto_log=OFF");
  }
  else if (parseFloatAfterPrefix(cmd, "safe_bb_steps ", value)) {
    bbSafeStopSteps = clampU8((int)lroundf(value), 0, 40);
    Serial.print("safe_bb_steps=");
    Serial.println(bbSafeStopSteps);
    saveConfig();
  }
  else if (parseFloatAfterPrefix(cmd, "safe_pv_steps ", value)) {
    pvSafeStopSteps = clampU8((int)lroundf(value), 0, 40);
    Serial.print("safe_pv_steps=");
    Serial.println(pvSafeStopSteps);
    saveConfig();
  }
  else if (parseFloatAfterPrefix(cmd, "bb_discharge_us ", value)) {
    bbInductorDischargeUs = (uint16_t)clampFloat(value, 0.0f, 20000.0f);
    Serial.print("bb_discharge_us=");
    Serial.println(bbInductorDischargeUs);
    saveConfig();
  }
  else if (parseFloatAfterPrefix(cmd, "pv_discharge_us ", value)) {
    pvInductorDischargeUs = (uint16_t)clampFloat(value, 0.0f, 20000.0f);
    Serial.print("pv_discharge_us=");
    Serial.println(pvInductorDischargeUs);
    saveConfig();
  }
  else if (parseFloatAfterPrefix(cmd, "divbatmid ", value)) {
    divFactorBus5 = clampFloat(value, 0.1f, 50.0f);
    Serial.println("divbatmid esta obsoleto: aplicado como divbus5 para ADS1 A0.");
    saveConfig();
  }
  else if (parseFloatAfterPrefix(cmd, "divspare ", value)) {
    divFactorSpare = clampFloat(value, 0.1f, 50.0f);
    saveConfig();
  }
  else if (parseFloatAfterPrefix(cmd, "divbus5 ", value)) {
    divFactorBus5 = clampFloat(value, 0.1f, 50.0f);
    saveConfig();
  }
  else if (parseFloatAfterPrefix(cmd, "divbat2s ", value) || parseFloatAfterPrefix(cmd, "divpv2s ", value)) {
    divFactorPv2s = clampFloat(value, 0.1f, 50.0f);
    saveConfig();
  }
  else if (parseFloatAfterPrefix(cmd, "divpanel ", value) || parseFloatAfterPrefix(cmd, "divvpv ", value)) {
    divFactorPanel = clampFloat(value, 0.1f, 50.0f);
    saveConfig();
  }
  else if (parseFloatAfterPrefix(cmd, "cal bus5 ", value) ||
           parseFloatAfterPrefix(cmd, "cal v5 ", value) ||
           parseFloatAfterPrefix(cmd, "cal a0 ", value)) {
    calibrateVoltage(adsMainAddr, ADS1_CH_BUS5, divFactorBus5, value, "bus5_ads1a0");
  }
  else if (parseFloatAfterPrefix(cmd, "cal bat2s ", value) ||
           parseFloatAfterPrefix(cmd, "cal pv2s ", value) ||
           parseFloatAfterPrefix(cmd, "cal vbat ", value) ||
           parseFloatAfterPrefix(cmd, "cal bat ", value) ||
           parseFloatAfterPrefix(cmd, "cal a1 ", value)) {
    calibrateVoltage(adsMainAddr, ADS1_CH_BAT2S, divFactorPv2s, value, "bat2s_ads1a1");
  }
  else if (parseFloatAfterPrefix(cmd, "cal spare ", value) ||
           parseFloatAfterPrefix(cmd, "cal a2 ", value)) {
    calibrateVoltage(adsMainAddr, ADS1_CH_SPARE, divFactorSpare, value, "spare_ads1a2");
  }
  else if (parseFloatAfterPrefix(cmd, "cal panel ", value) ||
           parseFloatAfterPrefix(cmd, "cal vpv ", value) ||
           parseFloatAfterPrefix(cmd, "cal pv ", value)) {
    calibrateVoltage(adsMpptAddr, ADS2_CH_PANEL, divFactorPanel, value, "panel_ads2a0");
  }
  else if (parseFloatAfterPrefix(cmd, "cal vin ", value)) {
    if (topology == TOPO_BUCK) calibrateVoltage(adsMainAddr, ADS1_CH_BAT2S, divFactorPv2s, value, "vin_bat2s");
    else calibrateVoltage(adsMainAddr, ADS1_CH_BUS5, divFactorBus5, value, "vin_bus5");
  }
  else if (parseFloatAfterPrefix(cmd, "cal vout ", value)) {
    if (topology == TOPO_BUCK) calibrateVoltage(adsMainAddr, ADS1_CH_BUS5, divFactorBus5, value, "vout_bus5");
    else calibrateVoltage(adsMainAddr, ADS1_CH_BAT2S, divFactorPv2s, value, "vout_bat2s");
  }
  else if (cmd == "zeroacs") {
    zeroCurrentSensor(adsMainAddr, ADS1_CH_IBUS5, calIBus5, "ibus");
    zeroCurrentSensor(adsMpptAddr, ADS2_CH_IPANEL, calIPanel, "ipv");
    zeroCurrentSensor(adsMpptAddr, ADS2_CH_IBAT, calIBat, "ibat");
  }
  else if (cmd == "zero ibus") zeroCurrentSensor(adsMainAddr, ADS1_CH_IBUS5, calIBus5, "ibus");
  else if (cmd == "zero ipv") zeroCurrentSensor(adsMpptAddr, ADS2_CH_IPANEL, calIPanel, "ipv");
  else if (cmd == "zero ibat") zeroCurrentSensor(adsMpptAddr, ADS2_CH_IBAT, calIBat, "ibat");
  else if (cmd == "flip ibus") {
    calIBus5.sign = -calIBus5.sign;
    calIBus5.ready = false;
    saveConfig();
  }
  else if (cmd == "flip ipv" || cmd == "currentflip") {
    calIPanel.sign = -calIPanel.sign;
    calIPanel.ready = false;
    saveConfig();
  }
  else if (cmd == "flip ibat") {
    calIBat.sign = -calIBat.sign;
    calIBat.ready = false;
    saveConfig();
  }
  else if (parseFloatAfterPrefix(cmd, "cal ibus ", value)) {
    calibrateCurrentSensor(adsMainAddr, ADS1_CH_IBUS5, calIBus5, value, "ibus");
  }
  else if (parseFloatAfterPrefix(cmd, "cal ipv ", value)) {
    calibrateCurrentSensor(adsMpptAddr, ADS2_CH_IPANEL, calIPanel, value, "ipv");
  }
  else if (parseFloatAfterPrefix(cmd, "cal ibat ", value)) {
    calibrateCurrentSensor(adsMpptAddr, ADS2_CH_IBAT, calIBat, value, "ibat");
  }
  else if (handleCurrentCalCommand(cmd)) {
  }
  else if (parseFloatAfterPrefix(cmd, "eta ", value) || parseFloatAfterPrefix(cmd, "kcl_eta ", value)) {
    boostKclEfficiency = clampFloat(value, 0.20f, 1.05f);
    Serial.print("kcl_eta=");
    Serial.println(boostKclEfficiency, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "pv_dmax ", value)) {
    pvDutyMax = clampFloat(value, 0.02f, 0.80f);
    Serial.print("pv_dmax=");
    Serial.println(pvDutyMax, 4);
  }
  else if (parseFloatAfterPrefix(cmd, "pv_dstart ", value)) {
    pvDutyStart = clampFloat(value, 0.0f, pvDutyMax);
    Serial.print("pv_dstart=");
    Serial.println(pvDutyStart, 4);
  }
  else if (parseFloatAfterPrefix(cmd, "pv_slew ", value)) {
    pvDutySlewPerStep = clampFloat(value, 0.0005f, 0.10f);
    Serial.print("pv_slew=");
    Serial.println(pvDutySlewPerStep, 4);
  }
  else if (parseFloatAfterPrefix(cmd, "pv_ssms ", value)) {
    pvSoftstartMs = (uint32_t)clampFloat(value, 0.0f, 20000.0f);
    Serial.print("pv_ssms=");
    Serial.println(pvSoftstartMs);
  }
  else if (parseFloatAfterPrefix(cmd, "pv_ctrlms ", value)) {
    pvControlPeriodMs = (uint32_t)clampFloat(value, 10.0f, 1000.0f);
    Serial.print("pv_ctrlms=");
    Serial.println(pvControlPeriodMs);
  }
  else if (parseFloatAfterPrefix(cmd, "pv_vpvref ", value)) {
    pvVpvRef = clampFloat(value, pvVpvRefMin, pvVpvRefMax);
    Serial.print("pv_vpvref=");
    Serial.println(pvVpvRef, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "pv_vpvmin ", value)) {
    pvVpvRefMin = clampFloat(value, 0.0f, pvVpvRefMax);
    pvVpvRef = clampFloat(pvVpvRef, pvVpvRefMin, pvVpvRefMax);
    Serial.print("pv_vpvmin=");
    Serial.println(pvVpvRefMin, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "pv_vpvmax ", value)) {
    pvVpvRefMax = fmaxf(value, pvVpvRefMin);
    pvVpvRef = clampFloat(pvVpvRef, pvVpvRefMin, pvVpvRefMax);
    Serial.print("pv_vpvmax=");
    Serial.println(pvVpvRefMax, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "pv_kp ", value)) {
    pvKpVpv = clampFloat(value, 0.0f, 5.0f);
    Serial.print("pv_kp=");
    Serial.println(pvKpVpv, 4);
  }
  else if (parseFloatAfterPrefix(cmd, "pv_ki ", value)) {
    pvKiVpv = clampFloat(value, 0.0f, 20.0f);
    Serial.print("pv_ki=");
    Serial.println(pvKiVpv, 4);
  }
  else if (parseFloatAfterPrefix(cmd, "pv_imax ", value)) {
    pvInputCurrentMax = clampFloat(value, 0.05f, 20.0f);
    Serial.print("pv_imax=");
    Serial.println(pvInputCurrentMax, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "pv_ibatmax ", value) ||
           parseFloatAfterPrefix(cmd, "ibatmax ", value)) {
    pvBatteryChargeCurrentMax = clampFloat(value, 0.05f, 20.0f);
    Serial.print("pv_ibatmax=");
    Serial.println(pvBatteryChargeCurrentMax, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "pv_pmax ", value)) {
    pvInputPowerMax = clampFloat(value, 0.5f, 100.0f);
    Serial.print("pv_pmax=");
    Serial.println(pvInputPowerMax, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "pv_panelmin ", value)) {
    pvPanelTripLow = clampFloat(value, 0.0f, 10.0f);
    Serial.print("pv_panelmin=");
    Serial.println(pvPanelTripLow, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "pv_batwarn ", value)) {
    pvBatWarnHigh = clampFloat(value, 0.0f, pvBatTripHigh);
    Serial.print("pv_batwarn=");
    Serial.println(pvBatWarnHigh, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "pv_battrip ", value)) {
    pvBatTripHigh = clampFloat(value, 0.0f, 12.0f);
    pvBatWarnHigh = fminf(pvBatWarnHigh, pvBatTripHigh);
    Serial.print("pv_battrip=");
    Serial.println(pvBatTripHigh, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "mppt_step ", value)) {
    mpptStepV = clampFloat(value, 0.001f, 0.25f);
    Serial.print("mppt_step=");
    Serial.println(mpptStepV, 4);
  }
  else if (parseFloatAfterPrefix(cmd, "mppt_ms ", value)) {
    mpptPeriodMs = (uint32_t)clampFloat(value, 100.0f, 60000.0f);
    Serial.print("mppt_ms=");
    Serial.println(mpptPeriodMs);
  }
  else if (parseFloatAfterPrefix(cmd, "mppt_epsp ", value)) {
    mpptEpsPowerW = clampFloat(value, 0.0f, 5.0f);
    Serial.print("mppt_epsp=");
    Serial.println(mpptEpsPowerW, 3);
  }
  else if (parseFloatAfterPrefix(cmd, "mppt_pmin ", value)) {
    mpptMinPowerW = clampFloat(value, 0.0f, pvInputPowerMax);
    Serial.print("mppt_pmin=");
    Serial.println(mpptMinPowerW, 3);
  }
  else if (cmd.startsWith("adsaddr main ")) {
    stopAllPower("cambio direccion ADS main");
    adsMainAddr = parseI2cAddress(cmd.substring(String("adsaddr main ").length()), adsMainAddr);
    sanitizeConfig();
    configureFastControlChannel();
    Serial.print("ADS1/main=");
    printHexAddr(adsMainAddr);
    Serial.println();
  }
  else if (cmd.startsWith("adsaddr mppt ")) {
    stopAllPower("cambio direccion ADS mppt");
    adsMpptAddr = parseI2cAddress(cmd.substring(String("adsaddr mppt ").length()), adsMpptAddr);
    sanitizeConfig();
    configureFastControlChannel();
    Serial.print("ADS2/mppt=");
    printHexAddr(adsMpptAddr);
    Serial.println();
  }
  else if (cmd == "adsswap") {
    stopAllPower("swap ADS");
    uint8_t tmp = adsMainAddr;
    adsMainAddr = adsMpptAddr;
    adsMpptAddr = tmp;
    configureFastControlChannel();
    Serial.print("ADS swap: main=");
    printHexAddr(adsMainAddr);
    Serial.print(" mppt=");
    printHexAddr(adsMpptAddr);
    Serial.println();
  }
  else if (cmd.startsWith("boost_pv2strip ")) {
    boostPv2sTripHigh = clampFloat(parseValueAfterSpace(cmd), 3.0f, 12.0f);
    Serial.print("BOOST PV2S trip high=");
    Serial.println(boostPv2sTripHigh, 3);
  }
  else if (cmd.startsWith("boost_pv2swarn ")) {
    boostPv2sWarnHigh = clampFloat(parseValueAfterSpace(cmd), 3.0f, 12.0f);
    Serial.print("BOOST PV2S warn high=");
    Serial.println(boostPv2sWarnHigh, 3);
  }
  else if (cmd.startsWith("boost_bus5min ")) {
    boostBus5TripLow = clampFloat(parseValueAfterSpace(cmd), 0.0f, 6.0f);
    Serial.print("BOOST BUS5 min=");
    Serial.println(boostBus5TripLow, 3);
  }
  else if (cmd.startsWith("boost_bus5warn ")) {
    boostBus5WarnLow = clampFloat(parseValueAfterSpace(cmd), 0.0f, 6.0f);
    Serial.print("BOOST BUS5 warn=");
    Serial.println(boostBus5WarnLow, 3);
  }
  else if (cmd.startsWith("buck_pv2smin ")) {
    buckPv2sTripLow = clampFloat(parseValueAfterSpace(cmd), 0.0f, 12.0f);
    Serial.print("BUCK PV2S min=");
    Serial.println(buckPv2sTripLow, 3);
  }
  else if (cmd.startsWith("buck_pv2strip ")) {
    buckPv2sTripHigh = clampFloat(parseValueAfterSpace(cmd), 0.0f, 12.0f);
    Serial.print("BUCK PV2S trip=");
    Serial.println(buckPv2sTripHigh, 3);
  }
  else if (cmd.startsWith("buck_bus5trip ")) {
    buckBus5TripHigh = clampFloat(parseValueAfterSpace(cmd), 0.0f, 8.0f);
    Serial.print("BUCK BUS5 trip=");
    Serial.println(buckBus5TripHigh, 3);
  }
  else if (cmd.startsWith("buck_bus5warn ")) {
    buckBus5WarnHigh = clampFloat(parseValueAfterSpace(cmd), 0.0f, 8.0f);
    Serial.print("BUCK BUS5 warn=");
    Serial.println(buckBus5WarnHigh, 3);
  }
  else {
    Serial.println("Comando no reconocido. Usa help.");
  }
}

void processCommandList(String line) {
  line.trim();
  int start = 0;
  while (start < line.length()) {
    int sep = line.indexOf(';', start);
    String cmd = (sep < 0) ? line.substring(start) : line.substring(start, sep);
    handleSerialCommand(cmd);
    if (sep < 0) break;
    start = sep + 1;
  }
}

bool prearmCheck(bool verbose) {
  if (!buckboostEnabledByMode()) {
    stopBuckboost("prearm rechazado");
    if (verbose) Serial.println("BUCKBOOST PREARM rechazado: modo PV_ONLY activo.");
    return false;
  }
  stopBuckboost("prearm");
  bool ok = updateTelemetryMeasurements();
  if (verbose) {
    Serial.println();
    Serial.println("=== BUCKBOOST PREARM ===");
    Serial.print("ADS/read: ");
    Serial.println(ok ? "PASS" : "FAIL");
  }
  if (ok) ok = checkProtections(verbose) && ok;
  if (faultLatched) {
    ok = false;
    if (verbose) Serial.println("Fault latched: FAIL. Usa clear_fault.");
  }
  if (verbose) {
    printStatus();
    Serial.println(ok ? "PREARM: PASS" : "PREARM: FAIL - no partir.");
  }
  return ok;
}

// ============================================================
// Setup / loop
// ============================================================
void setup() {
  pinMode(PIN_BB_PWM_HIGH, OUTPUT);
  digitalWrite(PIN_BB_PWM_HIGH, HIGH);
  pinMode(PIN_BB_PWM_LOW, OUTPUT);
  digitalWrite(PIN_BB_PWM_LOW, LOW);
  pinMode(PIN_PVBOOST_PWM, OUTPUT);
  digitalWrite(PIN_PVBOOST_PWM, LOW);
  pinMode(PIN_PVBOOST_GND, OUTPUT);
  digitalWrite(PIN_PVBOOST_GND, LOW);
  if (PIN_PVBOOST_EN >= 0) {
    pinMode(PIN_PVBOOST_EN, OUTPUT);
    digitalWrite(PIN_PVBOOST_EN, LOW);
  }

  Serial.begin(115200);
  Serial.setTimeout(80);
  delay(800);

  bbAllOff();
  pvEnableWrite(false);
  digitalWrite(PIN_PVBOOST_PWM, LOW);

  restoreFirmwareDefaults();
  bool cfgLoaded = loadConfig(false);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  scanI2CAtBoot();

  configureHardwarePwm();
  configurePvBoostPwm();
  configureFastControlChannel();
  bbAllOff();
  pvOutputsSafeOff();

  Serial.println();
  Serial.println("Agente 3 integrado buckboost + boost PV MPPT listo");
  Serial.println("Arranque seguro: buckboost OFF, pvboost OFF");
  Serial.println(cfgLoaded ? "Config: cargada desde NVS" : "Config: defaults de firmware");
  Serial.print("ADS1/main=");
  printHexAddr(adsMainAddr);
  Serial.print(" | ADS2/mppt=");
  printHexAddr(adsMpptAddr);
  Serial.println();
  Serial.print("PV boost PWM asumido GPIO");
  Serial.print(PIN_PVBOOST_PWM);
  Serial.print(" | EN GPIO");
  Serial.println(PIN_PVBOOST_EN);
  Serial.print("TestMode=");
  Serial.println(operationModeName());

  updateTelemetryMeasurements();
  lastControlUs = micros();
  lastFullReadMs = millis();
  lastPrintMs = millis();
  lastPvControlMs = millis();
  printStatus();
  printHelp();
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommandList(cmd);
  }

  unsigned long nowUs = micros();
  unsigned long nowMs = millis();

  if (buckboostEnabledByMode() && nowUs - lastControlUs >= controlPeriodUs) {
    lastControlUs += controlPeriodUs;
    bool okFast = updateFastMeasurement();

    if (okFast) {
      if (runMode != RUN_OFF) checkProtections(false);
      if (runMode == RUN_AUTO_FFPI && !faultLatched) updateFFPI();
    } else {
      Serial.println("Lectura ADS rapida fallida");
    }
  } else if (!buckboostEnabledByMode()) {
    lastControlUs = nowUs;
    bbAllOff();
  }

  if (nowMs - lastPvControlMs >= pvControlPeriodMs) {
    lastPvControlMs = nowMs;
    updatePvBoostControl();
  }

  if (nowMs - lastFullReadMs >= fullReadPeriodMs) {
    lastFullReadMs = nowMs;
    updateTelemetryMeasurements();
  }

  refreshBuckboostOutput();

  if (autoLog && nowMs - lastPrintMs >= printPeriodMs) {
    lastPrintMs = nowMs;

    Serial.print("BB=");
    Serial.print(topologyName());
    Serial.print(" run=");
    Serial.print(runName());
    Serial.print(" PV2S=");
    Serial.print(v_pv2s, 3);
    Serial.print("V BUS5=");
    Serial.print(v_bus5, 3);
    Serial.print("V tgt=");
    Serial.print(targetV, 2);
    Serial.print("V Ton=");
    Serial.print(tonCmdUs, 3);
    Serial.print("us");

    Serial.print(" | PV=");
    Serial.print(pvModeName());
    Serial.print(" arm=");
    Serial.print(pvArmed ? 1 : 0);
    Serial.print(" VPV=");
    Serial.print(meas.vPanel, 3);
    Serial.print("V IPV=");
    Serial.print(meas.iPanel, 3);
    Serial.print("A VBAT=");
    Serial.print(meas.vBat2s, 3);
    Serial.print("V duty=");
    Serial.print(pvDutyApplied, 4);
    Serial.print(" P=");
    Serial.print(meas.pPanel, 3);
    Serial.print("W");

    Serial.print(" | Ibus5=");
    Serial.print(meas.iBbBus5, 3);
    Serial.print("A Ibat=");
    Serial.print(meas.iBat, 3);
    Serial.print("A Ibb2S_kcl=");
    Serial.print(meas.iBbIn2sKcl, 3);
    Serial.print("A");

    if (faultLatched) Serial.print(" | BB_FAULT");
    if (pvFaultLatched) Serial.print(" | PV_FAULT");
    if (topology == TOPO_BOOST && v_bus5 < boostBus5WarnLow) Serial.print(" | WARN BUS5 LOW");
    if (topology == TOPO_BOOST && v_pv2s > boostPv2sWarnHigh) Serial.print(" | WARN PV2S HIGH");
    if (topology == TOPO_BUCK && v_bus5 > buckBus5WarnHigh) Serial.print(" | WARN BUS5 HIGH");
    if (pvMode != PV_OFF && meas.vBat2s > pvBatWarnHigh) Serial.print(" | WARN PV BAT HIGH");
    Serial.println();
  }
}
