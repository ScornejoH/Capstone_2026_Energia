/*
 * can_bus.cpp
 * ─────────────────────────────────────────────────────────────
 * Driver manual MCP2515 vía SPI y simulación de agente 2.
 * canInit() devuelve false si el módulo no responde;
 * el llamador (taskComms) actualiza hwStatus según corresponda.
 * ─────────────────────────────────────────────────────────────
 */

#include "can_bus.h"
#include "shared.h"
#include <math.h>

/* ============================================================
   PINES SPI Y OBJETOS
============================================================ */
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define PIN_CS     5
#define SPI_HZ    8000000

static SPIClass    spi(VSPI);
static SPISettings spiCfg(SPI_HZ, MSBFIRST, SPI_MODE0);

/* ============================================================
   INSTRUCCIONES Y REGISTROS MCP2515
============================================================ */
#define INSTR_RESET    0xC0
#define INSTR_READ     0x03
#define INSTR_WRITE    0x02
#define INSTR_BITMOD   0x05
#define INSTR_RTS_TX0  0x81

#define REG_CANSTAT    0x0E
#define REG_CANCTRL    0x0F
#define REG_CNF3       0x28
#define REG_CNF2       0x29
#define REG_CNF1       0x2A
#define REG_CANINTF    0x2C
#define REG_RXB0CTRL   0x60
#define REG_RXB1CTRL   0x70
#define REG_TXB0CTRL   0x30
#define REG_TXB0SIDH   0x31
#define REG_TXB0SIDL   0x32
#define REG_TXB0DLC    0x35
#define REG_TXB0D0     0x36

#define MODE_NORMAL    0x00
#define MODE_CONFIG    0x80
#define MODE_MASK      0xE0

/* ============================================================
   PARÁMETROS DE SIMULACIÓN AGENTE 2
============================================================ */
static const float VBAT_CENTER  = 3.80f;
static const float VBAT_AMP     = 0.12f;
static const float VBUS_CENTER  = 5.00f;
static const float VBUS_NOISE   = 0.05f;
static const float IOUT_AMP     = 1.50f;
static const float IOUT_OFFSET  = 0.00f;
static const float SIM_FREQ     = 0.04f;
static const float SIM_PHASE    = 2.09f;

/* ============================================================
   PRIMITIVAS SPI
============================================================ */
static inline void csLow()  { digitalWrite(PIN_CS, LOW); }
static inline void csHigh() { digitalWrite(PIN_CS, HIGH); }

static void mcpReset() {
  spi.beginTransaction(spiCfg);
  csLow();  spi.transfer(INSTR_RESET);  csHigh();
  spi.endTransaction();
  delay(20);
}

static uint8_t mcpRead(uint8_t reg) {
  spi.beginTransaction(spiCfg);
  csLow();
  spi.transfer(INSTR_READ);  spi.transfer(reg);
  uint8_t v = spi.transfer(0x00);
  csHigh();
  spi.endTransaction();
  return v;
}

static void mcpWrite(uint8_t reg, uint8_t val) {
  spi.beginTransaction(spiCfg);
  csLow();
  spi.transfer(INSTR_WRITE);  spi.transfer(reg);  spi.transfer(val);
  csHigh();
  spi.endTransaction();
}

static void mcpBitMod(uint8_t reg, uint8_t mask, uint8_t val) {
  spi.beginTransaction(spiCfg);
  csLow();
  spi.transfer(INSTR_BITMOD);  spi.transfer(reg);
  spi.transfer(mask);          spi.transfer(val);
  csHigh();
  spi.endTransaction();
}

static void mcpRTS() {
  spi.beginTransaction(spiCfg);
  csLow();  spi.transfer(INSTR_RTS_TX0);  csHigh();
  spi.endTransaction();
}

/* ============================================================
   INICIALIZACIÓN
   Devuelve true solo si el MCP2515 entra en modo NORMAL.
   Si no hay módulo conectado, CANSTAT no estará en CONFIG
   tras el reset y se retorna false inmediatamente.
============================================================ */
bool canInit() {
  pinMode(PIN_CS, OUTPUT);
  csHigh();
  spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  mcpReset();

  // Verificar que el MCP2515 respondió entrando en modo CONFIG
  uint8_t stat = mcpRead(REG_CANSTAT);
  if ((stat & MODE_MASK) != MODE_CONFIG) {
    Serial.printf("[CAN] No detectado (CANSTAT=0x%02X)\n", stat);
    return false;
  }

  /* Bitrate 500 kbps @ cristal 8 MHz */
  mcpWrite(REG_CNF1, 0x00);
  mcpWrite(REG_CNF2, 0x90);
  mcpWrite(REG_CNF3, 0x02);

  /* Filtros RX en modo promiscuo */
  mcpWrite(REG_RXB0CTRL, 0x60);
  mcpWrite(REG_RXB1CTRL, 0x60);

  /* Cambiar a modo normal */
  mcpBitMod(REG_CANCTRL, MODE_MASK, MODE_NORMAL);
  for (int i = 0; i < 50; i++) {
    if ((mcpRead(REG_CANSTAT) & MODE_MASK) == MODE_NORMAL) return true;
    delay(2);
  }
  Serial.println("[CAN] No entró en modo NORMAL");
  return false;
}

/* ============================================================
   TRANSMISIÓN
   Devuelve false si el TX no se confirma (bus ocupado,
   módulo desconectado, etc.). El llamador decide si reintentar.
============================================================ */
bool canSendFloat(uint16_t id, float val) {
  if (mcpRead(REG_TXB0CTRL) & 0x08) {
    mcpBitMod(REG_TXB0CTRL, 0x08, 0x00);
    delay(2);
  }

  mcpWrite(REG_TXB0SIDH, (id >> 3) & 0xFF);
  mcpWrite(REG_TXB0SIDL, (id & 0x07) << 5);
  mcpWrite(REG_TXB0DLC, 4);

  uint8_t buf[4];
  memcpy(buf, &val, 4);
  for (int i = 0; i < 4; i++) mcpWrite(REG_TXB0D0 + i, buf[i]);

  mcpBitMod(REG_CANINTF, 0x04, 0x00);
  mcpRTS();

  for (int i = 0; i < 100; i++) {
    if (mcpRead(REG_CANINTF) & 0x04) {
      mcpBitMod(REG_CANINTF, 0x04, 0x00);
      return true;
    }
    delay(1);
  }

  mcpBitMod(REG_TXB0CTRL, 0x08, 0x00);
  delay(5);
  return false;
}

/* ============================================================
   SIMULACIÓN AGENTE 2
============================================================ */
static float ruido(float amp) {
  return amp * (2.0f * (float)random(1000) / 1000.0f - 1.0f);
}

void simAgente2(float t, float* vbat, float* vbus, float* iout) {
  *iout = IOUT_AMP * sinf(2.0f * M_PI * SIM_FREQ * t + SIM_PHASE)
        + IOUT_OFFSET + ruido(0.05f);

  float bat_drift = -(*iout) * 0.0002f;
  *vbat = clampF(
    VBAT_CENTER
    + VBAT_AMP * sinf(2.0f * M_PI * SIM_FREQ * 0.1f * t + SIM_PHASE)
    + bat_drift + ruido(0.005f),
    3.00f, 4.20f
  );

  *vbus = clampF(
    VBUS_CENTER + 0.05f * sinf(2.0f * M_PI * 0.5f * t) + ruido(VBUS_NOISE),
    4.50f, 5.50f
  );
}
