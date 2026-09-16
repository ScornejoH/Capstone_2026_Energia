/*
 * NODO TX - simulacion del agente 2 -> CAN
 *
 * Envia una RONDA COMPLETA cada 1 segundo:
 *   ID 0x200 -> v_bat (float, 4 bytes)
 *   ID 0x201 -> v_bus (float, 4 bytes)
 *   ID 0x202 -> i_out (float, 4 bytes)
 *
 * Las 3 tramas salen seguidas (sin delay entre ellas) para que el RX
 * las reciba como un set y haga un solo PATCH a Firebase. Luego se
 * espera lo que falte para completar 1 segundo desde el inicio de la
 * ronda.
 *
 * Bitrate: 500 kbps @ cristal 8 MHz.
 * Pines VSPI estandar del ESP32.
 */

 #include <Arduino.h>
 #include <SPI.h>
 #include <string.h>
 #include <math.h>
 
 #define PIN_SCK    18
 #define PIN_MISO   19
 #define PIN_MOSI   23
 #define PIN_CS      5
 
 #define SPI_HZ  8000000
 
 // Periodo de la ronda completa (vbat + vbus + iout)
 #define ROUND_PERIOD_MS  1000
 
 // IDs CAN para cada variable del agente 2
 #define CAN_ID_VBAT  0x200
 #define CAN_ID_VBUS  0x201
 #define CAN_ID_IOUT  0x202
 
 // -------- Instrucciones SPI --------
 #define INSTR_RESET   0xC0
 #define INSTR_READ    0x03
 #define INSTR_WRITE   0x02
 #define INSTR_BITMOD  0x05
 #define INSTR_RTS_TX0 0x81
 
 // -------- Registros --------
 #define REG_CANSTAT   0x0E
 #define REG_CANCTRL   0x0F
 #define REG_TEC       0x1C
 #define REG_CNF3      0x28
 #define REG_CNF2      0x29
 #define REG_CNF1      0x2A
 #define REG_CANINTF   0x2C
 #define REG_EFLG      0x2D
 #define REG_RXB0CTRL  0x60
 #define REG_RXB1CTRL  0x70
 #define REG_TXB0CTRL  0x30
 #define REG_TXB0SIDH  0x31
 #define REG_TXB0SIDL  0x32
 #define REG_TXB0DLC   0x35
 #define REG_TXB0D0    0x36
 
 // -------- Modos --------
 #define MODE_NORMAL   0x00
 #define MODE_CONFIG   0x80
 #define MODE_MASK     0xE0
 
 // -------- struct compatible con autowp --------
 struct can_frame {
   uint32_t can_id;
   uint8_t  can_dlc;
   uint8_t  data[8];
 };
 
 SPIClass spi(VSPI);
 SPISettings spiCfg(SPI_HZ, MSBFIRST, SPI_MODE0);
 
 // -------- Parametros del agente 2 (mismos del codigo Firebase original) --------
 const float VBAT_CENTER = 3.80f;
 const float VBAT_AMP    = 0.12f;
 const float VBUS_CENTER = 5.00f;
 const float VBUS_NOISE  = 0.05f;
 const float IOUT_AMP    = 1.50f;
 const float IOUT_OFFSET = 0.00f;
 const float SIM_FREQ    = 0.04f;
 const float SIM_PHASE   = 2.09f;
 
 float t_sim = 0.0f;
 
 // ============================================================
 // Driver manual MCP2515
 // ============================================================
 void csLow()  { digitalWrite(PIN_CS, LOW); }
 void csHigh() { digitalWrite(PIN_CS, HIGH); }
 
 void mcpReset() {
   spi.beginTransaction(spiCfg);
   csLow();  spi.transfer(INSTR_RESET);  csHigh();
   spi.endTransaction();
   delay(20);
 }
 
 uint8_t mcpRead(uint8_t reg) {
   spi.beginTransaction(spiCfg);
   csLow();
   spi.transfer(INSTR_READ);
   spi.transfer(reg);
   uint8_t v = spi.transfer(0x00);
   csHigh();
   spi.endTransaction();
   return v;
 }
 
 void mcpWrite(uint8_t reg, uint8_t val) {
   spi.beginTransaction(spiCfg);
   csLow();
   spi.transfer(INSTR_WRITE);
   spi.transfer(reg);
   spi.transfer(val);
   csHigh();
   spi.endTransaction();
 }
 
 void mcpBitMod(uint8_t reg, uint8_t mask, uint8_t val) {
   spi.beginTransaction(spiCfg);
   csLow();
   spi.transfer(INSTR_BITMOD);
   spi.transfer(reg);
   spi.transfer(mask);
   spi.transfer(val);
   csHigh();
   spi.endTransaction();
 }
 
 void mcpRTS_TX0() {
   spi.beginTransaction(spiCfg);
   csLow();  spi.transfer(INSTR_RTS_TX0);  csHigh();
   spi.endTransaction();
 }
 
 bool setMode(uint8_t mode, const char* nombre) {
   mcpBitMod(REG_CANCTRL, MODE_MASK, mode);
   for (int i = 0; i < 50; i++) {
     uint8_t s = mcpRead(REG_CANSTAT);
     if ((s & MODE_MASK) == mode) {
       Serial.print("[OK] modo ");  Serial.println(nombre);
       return true;
     }
     delay(2);
   }
   Serial.print("[FAIL] modo ");  Serial.println(nombre);
   return false;
 }
 
 bool setBitrate500k_8MHz() {
   mcpWrite(REG_CNF1, 0x00);
   mcpWrite(REG_CNF2, 0x90);
   mcpWrite(REG_CNF3, 0x02);
   if (mcpRead(REG_CNF1) != 0x00) { Serial.println("[FAIL] CNF1"); return false; }
   if (mcpRead(REG_CNF2) != 0x90) { Serial.println("[FAIL] CNF2"); return false; }
   if (mcpRead(REG_CNF3) != 0x02) { Serial.println("[FAIL] CNF3"); return false; }
   Serial.println("[OK] bitrate 500kbps@8MHz");
   return true;
 }
 
 bool sendMessage(const can_frame *f) {
   uint8_t txctrl = mcpRead(REG_TXB0CTRL);
   if (txctrl & 0x08) {
     mcpBitMod(REG_TXB0CTRL, 0x08, 0x00);
     delay(2);
   }
 
   uint16_t sid = f->can_id & 0x7FF;
   mcpWrite(REG_TXB0SIDH, (sid >> 3) & 0xFF);
   mcpWrite(REG_TXB0SIDL, (sid & 0x07) << 5);
 
   uint8_t dlc = f->can_dlc;
   if (dlc > 8) dlc = 8;
   mcpWrite(REG_TXB0DLC, dlc);
 
   for (uint8_t i = 0; i < dlc; i++) {
     mcpWrite(REG_TXB0D0 + i, f->data[i]);
   }
 
   mcpBitMod(REG_CANINTF, 0x04, 0x00);
   mcpRTS_TX0();
 
   for (int i = 0; i < 100; i++) {
     uint8_t intf = mcpRead(REG_CANINTF);
     if (intf & 0x04) {
       mcpBitMod(REG_CANINTF, 0x04, 0x00);
       return true;
     }
     delay(1);
   }
 
   mcpBitMod(REG_TXB0CTRL, 0x08, 0x00);
   delay(5);
   return false;
 }
 
 // ============================================================
 // Simulacion
 // ============================================================
 static float ruido(float amp) {
   return amp * (2.0f * (float)random(1000) / 1000.0f - 1.0f);
 }
 
 void generar_datos_agente2(float t, float* vbat, float* vbus, float* iout) {
   *iout = IOUT_AMP * sinf(2.0f * M_PI * SIM_FREQ * t + SIM_PHASE)
           + IOUT_OFFSET
           + ruido(0.05f);
 
   float bat_drift = -(*iout) * 0.0002f;
   *vbat = VBAT_CENTER
           + VBAT_AMP * sinf(2.0f * M_PI * SIM_FREQ * 0.1f * t + SIM_PHASE)
           + bat_drift
           + ruido(0.005f);
   *vbat = constrain(*vbat, 3.00f, 4.20f);
 
   *vbus = VBUS_CENTER
           + 0.05f * sinf(2.0f * M_PI * 0.5f * t)
           + ruido(VBUS_NOISE);
   *vbus = constrain(*vbus, 4.50f, 5.50f);
 }
 
 // ============================================================
 // Helper: enviar un float
 // ============================================================
 bool enviarFloat(uint16_t id, float v) {
   can_frame f;
   f.can_id  = id;
   f.can_dlc = 4;
   memcpy(f.data, &v, 4);
   return sendMessage(&f);
 }
 
 // ============================================================
 // SETUP / LOOP
 // ============================================================
 uint32_t rondas = 0;
 
 void setup() {
   Serial.begin(115200);
   delay(500);
   Serial.println();
   Serial.println("=== NODO TX agente2 -> CAN (1 ronda/s) ===");
 
   randomSeed(esp_random());
 
   pinMode(PIN_CS, OUTPUT);
   csHigh();
   spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
 
   mcpReset();
   uint8_t s = mcpRead(REG_CANSTAT);
   Serial.printf("[1] tras RESET CANSTAT=0x%02X\n", s);
   if ((s & MODE_MASK) != MODE_CONFIG) {
     Serial.println("ABORT: no entro en config");
     while (1) delay(1000);
   }
 
   if (!setBitrate500k_8MHz()) { while (1) delay(1000); }
 
   mcpWrite(REG_RXB0CTRL, 0x60);
   mcpWrite(REG_RXB1CTRL, 0x60);
   Serial.println("[OK] RX abiertos");
 
   if (!setMode(MODE_NORMAL, "NORMAL")) { while (1) delay(1000); }
 
   Serial.println();
   Serial.println("Iniciando transmision agente2...");
   Serial.println();
 }
 
 void loop() {
   uint32_t t_inicio = millis();
 
   // Generar valores de la ronda
   float vbat, vbus, iout;
   generar_datos_agente2(t_sim, &vbat, &vbus, &iout);
 
   rondas++;
 
   // Enviar las 3 tramas seguidas (sin delay entre ellas)
   bool ok_vbat = enviarFloat(CAN_ID_VBAT, vbat);
   bool ok_vbus = enviarFloat(CAN_ID_VBUS, vbus);
   bool ok_iout = enviarFloat(CAN_ID_IOUT, iout);
 
   bool todo_ok = ok_vbat && ok_vbus && ok_iout;
 
   Serial.printf("Ronda #%lu  vbat=%.3f vbus=%.3f iout=%.3f  [%s]\n",
                 rondas, vbat, vbus, iout,
                 todo_ok ? "OK" : "FAIL");
 
   if (!todo_ok) {
     uint8_t eflg = mcpRead(REG_EFLG);
     uint8_t tec  = mcpRead(REG_TEC);
     Serial.printf("  vbat=%d vbus=%d iout=%d  EFLG=0x%02X TEC=%d\n",
                   ok_vbat, ok_vbus, ok_iout, eflg, tec);
   }
 
   t_sim += ROUND_PERIOD_MS / 1000.0f;
 
   // Esperar lo que falte para completar el periodo de la ronda
   uint32_t transcurrido = millis() - t_inicio;
   if (transcurrido < ROUND_PERIOD_MS) {
     delay(ROUND_PERIOD_MS - transcurrido);
   }
 }