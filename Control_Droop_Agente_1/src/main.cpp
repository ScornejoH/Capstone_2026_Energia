/*
 * main.cpp
 * ─────────────────────────────────────────────────────────────
 *  MICRORRED DC — AGENTE 1  (Control de Corriente)
 *
 *  Esquema de control:
 *    Lazo externo → PI voltaje (5V - v_bus) → i_ref_v
 *    Droop        → kd * (SoC - 0.5)        → i_ref_d
 *    i_ref        = i_ref_v + i_ref_d
 *    Lazo interno → PI corriente (i_ref - i_out) → duty
 *    Topología    → automática según signo de i_ref
 *                   i_ref > 0 : BOOST (batería → bus)
 *                   i_ref < 0 : BUCK  (bus → batería)
 *
 *  Sensores ADS1115:
 *    A0 → divisor voltaje bus
 *    A1 → divisor voltaje batería
 *    A3 → ACS712 5A (corriente de salida)
 *
 *  Cores:
 *    Core 1 → taskControl : lectura sensores + lazos PI + PWM
 *    Core 0 → taskComms   : WiFi + Firebase REST
 *    Core 0 → taskCAN     : gateway CAN (MCP2515) ↔ agentes 2 y 3
 *
 *  Módulos:
 *    shared.h/cpp        → variables compartidas, mutex, utilidades
 *    sensors.h/cpp       → ADS1115 + ACS712 + estimación SoC por LUT
 *    control.h/cpp       → lazos PI + PWM LEDC + protecciones
 *    wifi_firebase.h/cpp → WiFi + Firebase Auth + REST
 *    can_bus.h/cpp       → MCP2515 (SPI) + protocolo CAN + relay
 * ─────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include "shared.h"
#include "control.h"
#include "wifi_firebase.h"
#include "can_bus.h"

void setup() {
  pwmForceSafe();                 // ANTES que nada: medio puente OFF (evita gates flotantes)
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Agente 1 — DROOP MAESTRO (PI voltaje + difunde i_ref_v) ===");

  xMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(taskControl, "Control", 8192,  nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(taskComms,   "Comms",   28672, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(taskCAN,     "CAN",     6144,  nullptr, 3, nullptr, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
