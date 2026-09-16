/*
 * main.cpp
 * ─────────────────────────────────────────────────────────────
 *  MICRORRED DC — AGENTE 2  (Control de Corriente, nodo CAN)
 *
 *  Idéntico al agente 1 en control (mismo voltaje y corriente máxima),
 *  pero SIN WiFi: se comunica únicamente por CAN con el agente 1.
 *    - Envía su telemetría (i_out, v_bat, v_bus, SoC) y faults por CAN.
 *    - Recibe sus parámetros (config) por CAN desde el agente 1, que
 *      a su vez los toma de /agente2/params en Firebase (lo escribe la web).
 *
 *  Esquema de control (igual que el agente 1):
 *    Lazo externo → PI voltaje (targetV - v_bus) → i_ref_v
 *    Droop        → kd * (SoC - 0.5)             → i_ref_d
 *    i_ref        = i_ref_v + i_ref_d
 *    Lazo interno → PI corriente (i_ref - i_out) → duty
 *    Topología    → automática (con histéresis ±100 mA)
 *
 *  Sensores ADS1115:
 *    A2 → divisor voltaje bus
 *    A1 → divisor voltaje batería
 *    A3 → ACS712 (corriente de salida)
 *
 *  Cores:
 *    Core 1 → taskControl : lectura sensores + lazos PI + PWM
 *    Core 0 → taskCAN     : nodo CAN (MCP2515) ↔ agente 1
 * ─────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include "shared.h"
#include "control.h"
#include "can_bus.h"

void setup() {
  pwmForceSafe();                 // ANTES que nada: medio puente OFF (evita gates flotantes)
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Agente 2 — DROOP ESCLAVO (usa i_ref_v del maestro + droop) ===");

  xMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(taskControl, "Control", 8192, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(taskCAN,     "CAN",     6144, nullptr, 3, nullptr, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
