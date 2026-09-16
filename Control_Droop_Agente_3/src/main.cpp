/*
 * main.cpp — AGENTE 3
 * ─────────────────────────────────────────────────────────────
 *  MICRORRED DC — AGENTE 3 (buckboost bidireccional + boost PV/MPPT)
 *
 *  Nodo CAN ESCLAVO droop (sin WiFi): el buckboost usa el MISMO lazo de
 *  corriente que los agentes 1/2 y comparte el bus siguiendo i_ref_v del
 *  maestro (respaldo autónomo Opción 3). El subsistema PV/MPPT es
 *  independiente. Toda la configuración llega por CAN (protocolo extendido)
 *  desde la web vía el gateway (agente 1) y se persiste en NVS.
 *
 *  Cores:
 *    Core 1 → taskControl : sensores + buckboost (PI cascada) + PV/MPPT + PWM
 *    Core 0 → taskCAN     : nodo CAN (MCP2515) ↔ agente 1
 * ─────────────────────────────────────────────────────────────
 */
#include <Arduino.h>
#include "shared.h"
#include "control.h"
#include "can_bus.h"

void setup() {
  /* Estado seguro de TODAS las salidas de potencia ANTES de cualquier cosa
     (buckboost OFF + boost PV OFF + pull internos seguros de respaldo). */
  pwmForceSafe();

  Serial.begin(115200);
  delay(500);
  Serial.println("=== Agente 3 — buckboost (esclavo droop) + PV/MPPT ===");

  xMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(taskControl, "Control", 8192, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(taskCAN,     "CAN",     6144, nullptr, 3, nullptr, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
