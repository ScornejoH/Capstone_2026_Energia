/*
 * main.cpp
 * ─────────────────────────────────────────────────────────────
 *  MICRORRED DC — NODO AGENTE 2
 *  Buck-Boost FF+PI  +  CAN  +  Firebase Realtime Database
 * ─────────────────────────────────────────────────────────────
 *
 *  Core 1 → taskControl : lazo PWM (ADS1115 + FF+PI + LEDC)
 *  Core 0 → taskComms   : WiFi · Firebase REST · CAN MCP2515
 *
 *  Este archivo solo contiene setup() y loop().
 *  Toda la lógica está en los módulos:
 *    shared.h/cpp        → variables compartidas, mutex, utilidades
 *    control.h/cpp       → control PWM + ADS1115
 *    can_bus.h/cpp       → driver MCP2515 + simulación agente 2
 *    wifi_firebase.h/cpp → WiFi + Firebase Auth + REST
 * ─────────────────────────────────────────────────────────────
 *
 *  Dependencias (platformio.ini lib_deps):
 *    bblanchon/ArduinoJson @ ^6.21.5
 * ─────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include "shared.h"
#include "control.h"
#include "wifi_firebase.h"

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Microrred DC — arranque dual-core ===");

  /* Crear mutex antes de lanzar las tareas */
  xMutex = xSemaphoreCreateMutex();

  /* Core 1 — Control PWM (prioridad alta) */
  xTaskCreatePinnedToCore(
    taskControl, "Control",
    8192, nullptr,
    5,    nullptr, 1
  );

  /* Core 0 — Comunicaciones (prioridad normal) */
  xTaskCreatePinnedToCore(
    taskComms, "Comms",
    28672, nullptr,
    2,     nullptr, 0
  );
}

void loop() {
  /* loop() corre en Core 1 junto con taskControl.
     Solo hacemos yield para alimentar el watchdog. */
  vTaskDelay(pdMS_TO_TICKS(1000));
}
