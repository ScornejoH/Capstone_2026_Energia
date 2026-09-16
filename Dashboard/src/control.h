#pragma once
/*
 * control.h
 * ─────────────────────────────────────────────────────────────
 * Control PWM buck-boost con FF+PI y lectura ADS1115.
 * Corre en Core 1 como tarea FreeRTOS de alta prioridad.
 * ─────────────────────────────────────────────────────────────
 */

#include <Arduino.h>

/* ── PWM ── */
void configHwPwm();
void allOff();
void driveBoost(float ton, float allowed, float period);
void driveBuck (float ton, float allowed, float period);
float getAllowedTonMax(float period, float tMax, float tHard, float minOff);

/* ── ADS1115 ── */
bool configFastCh();
bool updateFast();
bool updateTelemetry();

/* ── Protecciones y control ── */
void tripFault(const char* reason);
bool checkProtections();
void updateFFPI();

/* ── Tarea FreeRTOS (Core 1) ── */
void taskControl(void* pvParams);
