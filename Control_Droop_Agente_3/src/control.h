#pragma once
/*
 * control.h — AGENTE 3
 * Tarea de control (Core 1): buckboost (lazo de corriente cascada, esclavo
 * droop) + subsistema PV/MPPT. Más utilidades llamadas desde taskCAN.
 */
#include <Arduino.h>
#include "shared.h"

/* Forzar salidas PWM a OFF (buckboost + boost PV) con digitalWrite directo, sin
   depender de LEDC ni de las tareas. Llamar como PRIMERA línea de setup() para
   minimizar la ventana en que los gates flotan tras el reset. Además fija pull
   internos en la dirección segura como respaldo ante cualquier alta-Z. */
void pwmForceSafe();

/* Tarea FreeRTOS — Core 1 */
void taskControl(void* pvParams);

/* Recalcula kpV/kiV/kpI/kiI desde omegaV/omegaI/plantL/plantC/plantVdc.
   Segura para llamar desde Core 0 (toma el mutex). */
void recomputeGains();

/* Trip de fault del buckboost (apaga y latchea). */
void tripFault(const char* reason, uint8_t code);

/* Trip de fault del subsistema PV. */
void pvTripFault(const char* reason, uint8_t code);
