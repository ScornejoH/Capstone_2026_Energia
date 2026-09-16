#pragma once
/*
 * can_bus.h
 * ─────────────────────────────────────────────────────────────
 * Driver manual MCP2515 vía SPI (sin librería externa).
 * 500 kbps @ cristal 8 MHz, VSPI estándar del ESP32.
 *
 * Incluye también la simulación de datos del agente 2
 * que se transmite periódicamente por el bus CAN.
 * ─────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include <SPI.h>

/* IDs CAN para los datos del agente 2 */
#define CAN_ID_VBAT    0x200
#define CAN_ID_VBUS    0x201
#define CAN_ID_IOUT    0x202

/* ── Inicialización ── */
bool canInit();

/* ── Transmisión ── */
bool canSendFloat(uint16_t id, float val);

/* ── Simulación agente 2 ── */
void simAgente2(float t, float* vbat, float* vbus, float* iout);
