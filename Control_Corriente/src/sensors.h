#pragma once
/*
 * sensors.h
 * ─────────────────────────────────────────────────────────────
 * Lectura ADS1115 (I2C) y estimación SoC por LUT.
 *
 * Canales ADS1115:
 *   A0 → divisor voltaje bus   → v_bus
 *   A1 → divisor voltaje bat   → v_bat
 *   A3 → ACS712 5A             → i_out
 *
 * ACS712 5A:
 *   Sensibilidad : 185 mV/A
 *   Offset (0 A) : acsOffset (defecto 2.500 V con VCC=5V)
 *   i_out = (v_A3 - acsOffset) / 0.185
 *   Positivo = corriente desde batería hacia bus (BOOST)
 *   Negativo = corriente desde bus hacia batería (BUCK)
 * ─────────────────────────────────────────────────────────────
 */

#include <Arduino.h>

/* Inicializar I2C y verificar presencia del ADS1115 */
bool sensorsInit();

/* Leer los tres canales y actualizar v_bus, v_bat, i_out en shared.
   Retorna true si la lectura fue exitosa. */
bool sensorsUpdate();

/* Estimar SoC desde voltaje de batería usando LUT.
   Retorna valor en [0.0, 1.0]. */
float estimateSoC(float v_bat);

/* Intentar reconectar el ADS tras fallo */
bool sensorsRetry();
