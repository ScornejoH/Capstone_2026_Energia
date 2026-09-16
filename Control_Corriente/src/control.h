#pragma once
/*
 * control.h
 * ─────────────────────────────────────────────────────────────
 * Lazos de control de corriente + PWM LEDC + protecciones.
 *
 * Esquema (ejecutado en Core 1 cada controlPeriodUs):
 *
 *   [Lazo externo — voltaje]
 *     error_v  = targetV - v_bus
 *     i_ref_v  = kpV·error_v + kiV·∫error_v dt
 *
 *   [Droop SoC]
 *     i_ref_d  = kdDroop·(SoC - 0.5)
 *
 *   [Referencia total]
 *     i_ref    = clamp(i_ref_v + i_ref_d, iRefMin, iRefMax)
 *
 *   [Lazo interno — corriente]
 *     error_i  = i_ref - i_out
 *     duty     = kpI·error_i + kiI·∫error_i dt
 *
 *   [Selección de topología por signo de i_ref]
 *     i_ref > 0 → BOOST  (batería → bus)
 *     i_ref < 0 → BUCK   (bus → batería)
 *
 *   [Protecciones]
 *     Sobrevoltaje / Subvoltaje en bus → tripFault()
 *     Sobrecorriente (|i_out|)         → tripFault()
 *     ADS desconectado                 → control inhibido
 * ─────────────────────────────────────────────────────────────
 */

#include <Arduino.h>
#include "shared.h"

/* Inicializar PWM hardware (llamar desde taskControl) */
void initPwm();

/* Apagar ambos canales PWM de forma segura */
void allOff();

/* Recalcular Kp/Ki de ambos lazos desde omega/zeta.
   Llamar tras modificar omegaV/zetaV/omegaI/zetaI. */
void recomputeGains();

/* Activar fault: apaga PWM, latchea faultLatched, guarda el código de la
   protección activada y pide escritura a Firebase */
void tripFault(FaultCode code);

/* Verificar umbrales de protección (retorna false si se activó un trip) */
bool checkProtections();

/* Tarea FreeRTOS — Core 1 */
void taskControl(void* pvParams);
