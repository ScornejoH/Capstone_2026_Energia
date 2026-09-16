#pragma once
/*
 * sensors.h — AGENTE 3
 * ─────────────────────────────────────────────────────────────
 * Driver de los dos ADS1115 (I2C a bajo nivel, ±4.096 V, 860 SPS).
 *   ADS1 (main, 0x48): A0=BUS5  A1=Vbat_2S  A2=spare  A3=I_buckboost(bus)
 *   ADS2 (mppt, 0x49): A0=VPV   A1=Vbat2s(opc) A2=IPV  A3=Ibat
 *
 * Corre íntegramente en Core 1 (taskControl).
 * ─────────────────────────────────────────────────────────────
 */
#include <Arduino.h>
#include "shared.h"

/* Inicializa I2C y verifica ADS1 (control del buckboost). Devuelve ads1Ok. */
bool sensorsInit();

/* Reintento de detección del ADS1. */
bool sensorsRetry();

/* Lectura rápida para el buckboost: ADS1 A0/A1/A3 → v_bus5, v_bat2s,
   i_bb_bus5 y SoC. Devuelve false si la lectura falla. */
bool sensorsUpdateBuckboost();

/* Lectura del subsistema PV: ADS2 A0/A2/A3 → v_panel, i_panel, i_bat,
   p_panel. Devuelve false si la lectura falla. */
bool sensorsUpdatePv();

/* Canales (índices single-ended) */
static const uint8_t ADS1_CH_BUS5  = 0;
static const uint8_t ADS1_CH_BAT2S = 1;
static const uint8_t ADS1_CH_SPARE = 2;
static const uint8_t ADS1_CH_IBUS5 = 3;
static const uint8_t ADS2_CH_PANEL  = 0;
static const uint8_t ADS2_CH_BAT2S  = 1;
static const uint8_t ADS2_CH_IPANEL = 2;
static const uint8_t ADS2_CH_IBAT   = 3;
