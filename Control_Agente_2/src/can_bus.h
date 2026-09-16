#pragma once
/*
 * can_bus.h  — AGENTE 2 (nodo CAN, sin WiFi)
 * ─────────────────────────────────────────────────────────────
 * Nodo CAN sobre MCP2515. A diferencia del agente 1 (gateway WiFi↔CAN),
 * este nodo:
 *   - TRANSMITE su telemetría (i_out, v_bat, v_bus, SoC) y sus faults.
 *   - RECIBE frames de CONFIG dirigidos a ÉL (A = THIS_AGENT_ID) y los
 *     aplica a las variables compartidas (igual que readParams hacía
 *     desde Firebase en el agente 1).
 *   - NO reenvía nada a otros agentes ni habla con Firebase.
 *
 * El formato de ID/payload es idéntico al del gateway para que ambos
 * proyectos hablen el mismo "idioma" en el bus.
 * ─────────────────────────────────────────────────────────────
 * PROTOCOLO — ID estándar 11 bits: 0xCAV
 *   C = clase (nibble alto)   A = agente (nibble medio)   V = variable (bajo)
 *   C=0 FAULT   C=1 ALTA FREC.   C=2 BAJA RELEV.   C=4 CONFIG
 * ─────────────────────────────────────────────────────────────
 */

#include <Arduino.h>

/* ============================================================
   IDENTIDAD Y FLAGS
============================================================ */
#define THIS_AGENT_ID          2     // este nodo es el agente 2

#define CAN_TX_OWN_TELEMETRY   1     // difundir i_out/v_bat/v_bus/SoC por CAN
#define CAN_TX_OWN_FAULTS      1     // emitir faults propios por CAN
#define CAN_DEBUG              0     // 1 = prints de diagnóstico por serial (RX/TX/CFG); poner en 0 al validar

/* Periodo de difusión de telemetría propia (ms). */
#define CAN_TX_PERIOD_MS       1000

/* ============================================================
   PINES SPI / INT  (VSPI; no chocan con I2C 21/22 ni PWM 25/26)
============================================================ */
#define PIN_CAN_SCK    18
#define PIN_CAN_MISO   19
#define PIN_CAN_MOSI   23
#define PIN_CAN_CS      5
#define PIN_CAN_INT    27            // INT del MCP2515 (activo bajo, open-drain)
#define CAN_SPI_HZ     8000000

/* ============================================================
   CAMPOS DEL ID  (formato 0xCAV)
============================================================ */
#define CAN_CLASS_FAULT    0x0
#define CAN_CLASS_HIGH     0x1
#define CAN_CLASS_LOW      0x2
#define CAN_CLASS_CONFIG   0x4

/* Variable de alta frecuencia (clase HIGH) — solo la corriente */
#define CAN_VAR_IOUT       0x0
/* Baja relevancia (clase LOW) — estado de batería/bus, lentos y redundantes.
   La potencia ya no se envía: se calcula en la web (i_out · v_bus). */
#define CAN_VAR_SOC        0x0
#define CAN_VAR_VBAT       0x1
#define CAN_VAR_VBUS       0x3

/* Códigos de fault (V de la clase FAULT).
   0x0 se reserva como "sin fault / fault despejado" (clear). */
#define CAN_FAULT_CLEAR    0x0
#define CAN_FAULT_GENERIC  0x0
#define CAN_FAULT_OVERV    0x1
#define CAN_FAULT_UNDERV   0x2
#define CAN_FAULT_OVERC    0x3
#define CAN_FAULT_SENSOR   0x4

/* Índices de parámetro de configuración (V de la clase CONFIG, 0..15).
   Deben coincidir con CFG_KEYS del gateway (agente 1). */
#define CFG_TARGET_V       0x0
#define CFG_OMEGA_V        0x1
#define CFG_OMEGA_I        0x2
#define CFG_PLANT_VDC      0x3
#define CFG_KD_DROOP       0x4
#define CFG_IREF_MAX       0x5
#define CFG_IREF_MIN       0x6
#define CFG_OV_TRIP        0x7
#define CFG_UV_TRIP        0x8
#define CFG_OC_TRIP        0x9
#define CFG_RUN_MODE       0xA
#define CFG_FAULT_ACK      0xB   // one-shot: limpia el fault latcheado de este agente
#define CFG_CAL_BUS        0xC   // one-shot: calibrar divisor de bus al voltaje real recibido
#define CFG_CAL_BAT        0xD   // one-shot: calibrar divisor de batería al voltaje real recibido
#define CFG_ACS_ZERO       0xE   // one-shot: fijar offset ACS = lectura cruda actual (0 A)
#define CFG_ACS_SENS       0xF   // sensibilidad ACS con signo (valor<0 ⇒ signo invertido)

/* ============================================================
   CONSTRUCCIÓN / DECODIFICACIÓN DE ID
============================================================ */
static inline uint16_t canMakeId(uint8_t cls, uint8_t agent, uint8_t var) {
  return (uint16_t)(((cls & 0xF) << 8) | ((agent & 0xF) << 4) | (var & 0xF));
}
static inline uint8_t canIdClass(uint16_t id) { return (id >> 8) & 0xF; }
static inline uint8_t canIdAgent(uint16_t id) { return (id >> 4) & 0xF; }
static inline uint8_t canIdVar  (uint16_t id) { return  id       & 0xF; }

/* ============================================================
   API PÚBLICA
============================================================ */

/* Tarea FreeRTOS — Core 0. Inicializa el MCP2515 y gestiona TX/RX. */
void taskCAN(void* pvParams);
