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
#define DROOP_MASTER           0     // 0 = esclavo: usa la i_ref_v que difunde el maestro
#define MASTER_AGENT_ID        1     // de qué agente se acepta la referencia común i_ref_v

#define CAN_TX_OWN_TELEMETRY   1     // difundir i_out/v_bat/v_bus/SoC por CAN
#define CAN_TX_OWN_FAULTS      1     // emitir faults propios por CAN
#define CAN_DEBUG              0     // 1 = prints de diagnóstico por serial (RX/TX/CFG); poner en 0 al validar

/* Si no llega i_ref_v dentro de este tiempo, el esclavo pasa a su PI de
   voltaje propio (autónomo) y regula el bus por sí solo. */
#define IREFV_TIMEOUT_MS       200

/* Periodo de difusión de telemetría propia (ms). */
#define CAN_TX_PERIOD_MS       1000

/* ============================================================
   PINES SPI / INT  (VSPI; no chocan con I2C 21/22 ni PWM 25/26)
============================================================ */
#define PIN_CAN_SCK    18
#define PIN_CAN_MISO   19
#define PIN_CAN_MOSI   23
#define PIN_CAN_CS      5
#define PIN_CAN_INT    4             // INT MCP2515 en GPIO4 (pin 26); activo bajo, open-drain
#define CAN_SPI_HZ     8000000

/* ============================================================
   CAMPOS DEL ID  (formato 0xCAV)
============================================================ */
#define CAN_CLASS_FAULT    0x0
#define CAN_CLASS_HIGH     0x1
#define CAN_CLASS_LOW      0x2
#define CAN_CLASS_CONFIG   0x4

/* Variables de alta frecuencia (clase HIGH) */
#define CAN_VAR_IOUT       0x0
#define CAN_VAR_IREFV      0x4   // referencia de corriente común (PI de voltaje del maestro)
#define CAN_VAR_VCONS      0x5   // estimación de consenso del voltaje de bus (opción B): float32
#define CAN_VAR_VBUS_ACT   0x6   // v_bus REAL medido, difundido SOLO con control ON (seguridad de bus)

/* ── Seguridad de conexión al bus (comparación de voltaje entre agentes) ── */
#define BUS_CHK_TX_MS      150   // periodo de difusión del v_bus real (con control ON)
#define BUS_CHK_FRESH_MS   500   // frescura de la medición de un par
#define BUS_CHK_DEBOUNCE_MS 1500 // mismatch sostenido antes de declarar desconexión

/* ── Consenso distribuido de promedio del voltaje de bus (opción B) ──
   Cada agente difunde su estimación xᵢ y la actualiza con
   xᵢ += dt·[γ(uᵢ−xᵢ) + kc·Σⱼ(xⱼ−xᵢ)]  (uᵢ = v_bus medido local). */
#define CONS_PERIOD_MS     50    // difusión/actualización con vecinos presentes (20 Hz)
#define CONS_BEACON_MS     1000  // difusión lenta si estás solo (bootstrap)
#define CONS_TIMEOUT_MS    300   // frescura de la estimación de un vecino
#define CONS_GAMMA         3.0f  // atracción a la medición local (1/s)
#define CONS_KC            8.0f  // ganancia de acoplamiento de consenso (1/s)
/* Baja relevancia (clase LOW) — estado de batería/bus, lentos y redundantes.
   La potencia ya no se envía: se calcula en la web (i_out · v_bus). */
#define CAN_VAR_SOC        0x0
#define CAN_VAR_VBAT       0x1
#define CAN_VAR_STATUS     0x2   // estado empaquetado: bit0=run(ON) bit1=topología(BOOST) bit2=desconectado del bus
#define CAN_VAR_VBUS       0x3
#define CAN_VAR_EDELIV     0x6   // energía entregada acumulada (Wh) — metering firmware
#define CAN_VAR_EABSORB    0x7   // energía absorbida acumulada (Wh) — metering firmware

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

/* Config EXTENDIDA: el nibble de 16 índices ya está lleno, así que los
   parámetros extra llegan en un frame CONFIG con var=CAN_VAR_EXT y payload
   [u8 índice][float32] (dlc=5). Se distingue del clásico por el dlc (5 vs 4). */
#define CAN_VAR_EXT        0x0
#define C2_ZETA_V          20    // ζ lazo voltaje
#define C2_ZETA_I          21    // ζ lazo corriente
#define C2_BUS_CHK_EN      22    // habilita la seguridad de conexión al bus (0/1)
#define C2_BUS_CHK_TOL     23    // tolerancia de coincidencia de voltaje (V)

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
