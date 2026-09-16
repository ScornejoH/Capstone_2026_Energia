#pragma once
/*
 * can_bus.h
 * ─────────────────────────────────────────────────────────────
 * Sistema CAN del AGENTE 1 (gateway WiFi ↔ CAN) sobre MCP2515.
 *
 * Se conserva la "forma de transacción" MCP2515↔ESP del proyecto
 * de referencia (driver SPI manual: reset/read/write/bitmod/RTS,
 * 500 kbps @ cristal 8 MHz, VSPI). Se AÑADE recepción (RX) por
 * interrupción (pin INT) que el TX-only original no tenía.
 *
 * ─────────────────────────────────────────────────────────────
 * PROTOCOLO — formato único de ID (11 bits estándar): 0xCAV
 *   C = clase     (nibble alto)   → prioridad (menor ID = mayor prioridad)
 *   A = agente    (nibble medio)  → 1..3 (origen; en config = destino)
 *   V = variable  (nibble bajo)   → 0..15
 *
 *   C=0  FAULT          A=origen   V=código fault     payload: 1 byte código
 *   C=1  ALTA FRECUENCIA A=origen  V: 0=i_out                   payload: float32
 *   C=2  BAJA RELEVANCIA A=origen  V: 0=SoC 1=v_bat 3=v_bus     payload: float32
 *   C=4  CONFIG         A=destino  V=índice parámetro  payload: float32
 *
 * Ejemplos:  0x013 = agente1 fault sobrecorriente
 *            0x120 = agente2 i_out      0x230 = agente3 SoC
 *            0x425 = config a agente2, parámetro 5 (i_ref_max)
 *
 * El Agente 1:
 *   - Recibe alta/baja frec. y faults de los agentes 2 y 3 → reenvía a
 *     Firebase (lo hace taskComms leyendo canGetAgentData()).
 *   - Transmite su propia telemetría y faults (togglable abajo).
 *   - Retransmite por CAN la config que la web deja en Firebase para
 *     los agentes 2 y 3 (taskComms encola con canQueueConfig()).
 * ─────────────────────────────────────────────────────────────
 */

#include <Arduino.h>

/* ============================================================
   IDENTIDAD Y FLAGS (modificables al reprogramar la ESP32)
============================================================ */
#define THIS_AGENT_ID          1     // este nodo es el agente 1

#define CAN_TX_OWN_TELEMETRY   1     // 1 = agente1 difunde i_out/v_bat/SoC por CAN
#define CAN_TX_OWN_FAULTS      1     // 1 = agente1 emite sus faults por CAN
#define CAN_RELAY_CONFIG       1     // 1 = agente1 retransmite config a 2/3
#define CAN_DEBUG              0     // 1 = prints de diagnóstico por serial (RX/TX/CFG); poner en 0 al validar

/* Periodo de difusión de telemetría propia (ms). ~ ritmo de la web. */
#define CAN_TX_PERIOD_MS       1000

/* Tiempo sin RX tras el cual un agente se marca offline (ms). */
#define CAN_AGENT_TIMEOUT_MS   5000

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

/* Códigos de fault (V de la clase FAULT) */
#define CAN_FAULT_GENERIC  0x0
#define CAN_FAULT_OVERV    0x1
#define CAN_FAULT_UNDERV   0x2
#define CAN_FAULT_OVERC    0x3
#define CAN_FAULT_SENSOR   0x4

/* Índices de parámetro de configuración (V de la clase CONFIG, 0..15).
   Espejan los parámetros que cada agente acepta (ver /agenteN/params). */
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
#define CFG_FAULT_ACK      0xB   // one-shot: limpia el fault latcheado del agente destino
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
   DATOS RECIBIDOS DE UN AGENTE REMOTO (para reenvío a WiFi)
============================================================ */
struct CanAgentData {
  bool     online;       // ha llegado algo dentro del timeout
  uint32_t lastRxMs;     // millis() del último frame recibido
  float    i_out;        // A
  float    v_bat;        // V
  float    v_bus;        // V (bus del remoto; para calibración y chequeo de discrepancia)
  float    soc;          // [0,1]
  bool     fault;        // último fault recibido sin reconocer
  uint8_t  faultCode;    // CAN_FAULT_*
};

/* ============================================================
   API PÚBLICA
============================================================ */

/* Tarea FreeRTOS — Core 0. Inicializa el MCP2515 y gestiona TX/RX. */
void taskCAN(void* pvParams);

/* Copia atómica de los datos recibidos del agente `agent` (2 ó 3).
   Retorna false si el agente no es válido. */
bool canGetAgentData(uint8_t agent, CanAgentData* out);

/* Encola un frame de configuración hacia el agente destino (2 ó 3).
   Lo llama taskComms cuando la web cambia un parámetro en Firebase.
   Retorna false si la cola está llena o el destino no es válido. */
bool canQueueConfig(uint8_t destAgent, uint8_t paramIdx, float value);
