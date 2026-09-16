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
 *   C=1  ALTA FRECUENCIA A=origen  V: 0=i_out 4=i_ref_v(maestro) payload: float32
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
#define DROOP_MASTER           1     // 1 = este nodo corre el PI de voltaje y difunde i_ref_v

#define CAN_TX_OWN_TELEMETRY   1     // 1 = agente1 difunde i_out/v_bat/SoC por CAN
#define CAN_TX_OWN_FAULTS      1     // 1 = agente1 emite sus faults por CAN
#define CAN_RELAY_CONFIG       1     // 1 = agente1 retransmite config a 2/3
#define CAN_DEBUG              0     // 1 = prints de diagnóstico por serial (RX/TX/CFG); poner en 0 al validar

/* Periodo de difusión de telemetría propia (ms). ~ ritmo de la web. */
#define CAN_TX_PERIOD_MS       1000

/* Periodo de difusión de la referencia común i_ref_v (señal de control → rápida). */
#define CAN_IREFV_PERIOD_MS    30

/* Tiempo sin RX tras el cual un agente se marca offline (ms). */
#define CAN_AGENT_TIMEOUT_MS   5000

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
   xᵢ += dt·[γ(uᵢ−xᵢ) + kc·Σⱼ(xⱼ−xᵢ)]  (uᵢ = v_bus medido local).
   Converge al promedio de las mediciones; kc≫γ ⇒ los 3 concuerdan. */
#define CONS_PERIOD_MS     50    // difusión/actualización con vecinos presentes (20 Hz)
#define CONS_BEACON_MS     1000  // difusión lenta si estás solo (bootstrap, evita stall de TX sin ACK)
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

/* Agente 3: frames EXTENDIDOS (telemetría LOW, config CONFIG) con índice en
   el payload → [u8 idx][float32], dlc=5. Se distinguen de la telemetría
   normal del agente 2 por el dlc (5 vs 4). */
#define CAN_VAR_EXT        0x0

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

/* Config EXTENDIDA para el agente 2 (el nibble ya está lleno): frame CONFIG
   con var=CAN_VAR_EXT y payload [u8 idx][f32], dlc=5. */
#define C2_ZETA_V          20    // ζ lazo voltaje
#define C2_ZETA_I          21    // ζ lazo corriente
#define C2_BUS_CHK_EN      22    // seguridad de bus: habilitar (0/1)
#define C2_BUS_CHK_TOL     23    // seguridad de bus: tolerancia (V)

/* ============================================================
   AGENTE 3 — PROTOCOLO EXTENDIDO (contrato web ↔ gateway ↔ agente 3)
   Debe coincidir con can_bus.h de Control_Droop_Agente_3.
============================================================ */
/* Índices de CONFIG extendida (payload [u8 idx][f32]) */
#define C3_TARGET_V        0
#define C3_OMEGA_V         1
#define C3_OMEGA_I         2
#define C3_PLANT_VDC       3
#define C3_KD_DROOP        4
#define C3_IREF_MAX        5
#define C3_IREF_MIN        6
#define C3_OV_TRIP         7
#define C3_UV_TRIP         8
#define C3_OC_TRIP         9
#define C3_RUN_MODE        10
#define C3_FAULT_ACK       11
#define C3_BAT_OV_TRIP     12
#define C3_BAT_UV_TRIP     13
#define C3_SOC_EMPTY_V     14
#define C3_SOC_FULL_V      15
#define C3_PWM_PERIOD_US   16
#define C3_TON_MAX_US      17
#define C3_MIN_OFF_US      18
#define C3_SOFTSTART_S     19
#define C3_ZETA_V          20
#define C3_ZETA_I          21
#define C3_MPPT_REACQ_MS   22   // MPPT: tiempo sin potencia antes del barrido de re-adquisición (ms)
#define C3_PV_MODE         30
#define C3_PV_ARM          31
#define C3_PV_FIXED_DUTY   32
#define C3_PV_VPV_REF      33
#define C3_PV_VPV_REF_MIN  34
#define C3_PV_VPV_REF_MAX  35
#define C3_PV_KP           36
#define C3_PV_KI           37
#define C3_PV_DUTY_MIN     38
#define C3_PV_DUTY_START   39
#define C3_PV_DUTY_MAX     40
#define C3_PV_DUTY_SLEW    41
#define C3_PV_SOFTSTART_MS 42
#define C3_PV_CTRL_MS      43
#define C3_PV_PANEL_TRIPLO 44
#define C3_PV_BAT_WARN_HI  45
#define C3_PV_BAT_TRIP_HI  46
#define C3_PV_IIN_MAX      47
#define C3_PV_IBAT_MAX     48
#define C3_PV_PIN_MAX      49
#define C3_PV_IREV_MAX     50
#define C3_MPPT_STEP_V     51
#define C3_MPPT_EPS_POWER  52
#define C3_MPPT_MIN_POWER  53
#define C3_MPPT_PERIOD_MS  54
#define C3_OP_MODE         55
#define C3_PV_FAULT_ACK    56
#define C3_PV_IBAT_LIM     57
#define C3_IBAT_IDLE       58   // corriente de idle del sistema (A): baseline del cero de i_bat
#define C3_BUS_CHK_EN      59   // seguridad de bus: habilitar (0/1)
#define C3_BUS_CHK_TOL     79   // seguridad de bus: tolerancia (V)
#define C3_CAL_BUS5        60
#define C3_CAL_BAT2S       61
#define C3_CAL_PANEL       62
#define C3_CAL_SPARE       63
#define C3_DIV_BUS5        64
#define C3_DIV_BAT2S       65
#define C3_DIV_PANEL       66
#define C3_DIV_SPARE       67
#define C3_IZERO_IBUS      68
#define C3_ISENS_IBUS      69
#define C3_IZERO_IPANEL    70
#define C3_ISENS_IPANEL    71
#define C3_IZERO_IBAT      72
#define C3_ISENS_IBAT      73
#define C3_KCL_ETA         74
#define C3_SAVE_CFG        75
#define C3_IMODE_IBUS      76   // 0=5A 1=20A (rango ACS712)
#define C3_IMODE_IPANEL    77
#define C3_IMODE_IBAT      78

/* Índices de TELEMETRÍA extendida (payload [u8 idx][f32]) */
#define T3_V_BUS5          0
#define T3_V_BAT2S         1
#define T3_V_PANEL         2
#define T3_I_BB_BUS5       3
#define T3_I_PANEL         4
#define T3_I_BAT           5
#define T3_P_PANEL         6
#define T3_SOC             7
#define T3_STATUS          8
#define T3_TON_CMD         9
#define T3_PV_DUTY         10
#define T3_PV_VPV_REF      11
#define T3_BB_FAULT_CODE   12
#define T3_PV_FAULT_CODE   13
#define T3_E_DELIV         14   // Wh entregados acumulados
#define T3_E_ABSORB        15   // Wh absorbidos acumulados
#define T3_V_MID           16   // voltaje del punto medio de la 2S (balance de celdas)
/* Bits de T3_STATUS */
#define T3_ST_BB_RUN       0x01
#define T3_ST_TOPO_BOOST   0x02
#define T3_ST_PV_ARMED     0x04
#define T3_ST_BB_FAULT     0x08
#define T3_ST_PV_FAULT     0x10
#define T3_ST_PVMODE_SHIFT 5     // bits 5..7 = pv_mode
#define T3_ST_BUS_DISC     0x100 // bit 8 = buckboost desconectado del bus (seguridad)

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
  float    e_deliv;      // Wh entregados acumulados (metering firmware)
  float    e_absorb;     // Wh absorbidos acumulados (metering firmware)
  bool     running;      // control activo (run_mode ON) reportado por el remoto
  uint8_t  topo;         // 0 = BUCK, 1 = BOOST
  bool     busDisc;      // seguridad: reportado desconectado del bus (bit2 del status)
  bool     fault;        // último fault recibido sin reconocer
  uint8_t  faultCode;    // CAN_FAULT_*
};

/* ============================================================
   DATOS RECIBIDOS DEL AGENTE 3 (telemetría extendida, para reenvío a WiFi)
============================================================ */
struct CanAgent3Data {
  bool     online;
  uint32_t lastRxMs;
  float    v_bus5, v_bat2s, v_panel, v_mid;
  float    i_bb, i_panel, i_bat, p_panel, soc;
  float    e_deliv, e_absorb;
  float    ton_cmd, pv_duty, pv_vpv_ref;
  bool     bb_run;        // buckboost control activo
  uint8_t  topo;          // 0=BUCK 1=BOOST
  bool     busDisc;       // seguridad: buckboost desconectado del bus
  bool     pv_armed;
  uint8_t  pv_mode;       // 0..4
  bool     bb_fault;
  uint8_t  bb_fault_code;
  bool     pv_fault;
  uint8_t  pv_fault_code;
};

/* ============================================================
   API PÚBLICA
============================================================ */

/* Tarea FreeRTOS — Core 0. Inicializa el MCP2515 y gestiona TX/RX. */
void taskCAN(void* pvParams);

/* Copia atómica de los datos del agente 2 (esquema clásico). */
bool canGetAgentData(uint8_t agent, CanAgentData* out);

/* Copia atómica de la telemetría extendida del agente 3. */
bool canGetAgent3(CanAgent3Data* out);

/* Encola config clásica (4 bytes) hacia el agente 2 (paramIdx 0..15). */
bool canQueueConfig(uint8_t destAgent, uint8_t paramIdx, float value);

/* Encola config EXTENDIDA (índice en payload) hacia el agente destino (2 ó 3). */
bool canQueueConfigExt(uint8_t destAgent, uint8_t paramIdx, float value);

/* Estimación de consenso del voltaje de bus del agente `agent` (1..3).
   Devuelve false si esa estimación no es fresca (agente ausente). */
bool canGetConsensus(uint8_t agent, float* out);
