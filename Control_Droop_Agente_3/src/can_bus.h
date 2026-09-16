#pragma once
/*
 * can_bus.h — AGENTE 3 (nodo CAN esclavo droop + subsistema PV/MPPT)
 * ─────────────────────────────────────────────────────────────
 * Conserva la "forma de transacción" MCP2515↔ESP de los otros agentes
 * (driver SPI manual, 500 kbps @ cristal 8 MHz, VSPI) y la recepción por
 * sondeo. El agente 3 es ESCLAVO droop como el agente 2 (recibe i_ref_v del
 * maestro y regula el bus compartiendo corriente, con respaldo autónomo
 * Opción 3), pero maneja DOS convertidores: el buckboost bidireccional
 * (batería 2S ↔ bus 5V) y el boost PV/MPPT.
 *
 * ─────────────────────────────────────────────────────────────
 * PROTOCOLO — formato de ID (11 bits estándar): 0xCAV
 *   C = clase (prioridad, menor ID = mayor prioridad)
 *   A = agente (origen; en config = destino)
 *   V = variable
 *
 * El agente 3 tiene MUCHOS más parámetros (~70) que los 16 que caben en el
 * nibble V. Por eso config y telemetría del agente 3 usan FRAMES EXTENDIDOS:
 *   ID  = canMakeId(clase, 3, CAN_VAR_EXT)
 *   payload (5 bytes) = [u8 índice][float32 valor]
 * El índice (0..255) selecciona el parámetro/variable real. Así un único
 * par (clase,var) transporta todo el set, y el gateway (agente 1) mapea
 * /agente3/params/<clave> ↔ índice extendido.
 *
 * Lo único que el agente 3 RECIBE en alta frecuencia es i_ref_v del maestro,
 * que reutiliza el formato normal (CAN_CLASS_HIGH, MASTER, CAN_VAR_IREFV).
 * ─────────────────────────────────────────────────────────────
 */

#include <Arduino.h>

/* ============================================================
   IDENTIDAD Y FLAGS
============================================================ */
#define THIS_AGENT_ID    3
#define MASTER_AGENT_ID  1
#define DROOP_MASTER     0     // esclavo
#define CAN_DEBUG        0     // 1 = imprime RX (todo lo que llega) + config aplicada (poner en 0 al validar)
#define CAN_DEBUG_TX     0     // 1 = imprime CADA frame de telemetría TX (ruidoso); 0 = solo 1 línea/ciclo

/* Recibe i_ref_v del maestro; si caduca, regula autónomo (Opción 3) */
#define IREFV_TIMEOUT_MS 200

/* Periodo de difusión de telemetría propia (un índice extendido por tick) */
#define CAN_TX_PERIOD_MS 80     // 14 índices ⇒ set completo ≈ cada 1.1 s

/* Tiempo sin RX de config tras el cual no se asume nada (informativo) */
#define CAN_AGENT_TIMEOUT_MS 5000

/* ============================================================
   PINES SPI / INT  (VSPI; iguales a los otros agentes)
============================================================ */
#define PIN_CAN_SCK    18
#define PIN_CAN_MISO   19
#define PIN_CAN_MOSI   23
#define PIN_CAN_CS      5
#define PIN_CAN_INT    4     // INT MCP2515 en GPIO4 (pin 26). GPIO27 = PWM boost PV.
#define CAN_SPI_HZ     8000000

/* ============================================================
   CAMPOS DEL ID  (formato 0xCAV)
============================================================ */
#define CAN_CLASS_FAULT    0x0
#define CAN_CLASS_HIGH     0x1
#define CAN_CLASS_LOW      0x2
#define CAN_CLASS_CONFIG   0x4

/* Alta frecuencia (clase HIGH) */
#define CAN_VAR_IOUT       0x0
#define CAN_VAR_IREFV      0x4   // referencia común del maestro (RX)
#define CAN_VAR_VCONS      0x5   // estimación de consenso del voltaje de bus (opción B): float32
#define CAN_VAR_VBUS_ACT   0x6   // v_bus5 REAL medido, difundido SOLO con control ON (seguridad de bus)

/* ── Seguridad de conexión al bus (comparación de voltaje entre agentes) ── */
#define BUS_CHK_TX_MS      150   // periodo de difusión del v_bus real (con control ON)
#define BUS_CHK_FRESH_MS   500   // frescura de la medición de un par
#define BUS_CHK_DEBOUNCE_MS 1500 // mismatch sostenido antes de declarar desconexión

/* ── Consenso distribuido de promedio del voltaje de bus (opción B) ──
   Cada agente difunde su estimación xᵢ y la actualiza con
   xᵢ += dt·[γ(uᵢ−xᵢ) + kc·Σⱼ(xⱼ−xᵢ)]  (uᵢ = v_bus5 medido local). */
#define CONS_PERIOD_MS     50    // difusión/actualización con vecinos presentes (20 Hz)
#define CONS_BEACON_MS     1000  // difusión lenta si estás solo (bootstrap)
#define CONS_TIMEOUT_MS    300   // frescura de la estimación de un vecino
#define CONS_GAMMA         3.0f  // atracción a la medición local (1/s)
#define CONS_KC            8.0f  // ganancia de acoplamiento de consenso (1/s)

/* Variable reservada para los frames EXTENDIDOS del agente 3
   (telemetría en clase LOW, config en clase CONFIG): payload [u8 idx][f32]. */
#define CAN_VAR_EXT        0x0

/* Códigos de fault (V de la clase FAULT) — compartidos con la red */
#define CAN_FAULT_GENERIC  0x0
#define CAN_FAULT_OVERV    0x1
#define CAN_FAULT_UNDERV   0x2
#define CAN_FAULT_OVERC    0x3
#define CAN_FAULT_SENSOR   0x4
#define CAN_FAULT_BATOV    0x5   // batería 2S sobre-voltaje
#define CAN_FAULT_BATUV    0x6   // batería 2S sub-voltaje
#define CAN_FAULT_PV       0x7   // fault del subsistema PV (genérico, retrocompat)
#define CAN_FAULT_CLEAR    0xF   // "fault despejado"

/* Subtipos de fault del subsistema PV — viajan en el campo pv_fault_code
   (namespace propio, independiente del fault del buckboost). Permiten al
   dashboard mostrar QUÉ protección PV se disparó. */
#define PVF_SENSOR         0x4   // ADS2 (lectura falló) — = CAN_FAULT_SENSOR
#define PVF_BAT_HI         0x11  // batería 2S alta (VBAT2S)
#define PVF_VPV_LOW        0x12  // VPV colapsado (panel)
#define PVF_IPV_HI         0x13  // corriente de panel alta
#define PVF_IBAT_HI        0x14  // corriente de carga de batería alta
#define PVF_IPV_REV        0x15  // corriente de panel reversa/signo
#define PVF_PPV_HI         0x16  // potencia de panel alta

/* ============================================================
   ÍNDICES DE CONFIG EXTENDIDA  (payload [u8 idx][f32])
   Contrato web ↔ gateway ↔ agente 3.  El gateway mapea cada clave de
   /agente3/params a uno de estos índices.
============================================================ */
/* — Buckboost (esclavo droop, mismo lazo de corriente) — */
#define C3_TARGET_V        0
#define C3_OMEGA_V         1
#define C3_OMEGA_I         2
#define C3_PLANT_VDC       3
#define C3_KD_DROOP        4
#define C3_IREF_MAX        5
#define C3_IREF_MIN        6
#define C3_OV_TRIP         7    // bus sobre-voltaje
#define C3_UV_TRIP         8    // bus sub-voltaje
#define C3_OC_TRIP         9    // i_bb sobre-corriente (abs)
#define C3_RUN_MODE        10   // 0/1 buckboost ON
#define C3_FAULT_ACK       11   // one-shot: limpia fault del buckboost
#define C3_BAT_OV_TRIP     12   // batería 2S sobre-voltaje
#define C3_BAT_UV_TRIP     13   // batería 2S sub-voltaje
#define C3_SOC_EMPTY_V     14   // V de 2S a SoC=0
#define C3_SOC_FULL_V      15   // V de 2S a SoC=1
#define C3_PWM_PERIOD_US   16
#define C3_TON_MAX_US      17
#define C3_MIN_OFF_US      18
#define C3_SOFTSTART_S     19
#define C3_ZETA_V          20   // ζ lazo voltaje
#define C3_ZETA_I          21   // ζ lazo corriente
#define C3_MPPT_REACQ_MS   22   // MPPT: tiempo sin potencia antes del barrido de re-adquisición (ms)

/* — Subsistema PV / MPPT (portado del firmware base) — */
#define C3_PV_MODE         30   // 0=OFF 1=SENSORS 2=FIXED 3=VPV_PI 4=MPPT_PO
#define C3_PV_ARM          31   // one-shot/estado: 1=armar 0=desarmar
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
#define C3_OP_MODE         55   // 0=FULL 1=BUCKBOOST_ONLY 2=PV_ONLY
#define C3_PV_FAULT_ACK    56   // one-shot: limpia fault del PV
#define C3_PV_IBAT_LIM     57   // límite SOFT de corriente de carga PV (A, magnitud; 0=off)
#define C3_IBAT_IDLE       58   // corriente de idle del sistema (A): baseline del cero de i_bat
#define C3_BUS_CHK_EN      59   // seguridad de bus: habilitar (0/1)
#define C3_BUS_CHK_TOL     79   // seguridad de bus: tolerancia (V)

/* — Calibración (one-shots y valores directos) — */
#define C3_CAL_BUS5        60   // real V → divFactorBus5
#define C3_CAL_BAT2S       61   // real V → divFactorPv2s
#define C3_CAL_PANEL       62   // real V → divFactorPanel
#define C3_CAL_SPARE       63   // real V → divFactorSpare
#define C3_DIV_BUS5        64   // set directo
#define C3_DIV_BAT2S       65
#define C3_DIV_PANEL       66
#define C3_DIV_SPARE       67
#define C3_IZERO_IBUS      68   // one-shot: 0 A = lectura cruda actual
#define C3_ISENS_IBUS      69   // V/A con signo embebido (valor<0 ⇒ signo −)
#define C3_IZERO_IPANEL    70
#define C3_ISENS_IPANEL    71
#define C3_IZERO_IBAT      72
#define C3_ISENS_IBAT      73
#define C3_KCL_ETA         74
#define C3_SAVE_CFG        75   // one-shot: persistir a NVS
#define C3_IMODE_IBUS      76   // 0=5A 1=20A
#define C3_IMODE_IPANEL    77   // 0=5A 1=20A
#define C3_IMODE_IBAT      78   // 0=5A 1=20A

/* ============================================================
   ÍNDICES DE TELEMETRÍA EXTENDIDA  (payload [u8 idx][f32])
   CAN → gateway → /agentes/agente3/<clave>
============================================================ */
#define T3_V_BUS5          0
#define T3_V_BAT2S         1
#define T3_V_PANEL         2
#define T3_I_BB_BUS5       3    // i_out compartida en el bus
#define T3_I_PANEL         4
#define T3_I_BAT           5
#define T3_P_PANEL         6
#define T3_SOC             7
#define T3_STATUS          8    // bits empaquetados (ver below)
#define T3_TON_CMD         9
#define T3_PV_DUTY         10
#define T3_PV_VPV_REF      11
#define T3_BB_FAULT_CODE   12
#define T3_PV_FAULT_CODE   13
#define T3_E_DELIV         14   // Wh entregados acumulados
#define T3_E_ABSORB        15   // Wh absorbidos acumulados
#define T3_V_MID           16   // voltaje del punto medio de la 2S (A2 spare) → balance de celdas
#define T3_TELE_COUNT      17   // total de índices que cicla txOwnTelemetry

/* Empaquetado de T3_STATUS (float con bits):
   bit0 = buckboost run (ON)
   bit1 = topología BOOST (0 = BUCK)
   bit2 = pv armado
   bit3 = buckboost fault latcheado
   bit4 = pv fault latcheado
   bits 5..7 = pvMode (0..4) */
#define T3_ST_BB_RUN       0x01
#define T3_ST_TOPO_BOOST   0x02
#define T3_ST_PV_ARMED     0x04
#define T3_ST_BB_FAULT     0x08
#define T3_ST_PV_FAULT     0x10
#define T3_ST_PVMODE_SHIFT 5     // bits 5..7 = pvMode
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
   API PÚBLICA
============================================================ */
void taskCAN(void* pvParams);
