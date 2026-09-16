/*
 * control.cpp
 * ─────────────────────────────────────────────────────────────
 * Lazos PI de voltaje y corriente + droop SoC + PWM LEDC.
 * Corre íntegramente en Core 1.
 *
 * Parámetros PI derivados de frecuencia natural y amortiguamiento:
 *   Kp = 2 · ζ · ω_n
 *   Ki = ω_n²
 *
 * El lazo de voltaje (externo, lento) genera i_ref.
 * El lazo de corriente (interno, rápido) genera el duty cycle.
 * La topología se selecciona automáticamente por el signo de i_ref.
 * ─────────────────────────────────────────────────────────────
 */

#include "control.h"
#include "shared.h"
#include "sensors.h"
#include "can_bus.h"
#include <math.h>
#include "driver/gpio.h"

/* ============================================================
   PWM LEDC
============================================================ */
static const uint8_t  CH_HIGH  = 0;
static const uint8_t  CH_LOW   = 1;
static const uint8_t  PWM_BITS = 10;
static const uint32_t DUTY_MAX = (1UL << PWM_BITS) - 1;

static bool pwmReady = false;

/* ============================================================
   SALIDA PWM SEGURA EN ARRANQUE
   Medio puente OFF: QH=HIGH (activo-LOW ⇒ OFF), QL=LOW (activo-HIGH ⇒ OFF).
   Se llama como PRIMERA línea de setup(), antes de Serial/tareas, para acortar
   al mínimo la ventana en que los gates flotan tras el reset. Los pull internos
   (~45 kΩ) en la dirección segura son un respaldo: solo actúan si el pin queda
   en alta-Z (p. ej. durante ledcAttachPin); NO cubren el arranque previo a
   setup() (ahí el pin flota hasta que corre este código). Para cubrir el reset
   completo, reflasheo y brownout hacen falta pull EXTERNOS en el driver.
============================================================ */
void pwmForceSafe() {
  pinMode(PIN_PWM_HIGH, OUTPUT); digitalWrite(PIN_PWM_HIGH, HIGH);   // QH activo-LOW → OFF
  pinMode(PIN_PWM_LOW,  OUTPUT); digitalWrite(PIN_PWM_LOW,  LOW);    // QL activo-HIGH → OFF
  gpio_set_pull_mode((gpio_num_t)PIN_PWM_HIGH, GPIO_PULLUP_ONLY);    // si flota → HIGH (OFF)
  gpio_set_pull_mode((gpio_num_t)PIN_PWM_LOW,  GPIO_PULLDOWN_ONLY);  // si flota → LOW  (OFF)
}

void initPwm() {
  uint32_t period;
  xSemaphoreTake(xMutex, portMAX_DELAY);
  period = pwmPeriodUs;
  xSemaphoreGive(xMutex);

  uint32_t freq = period ? 1000000UL / period : 20000;
  ledcSetup(CH_HIGH, freq, PWM_BITS);  ledcAttachPin(PIN_PWM_HIGH, CH_HIGH);
  ledcSetup(CH_LOW,  freq, PWM_BITS);  ledcAttachPin(PIN_PWM_LOW,  CH_LOW);
  pwmReady = true;
}

void allOff() {
  if (pwmReady) {
    ledcWrite(CH_HIGH, DUTY_MAX);
    ledcWrite(CH_LOW,  0);
  } else {
    pinMode(PIN_PWM_HIGH, OUTPUT);
    pinMode(PIN_PWM_LOW,  OUTPUT);
    digitalWrite(PIN_PWM_HIGH, HIGH);
    digitalWrite(PIN_PWM_LOW,  LOW);
  }
}

static uint32_t dutyFromUs(float ton, float period) {
  if (ton <= 0.0f || period <= 0.0f) return 0;
  uint32_t d = (uint32_t)lroundf((ton / period) * (float)DUTY_MAX);
  return d > DUTY_MAX ? DUTY_MAX : d;
}

static void driveBoost(float ton, float period, float maxTon) {
  ton = clampF(ton, 0.0f, maxTon);
  ledcWrite(CH_HIGH, DUTY_MAX);
  ledcWrite(CH_LOW,  dutyFromUs(ton, period));
}

static void driveBuck(float ton, float period, float maxTon) {
  ton = clampF(ton, 0.0f, maxTon);
  ledcWrite(CH_HIGH, DUTY_MAX - dutyFromUs(ton, period));
  ledcWrite(CH_LOW,  0);
}

/* ============================================================
   CÁLCULO DE GANANCIAS PI DESDE FRECUENCIAS DE CRUCE
   L y C son constantes de hardware (PLANT_L_H, PLANT_C_F).
   El usuario ajusta omegaV y omegaI (wc en rad/s).

   Lazo voltaje (planta Gv=1/(sC)):
     Kp_v = wc_v · C            Ki_v = Kp_v · wc_v / 10
   Lazo corriente (planta Gi=Vdc/(sL), salida = duty adimensional):
     Kp_i = wc_i · L / Vdc      Ki_i = Kp_i · wc_i / 10
   El /Vdc es imprescindible: sin él Kp_i queda Vdc veces demasiado grande
   y el cruce real se dispara → oscilación.
============================================================ */
void recomputeGains() {
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
    float vdc = plantVdc < 0.5f ? 0.5f : plantVdc;   // piso para evitar ganancia infinita
    /* Ubicación de polos 2.º orden: lazo cerrado = s²+2ζω_n s+ω_n² */
    kpV = 2.0f * zetaV * omegaV * PLANT_C_F;
    kiV = omegaV * omegaV * PLANT_C_F;
    kpI = 2.0f * zetaI * omegaI * PLANT_L_H / vdc;
    kiI = omegaI * omegaI * PLANT_L_H / vdc;
    xSemaphoreGive(xMutex);
  }
}

/* ============================================================
   FAULT
============================================================ */
void tripFault(const char* reason, uint8_t code) {
  allOff();
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    faultLatched  = true;
    faultCode     = code;
    runMode       = RUN_OFF;
    requestedMode = RUN_OFF;
    xSemaphoreGive(xMutex);
  }
  pendingFaultWrite = true;
}

/* ============================================================
   PROTECCIONES
============================================================ */
bool checkProtections() {
  float vb, iout, ovTrip, uvTrip, ocTrip;
  bool  enabled;
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) != pdTRUE) return true;
  vb     = v_bus;
  iout   = i_out;
  ovTrip = overvoltageTrip;
  uvTrip = undervoltageTrip;
  ocTrip = overcurrentTrip;
  enabled = faultsEnabled;
  xSemaphoreGive(xMutex);

  if (!enabled) return true;

  if (vb > ovTrip) { tripFault("Sobrevoltaje en bus", CAN_FAULT_OVERV);     return false; }
  if (vb < uvTrip) { tripFault("Subvoltaje en bus",   CAN_FAULT_UNDERV);    return false; }
  if (fabsf(iout) > ocTrip) { tripFault("Sobrecorriente", CAN_FAULT_OVERC); return false; }
  return true;
}

/* ============================================================
   MINI SMART METERING — energía real integrada en firmware (Wh)
   P = v_bus · i_out_filt (potencia en el bus). Entregada si P>0, absorbida
   si P<0. Decimado (4 Hz) + banda muerta (ignora el offset del sensor) +
   guarda contra huecos largos (arranque/OFF/fault) para no integrar energía
   falsa. Acumuladores en double: sin pérdida de precisión en corridas largas.
============================================================ */
static void accumulateEnergy(float vbus, float iout) {
  static double   eD = 0.0, eA = 0.0;
  static uint32_t lastMs = 0;
  const uint32_t  PERIOD_MS = 250;
  const float     IDEADBAND = 0.02f;   // A
  uint32_t now = millis();
  if (lastMs == 0) { lastMs = now; return; }
  uint32_t dtm = now - lastMs;
  if (dtm < PERIOD_MS) return;          // decimación
  lastMs = now;
  if (dtm > 2000) return;               // hueco largo → discontinuidad, no integrar
  if (fabsf(iout) < IDEADBAND) return;  // sin flujo neto → no integrar offset
  float dtH = (float)dtm / 3600000.0f;
  float p   = vbus * iout;
  if (p >= 0.0f) eD += (double)p * dtH; else eA += (double)(-p) * dtH;
  e_deliv  = (float)eD;
  e_absorb = (float)eA;
}

/* ============================================================
   TAREA FREERTOS — Core 1
============================================================ */
void taskControl(void* pvParams) {
  /* ── Init sensores ── */
  bool adsOk = sensorsInit();
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
    hwStatus = adsOk ? HW_OK : HW_NO_ADS;
    xSemaphoreGive(xMutex);
  }
  Serial.printf("[SENS] ADS1115: %s\n", adsOk ? "OK" : "NO detectado");

  /* ── Init PWM ── */
  initPwm();
  allOff();

  /* ── Calcular ganancias desde los omega/Vdc por defecto, para que el
        control esté bien sintonizado aunque arranque sin WiFi/Firebase. ── */
  recomputeGains();

  /* ── Integradores PI ── */
  float integV  = 0.0f;
  float integI  = 0.0f;
  float iOutFilt = 0.0f;  // corriente filtrada (paso bajo ~20 Hz)
  float softStart = 0.0f; // rampa de soft-start [0..1], se reinicia en cada OFF→ON

  /* ── Tiempo de control (µs → s) ── */
  static const float DT = 0.001f;   // 1 ms = periodo de control
  static const uint32_t ADS_RETRY_MS    = 5000;
  static const uint8_t  ADS_FAIL_THRESH = 5;
  uint8_t  adsFailCount = 0;
  uint32_t adsLastRetry = 0;

  /* ── Topología anterior (para detectar cambio de modo) ── */
  TopologyMode prevTopo = TOPO_BOOST;

  uint32_t lastMs = millis();

  for (;;) {
    /* El yield al IDLE (watchdog) lo hace adsSingleShot con vTaskDelay(1)
       DURANTE la conversión del ADS, solapando espera y cesión de CPU → el
       lazo sube a ~700 Hz. Las ramas que NO leen sensores ceden explícitamente. */
    uint32_t now = millis();
    float dt = clampF((float)(now - lastMs) * 0.001f, 0.001f, 0.050f);  // s, máx 50 ms
    lastMs = now;

    /* Metering (usa los valores del ciclo previo; se auto-decima a 4 Hz). */
    accumulateEnergy(v_bus, i_out_filt);

    /* ── Medidor de frecuencia del lazo (= muestreo de corriente) ──
       Imprime cada 1 s los Hz reales del lazo y del bus (loop/N_BUS).
       Poner LOOP_HZ_DEBUG en false al terminar de caracterizar. */
    static const bool LOOP_HZ_DEBUG = false;
    static uint32_t loopCnt = 0, loopT0 = 0;
    loopCnt++;
    if (LOOP_HZ_DEBUG && (now - loopT0 >= 1000)) {
      Serial.printf("[LOOP] i_out=%lu Hz  v_bus~%lu Hz\n",
                    (unsigned long)loopCnt, (unsigned long)(loopCnt / 10));
      loopCnt = 0; loopT0 = now;
    }

    /* ── Reintento ADS si está offline ── */
    if (!adsOk) {
      if (millis() - adsLastRetry >= ADS_RETRY_MS) {
        adsLastRetry = millis();
        adsOk = sensorsRetry();
        if (adsOk) {
          adsFailCount = 0;
          if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            hwStatus = HW_OK;
            xSemaphoreGive(xMutex);
          }
          Serial.println("[SENS] ADS1115 reconectado");
        }
      }
      allOff();
      vTaskDelay(1);          // rama sin lectura de sensores → ceder CPU al IDLE
      continue;
    }

    /* ── Leer sensores ── */
    if (!sensorsUpdate()) {
      adsFailCount++;
      if (adsFailCount >= ADS_FAIL_THRESH) {
        adsOk = false; adsFailCount = 0; adsLastRetry = millis();
        if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          hwStatus = HW_NO_ADS;
          xSemaphoreGive(xMutex);
        }
        tripFault("ADS1115 desconectado", CAN_FAULT_SENSOR);
      }
      vTaskDelay(1);          // por si el fallo fue antes del yield de adsSingleShot
      continue;
    }
    adsFailCount = 0;

    /* ── Filtro paso bajo de corriente (~20 Hz) — corre CADA ciclo (aun en OFF)
       para que el dashboard muestre la corriente FILTRADA digitalmente. El
       lazo interno también usa iOutFilt. ── */
    {
      const float tau = 1.0f / (2.0f * (float)M_PI * 20.0f);
      float alpha = dt / (tau + dt);
      iOutFilt += alpha * (i_out - iOutFilt);
      i_out_filt = iOutFilt;   // compartido para telemetría/dashboard
    }

    /* ── Protecciones ── */
    if (!checkProtections()) {
      integV = 0.0f; integI = 0.0f;
      continue;
    }

    /* ── Leer estado de control (bajo mutex) ── */
    RunMode rmode;
    bool    fault;
    float   tgtV, kpv, kiv, kpi, kii, kd, irMax, irMin;
    float   vBus, iOut, soc_, period, maxTon, minOff;

    if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) != pdTRUE) continue;
    rmode  = requestedMode;
    fault  = faultLatched;
    tgtV   = targetV;
    kpv    = kpV;  kiv = kiV;
    kpi    = kpI;  kii = kiI;
    kd     = kdDroop;
    irMax  = iRefMax;  irMin = iRefMin;
    vBus   = v_bus;    iOut  = i_out;   soc_ = soc;
    period = (float)pwmPeriodUs;
    maxTon = tonMaxUs;
    minOff = minOffUs;
    xSemaphoreGive(xMutex);

    /* ── Si fault, OFF o desconectado del bus: apagar y resetear integradores ──
       busDisconnected (seguridad): si mi v_bus no coincide con la red asumo que
       no estoy en el bus y no inyecto. Auto-recupera cuando vuelve a coincidir. */
    if (fault || rmode == RUN_OFF || busDisconnected) {
      allOff();
      integV = 0.0f; integI = 0.0f; softStart = 0.0f;
      if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        runMode    = RUN_OFF;
        i_ref      = 0.0f;
        i_ref_v    = 0.0f;
        i_ref_d    = 0.0f;
        dutyCmd    = 0.0f;
        dutyApplied = 0.0f;
        xSemaphoreGive(xMutex);
      }
      continue;
    }

    /* (La corriente ya viene filtrada en iOutFilt, calculado cada ciclo arriba.) */

    /* ── Soft-start: el límite de corriente sube de 0 a pleno en SOFTSTART_S ──
       Evita el golpe de corriente al entrar con el bus ya levantado (hot-plug)
       o en arranque en frío; da tiempo al lazo de voltaje a redistribuir. */
    softStart = (softStart < 1.0f) ? (softStart + dt / SOFTSTART_S) : 1.0f;
    float irMaxEff = irMax * softStart;
    float irMinEff = irMin * softStart;

    /* ── Lazo externo: PI voltaje → base común i_ref_v ──
       El maestro corre este PI y difunde la base por CAN. No se divide entre
       N: cada agente aporta esta misma base, así en pareja reparten por igual
       y en solitario cada uno regula con su dinámica de diseño. */
    float errV  = tgtV - vBus;
    integV     += errV * dt;
    float iRefV = kpv * errV + kiv * integV;
    /* Anti-windup (back-calculation): si iRefV satura contra el límite de
       corriente —o contra el clamp reducido del soft-start— se recalcula integV
       para dejarlo justo en el borde. Sin esto el integrador se dispara mientras
       el bus sube y produce sobreimpulso y oscilación de ciclo límite. */
    if (kiv > 1e-6f) {
      if      (iRefV > irMaxEff) { integV = (irMaxEff - kpv * errV) / kiv; iRefV = irMaxEff; }
      else if (iRefV < irMinEff) { integV = (irMinEff - kpv * errV) / kiv; iRefV = irMinEff; }
    }

    /* ── Droop SoC → i_ref_d ── */
    float iRefD = kd * (soc_ - 0.5f);

    /* ── Referencia de corriente total = base común + droop propio ── */
    float iRef = clampF(iRefV + iRefD, irMinEff, irMaxEff);

    /* ── Selección de topología con histéresis ──
       Permanece en BOOST hasta que i_ref < -100 mA, y en BUCK hasta que
       i_ref > +100 mA. Evita el "chattering" de modo cerca de i_ref = 0. */
    const float TOPO_HYST_A = 0.1f;   // ±100 mA
    TopologyMode topo = prevTopo;
    if (prevTopo == TOPO_BOOST) {
      if (iRef < -TOPO_HYST_A) topo = TOPO_BUCK;
    } else { /* TOPO_BUCK */
      if (iRef >  TOPO_HYST_A) topo = TOPO_BOOST;
    }

    /* Al cambiar de modo, resetear integrador interno para evitar windup */
    if (topo != prevTopo) {
      integI = 0.0f;
      prevTopo = topo;
    }

    /* ── Lazo interno: PI corriente → duty ── */
    float errI  = iRef - iOutFilt;   // usar corriente filtrada
    integI     += errI * dt;
    float duty  = kpi * errI + kii * integI;   // [A·s/A = s]... escala libre

    /* Convertir duty (salida PI) a ton en microsegundos.
       duty está en [0,1] si los PI están bien sintonizados — si no,
       se satura con el clamp de tonMaxUs. */
    float maxAllowed = min(maxTon, period - minOff);
    maxAllowed = max(0.0f, maxAllowed);
    float ton = clampF(fabsf(duty) * period, 0.0f, maxAllowed);

    /* Saturar integrador (anti-windup simple por saturación) */
    float dutyMax = maxAllowed / period;
    integI = clampF(integI, -dutyMax / kii, dutyMax / kii);

    /* ── Aplicar PWM ── */
    if (topo == TOPO_BOOST) {
      driveBoost(ton, period, maxAllowed);
    } else {
      driveBuck(ton, period, maxAllowed);
    }

    float applied = ton / period;

    /* ── Actualizar variables compartidas ── */
    if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      runMode        = RUN_ON;
      i_ref_v        = iRefV;
      i_ref_d        = iRefD;
      i_ref          = iRef;
      dutyCmd        = fabsf(duty);
      dutyApplied    = applied;
      activeTopology = topo;
      xSemaphoreGive(xMutex);
    }
  }
}
