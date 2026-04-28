/**
 * esp32_firebase_test.ino
 * =====================================================================
 * Código de PRUEBA para el ESP32.
 * Genera datos ficticios para 3 agentes y los envía a Firebase
 * Realtime Database cada 2 segundos via HTTPS REST API.
 *
 * NO usa librería Firebase — solo WiFiClientSecure + HTTPClient.
 * NO hay control real — solo datos simulados para probar el dashboard.
 *
 * Estructura de datos enviada a Firebase:
 *   /agentes/agente1/v_bat, v_bus, i_out
 *   /agentes/agente2/v_bat, v_bus, i_out
 *   /agentes/agente3/v_bat, v_bus, i_out
 *
 * Librería requerida (Library Manager):
 *   - ArduinoJson  (by Benoit Blanchon)
 *
 * WiFiClientSecure + HTTPClient vienen incluidas con el ESP32 package.
 * =====================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

/* ─────────────────────────────────────────────
   CONFIGURACIÓN — rellena estos valores
   ───────────────────────────────────────────── */

/* Tu red WiFi */
#include "secrets.h"   // WIFI_SSID, WIFI_PASS, FIREBASE_EMAIL, FIREBASE_PASS (ver secrets.example.h)

/* Firebase Realtime Database
   Consola Firebase → Realtime Database → URL del proyecto
   Formato: https://TU_PROYECTO-default-rtdb.firebaseio.com  */
#define FIREBASE_HOST  "microred-dc-default-rtdb.firebaseio.com"
#define FIREBASE_PATH  "/agentes.json"

/* Auth: deja vacío si las reglas de la DB son públicas (solo para pruebas).
   En producción usa un token de acceso o reglas de seguridad apropiadas.   */
#define FIREBASE_AUTH  ""   /* ej: "?auth=TU_DATABASE_SECRET"               */

/* Intervalo de envío */
#define SEND_INTERVAL_MS  2000

/* ADS1115 */
#define ADS_ADDR        0x48
#define ADS_SDA_PIN     21
#define ADS_SCL_PIN     22
#define ADS_I2C_HZ      100000
#define ADC_RAW_MAX     4095.0f  /* salida normalizada a 12 bits */
#define ADC_VOLT_MAX    5.0f     /* escala real del voltaje medido */

/* Firebase Auth (rellena para producción) */
#define FIREBASE_APIKEY "AIzaSyDwXigQYn1N0nreSYCJ8h0ta63i1mjIWK8"


/* ─────────────────────────────────────────────
   PARÁMETROS DE SIMULACIÓN
   Cada agente tiene una señal senoidal con
   frecuencia y fase distintas para simular
   comportamientos realistas diferenciados.
   ───────────────────────────────────────────── */
typedef struct {
    const char* id;           /* nombre del nodo en Firebase */
    float  vbat_center;       /* tensión central batería [V]  */
    float  vbat_amp;          /* amplitud oscilación vbat [V] */
    float  vbus_center;       /* tensión central bus [V]      */
    float  vbus_noise;        /* ruido sobre V_bus [V]        */
    float  iout_amp;          /* amplitud corriente [A]       */
    float  iout_offset;       /* offset corriente [A]         */
    float  freq;              /* frecuencia de variación [Hz] */
    float  phase;             /* fase inicial [rad]           */
} AgentSim_t;

/* Tres agentes con comportamientos distintos:
   Agente 1 — SoC alto, inyectando al bus (corriente positiva)
   Agente 2 — SoC medio, alternando inyección y consumo
   Agente 3 — SoC bajo, principalmente consumiendo (cargando batería) */
static const AgentSim_t sims[3] = {
    { "agente1",  4.05f, 0.08f,  5.0f, 0.04f,  1.2f,  0.8f,  0.03f, 0.0f  },
    { "agente2",  3.80f, 0.12f,  5.0f, 0.05f,  1.5f,  0.0f,  0.04f, 2.09f },
    { "agente3",  3.55f, 0.10f,  5.0f, 0.03f,  1.0f, -0.6f,  0.05f, 4.19f }
};

/* ─────────────────────────────────────────────
   GLOBALES
   ───────────────────────────────────────────── */
static WiFiClientSecure  client;
static Adafruit_ADS1115  ads;
static bool              ads_ok = false;
static float             t_sim = 0.0f;   /* tiempo de simulación [s]    */
static String            auth_token = "";
static uint32_t          token_ts = 0;

/* ─────────────────────────────────────────────
   PROTOTIPOS
   ───────────────────────────────────────────── */
static void  wifi_connect(void);
static bool  ads_init(void);
static void  leer_vbat_ads(float* vbat);
static bool  firebase_login(void);
static void  check_token_refresh(void);
static String build_url(const String& path);
static bool  firebase_patch(const String& path, const String& body);
static void  generar_datos(float t, float* vbat, float* vbus,
                            float* iout, const AgentSim_t* ag);
static float ruido(float amp);
/* Control task prototypes (from boost) */
static void controlTask(void* pvParameters);
static void commsTask(void* pvParameters);

/* Gate / control pins */
#define PIN_PWM_HIGH 25
#define PIN_PWM_LOW  26

/* Control parameters (defaults mainly copied from boost.cpp) */
static float voutTarget = 5.00f;
static float voutTrip = 5.65f;
static float voutWarn = 5.30f;

static float vinMinTrip = 3.00f;
static float vinWarn = 3.20f;

static float kp = 5.0f;
static float ki = 0.8f;
static float ffScale = 1.20f;

static float piIntegral = 0.0f;
static float tonCmdUs = 0.0f;

static float tonMinUs = 0.0f;
static float tonMaxUs = 35.0f;
static float tonHardMaxUs = 45.0f;

static float maxTonStepUpUs = 1.0f;
static float maxTonStepDownUs = 2.0f;

static float tonFracAcc = 0.0f;

static const unsigned long CONTROL_PERIOD_MS = 20;

/* Run modes */
enum RunMode {
   MODE_OFF,
   MODE_MANUAL,
   MODE_AUTO_FFPI
};

static RunMode runMode = MODE_OFF;
static bool faultLatched = false;

/* Measurements shared between tasks */
static float vin_ag1 = 0.0f;
static float vout_bus = 0.0f;

/* Manual ton in case of manual mode */
static float manualTonUs = 10.0f;

/* Synchronization for ADS1115 access */
static SemaphoreHandle_t adsMutex = NULL;

/* Utility prototypes for control */
static uint32_t computeDitheredTon(float tonUs);
static void runBoostCycle(float tonUs);
static float computeBoostFeedforwardTon();
static void updateFFPI();
static bool updateVout();
static bool updateVin();
static bool updateAllMeasurements();
static float clampFloat(float x, float xmin, float xmax);
static float slewLimitAsym(float current, float target, float maxUp, float maxDown);
static void tripFault(const char* reason);
static void clearFault();

/* ═══════════════════════════════════════════════
   SETUP
   ═══════════════════════════════════════════════ */
void setup() {
    Serial.begin(115200);
    delay(300);

    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println(  "║  ESP32 Firebase Test — 3 Agentes Fake ║");
    Serial.println(  "╚════════════════════════════════════════╝\n");

    randomSeed(esp_random());

    /* Omitir verificación de certificado SSL (solo para pruebas).
       Debe configurarse antes de cualquier petición HTTPS. */
    client.setInsecure();

    wifi_connect();
      ads_ok = ads_init();
     /* Inicializar autenticación en Firebase (si se configuran credenciales) */
     if (!firebase_login()) {
        Serial.println("[WARN] No se pudo autenticar en Firebase. Se intentará luego.");
     }

    Serial.println("[INFO] Listo. Enviando datos cada "
                   + String(SEND_INTERVAL_MS) + " ms...\n");
    Serial.println("  Agente | V_bat (V) | V_bus (V) | I_out (A) | Modo");
    Serial.println("  -------|-----------|-----------|-----------|------");
}

/* ═══════════════════════════════════════════════
   FreeRTOS tasks
   - controlTask: runs boost control loop on a dedicated core
   - commsTask: generates JSON and sends to Firebase, runs on other core
   ═══════════════════════════════════════════════ */

static void printStatus();

void setup() {
   Serial.begin(115200);
   delay(300);

   Serial.println("\n╔════════════════════════════════════════╗");
   Serial.println("║  ESP32 Firebase Test — 3 Agentes Fake ║");
   Serial.println("╚════════════════════════════════════════╝\n");

   randomSeed(esp_random());

   /* Omitir verificación de certificado SSL (solo para pruebas). */
   client.setInsecure();

   wifi_connect();
   ads_ok = ads_init();

   /* Create mutex for ADS access */
   adsMutex = xSemaphoreCreateMutex();

   /* Initialize authentication (optional) */
   if (!firebase_login()) {
      Serial.println("[WARN] No se pudo autenticar en Firebase. Se intentará luego.");
   }

   /* Configure gate pins for boost control */
   pinMode(PIN_PWM_HIGH, OUTPUT);
   pinMode(PIN_PWM_LOW, OUTPUT);
   digitalWrite(PIN_PWM_HIGH, HIGH); // high-side OFF (as in boost.cpp)
   digitalWrite(PIN_PWM_LOW, LOW);   // low-side OFF

   /* Create tasks: comms on core 0, control on core 1 */
   xTaskCreatePinnedToCore(commsTask, "commsTask", 6 * 1024, NULL, 1, NULL, 0);
   xTaskCreatePinnedToCore(controlTask, "controlTask", 6 * 1024, NULL, 2, NULL, 1);

   Serial.println("[INFO] Tasks created: control@core1, comms@core0");
   Serial.println("[INFO] Listo. Enviando datos cada " + String(SEND_INTERVAL_MS) + " ms...\n");
   Serial.println("  Agente | V_bat (V) | V_bus (V) | I_out (A) | Modo");
   Serial.println("  -------|-----------|-----------|-----------|------");

   /* Delete setup task if desired; keep it lightweight */
}

void loop() {
   delay(1000);
}

/* commsTask: builds JSON and sends to Firebase on a dedicated core */
static void commsTask(void* pvParameters) {
   (void)pvParameters;

   for (;;) {
      /* Generate simulated data */
      float vbat[3], vbus[3], iout[3];

      for (int i = 0; i < 3; i++) {
         generar_datos(t_sim, &vbat[i], &vbus[i], &iout[i], &sims[i]);
      }

      /* If ADS present, read Vbat (shared resource) */
      if (ads_ok) {
         if (adsMutex && xSemaphoreTake(adsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            leer_vbat_ads(vbat);
            xSemaphoreGive(adsMutex);
         }
      }

      t_sim += SEND_INTERVAL_MS / 1000.0f;

      /* Build JSON */
      StaticJsonDocument<512> doc;
      for (int i = 0; i < 3; i++) {
         JsonObject ag = doc.createNestedObject(sims[i].id);
         ag["v_bat"] = roundf(vbat[i] * 1000.0f) / 1000.0f;
         ag["v_bus"] = roundf(vbus[i] * 1000.0f) / 1000.0f;
         ag["i_out"] = roundf(iout[i] * 1000.0f) / 1000.0f;
      }

      String body;
      serializeJson(doc, body);

      check_token_refresh();
      bool ok = firebase_patch(String(FIREBASE_PATH), body);

      for (int i = 0; i < 3; i++) {
         Serial.printf("  %s |   %.3f   |   %.3f   |   %.3f   | %s\n",
                    sims[i].id, vbat[i], vbus[i], iout[i],
                    ok ? "OK" : "ERROR");
      }

      if (!ok) {
         Serial.println("  [WARN] Fallo al enviar. Reintentando en el próximo ciclo.");
      }

      if (WiFi.status() != WL_CONNECTED) {
         Serial.println("[WARN] WiFi desconectado. Reconectando...");
         wifi_connect();
      }

      vTaskDelay(pdMS_TO_TICKS(SEND_INTERVAL_MS));
   }
}

/* controlTask: implements the boost control loop using ADS1115 via ads object */
static void controlTask(void* pvParameters) {
   (void)pvParameters;
   unsigned long lastControlMs = millis();

   for (;;) {
      unsigned long nowMs = millis();
      if (nowMs - lastControlMs >= CONTROL_PERIOD_MS) {
         lastControlMs = nowMs;

         bool okVout = updateVout();
         static unsigned long lastVinReadMs = 0;
         if (nowMs - lastVinReadMs >= 200) {
            lastVinReadMs = nowMs;
            updateVin();
         }

         if (okVout) {
            if (vout_bus > voutTrip) {
               tripFault("VOUT sobre limite");
            }

            if (vin_ag1 < vinMinTrip) {
               tripFault("VIN bajo limite");
            }

            if (runMode == MODE_AUTO_FFPI && !faultLatched) {
               updateFFPI();
            }
         }
         else {
            Serial.println("Lectura VOUT fallida");
         }

         /* Actuate depending on mode */
         if (runMode == MODE_OFF || faultLatched) {
            digitalWrite(PIN_PWM_HIGH, HIGH);
            digitalWrite(PIN_PWM_LOW, LOW);
         }
         else if (runMode == MODE_MANUAL) {
            runBoostCycle(manualTonUs);
         }
         else if (runMode == MODE_AUTO_FFPI) {
            runBoostCycle(tonCmdUs);
         }
      }

      /* Print periodic status */
      static unsigned long lastPrintMs = 0;
      if (millis() - lastPrintMs >= 1000) {
         lastPrintMs = millis();
         printStatus();
      }

      vTaskDelay(pdMS_TO_TICKS(5));
   }
}

/* ------------------ control helper implementations ------------------ */
static float clampFloat(float x, float xmin, float xmax) {
   if (x < xmin) return xmin;
   if (x > xmax) return xmax;
   return x;
}

static float slewLimitAsym(float current, float target, float maxUp, float maxDown) {
   if (target > current + maxUp) return current + maxUp;
   if (target < current - maxDown) return current - maxDown;
   return target;
}

uint32_t computeDitheredTon(float tonUs) {
   tonUs = clampFloat(tonUs, tonMinUs, tonMaxUs);
   if (tonUs <= 0.0f) { tonFracAcc = 0.0f; return 0; }
   float baseF = floorf(tonUs);
   float frac = tonUs - baseF;
   uint32_t tonInt = (uint32_t)baseF;
   tonFracAcc += frac;
   if (tonFracAcc >= 1.0f) { tonInt += 1; tonFracAcc -= 1.0f; }
   if (tonInt > (uint32_t)tonMaxUs) tonInt = (uint32_t)tonMaxUs;
   return tonInt;
}

void runBoostCycle(float tonUs) {
   const uint32_t T_PERIOD_US = 100;
   digitalWrite(PIN_PWM_HIGH, HIGH); // highOff
   if (tonUs <= 0.0f) {
      digitalWrite(PIN_PWM_LOW, LOW); // lowOff
      delayMicroseconds(T_PERIOD_US);
      return;
   }
   uint32_t tonInt = computeDitheredTon(tonUs);
   if (tonInt == 0) { digitalWrite(PIN_PWM_LOW, LOW); delayMicroseconds(T_PERIOD_US); return; }
   digitalWrite(PIN_PWM_LOW, HIGH); // lowOn
   delayMicroseconds(tonInt);
   digitalWrite(PIN_PWM_LOW, LOW); // lowOff
   if (tonInt < T_PERIOD_US) delayMicroseconds(T_PERIOD_US - tonInt);
}

float computeBoostFeedforwardTon() {
   if (vin_ag1 <= 0.1f || voutTarget <= vin_ag1) return 0.0f;
   float dutyIdeal = 1.0f - (vin_ag1 / voutTarget);
   dutyIdeal = clampFloat(dutyIdeal, 0.0f, 0.85f);
   float tonIdeal = dutyIdeal * 100.0f;
   return ffScale * tonIdeal;
}

void updateFFPI() {
   if (vout_bus > voutTrip) { tripFault("VOUT sobre limite"); return; }
   if (vin_ag1 < vinMinTrip) { tripFault("VIN bajo limite"); return; }
   float dt = CONTROL_PERIOD_MS / 1000.0f;
   float error = voutTarget - vout_bus;
   if (error <= -0.08f) { piIntegral *= 0.5f; tonCmdUs = slewLimitAsym(tonCmdUs, 0.0f, maxTonStepUpUs, maxTonStepDownUs); return; }
   if (error <= 0.00f) { piIntegral *= 0.8f; tonCmdUs = slewLimitAsym(tonCmdUs, 0.0f, maxTonStepUpUs, maxTonStepDownUs); return; }
   piIntegral += error * dt;
   piIntegral = clampFloat(piIntegral, -4.0f, 4.0f);
   float tonFF = computeBoostFeedforwardTon();
   float tonPI = kp * error + ki * piIntegral;
   float tonTarget = tonFF + tonPI;
   if (error > 1.00f) { tonTarget = fmaxf(tonTarget, 28.0f); }
   else if (error > 0.70f) { tonTarget = fmaxf(tonTarget, 24.0f); }
   else if (error > 0.45f) { tonTarget = fmaxf(tonTarget, 20.0f); }
   tonTarget = clampFloat(tonTarget, 0.0f, tonMaxUs);
   tonCmdUs = slewLimitAsym(tonCmdUs, tonTarget, maxTonStepUpUs, maxTonStepDownUs);
}

/* ADS1115-based measurement helpers using existing 'ads' object */
bool updateVout() {
   if (!adsMutex || xSemaphoreTake(adsMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
      return false;
   }
   int16_t raw = ads.readADC_SingleEnded(0);
   float a0 = ads.computeVolts(raw);
   xSemaphoreGive(adsMutex);
   if (isnan(a0)) return false;
   const float DIV_FACTOR_A0 = 3.12f; // from boost
   vout_bus = a0 * DIV_FACTOR_A0;
   return true;
}

bool updateVin() {
   if (!adsMutex || xSemaphoreTake(adsMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
      return false;
   }
   int16_t raw = ads.readADC_SingleEnded(1);
   float a1 = ads.computeVolts(raw);
   xSemaphoreGive(adsMutex);
   if (isnan(a1)) return false;
   const float DIV_FACTOR_A1 = 3.10f;
   vin_ag1 = a1 * DIV_FACTOR_A1;
   return true;
}

bool updateAllMeasurements() {
   bool ok0 = updateVout();
   bool ok1 = updateVin();
   return ok0 && ok1;
}

void tripFault(const char* reason) {
   faultLatched = true;
   runMode = MODE_OFF;
   tonFracAcc = 0.0f;
   digitalWrite(PIN_PWM_HIGH, HIGH);
   digitalWrite(PIN_PWM_LOW, LOW);
   Serial.print("FAULT: ");
   Serial.println(reason);
}

void clearFault() {
   faultLatched = false;
   runMode = MODE_OFF;
   piIntegral = 0.0f;
   tonCmdUs = 0.0f;
   tonFracAcc = 0.0f;
   digitalWrite(PIN_PWM_HIGH, HIGH);
   digitalWrite(PIN_PWM_LOW, LOW);
   Serial.println("Fault limpiada. Estado: OFF");
}

void printStatus() {
   Serial.println();
   Serial.print("RunMode = ");
   if (runMode == MODE_OFF) Serial.print("OFF");
   else if (runMode == MODE_MANUAL) Serial.print("MANUAL");
   else if (runMode == MODE_AUTO_FFPI) Serial.print("AUTO_FFPI");
   Serial.print(" | Fault = ");
   Serial.print(faultLatched ? "SI" : "NO");
   Serial.print(" | VIN_AG1 = ");
   Serial.print(vin_ag1, 3);
   Serial.print(" V");
   Serial.print(" | VOUT_BUS = ");
   Serial.print(vout_bus, 3);
   Serial.print(" V");
   Serial.print(" | Target = ");
   Serial.print(voutTarget, 2);
   Serial.print(" V");
   Serial.print(" | Ton = ");
   Serial.print(tonCmdUs, 3);
   Serial.print(" us");
   Serial.print(" | TonMax = ");
   Serial.print(tonMaxUs, 2);
   Serial.print(" us");
   Serial.print(" | Kp = ");
   Serial.print(kp, 2);
   Serial.print(" | Ki = ");
   Serial.print(ki, 2);
   Serial.print(" | ffScale = ");
   Serial.println(ffScale, 2);
}

/* ═══════════════════════════════════════════════
   GENERACIÓN DE DATOS FICTICIOS
   ═══════════════════════════════════════════════ */

/**
 * Genera valores simulados para un agente en el instante t.
 * v_bat : varía lentamente simulando descarga/carga de batería
 * v_bus : oscila cerca de 5 V con pequeño ruido
 * i_out : señal senoidal con offset para simular boost o buck
 */
static void generar_datos(float t, float* vbat, float* vbus,
                           float* iout, const AgentSim_t* ag) {
    /* Corriente: senoidal + offset + ruido pequeño */
    *iout = ag->iout_amp * sinf(2.0f * M_PI * ag->freq * t + ag->phase)
            + ag->iout_offset
            + ruido(0.05f);

    /* Tensión batería: varía lentamente con la corriente
       Si i_out > 0 (inyecta) → batería se descarga → v_bat baja
       Si i_out < 0 (consume) → batería se carga   → v_bat sube */
    float bat_drift = -(*iout) * 0.0002f;   /* deriva muy lenta */
    *vbat = ag->vbat_center
            + ag->vbat_amp * sinf(2.0f * M_PI * ag->freq * 0.1f * t + ag->phase)
            + bat_drift
            + ruido(0.005f);

    /* Clamp físico: batería Li-ion entre 3.0 V y 4.2 V */
    *vbat = constrain(*vbat, 3.00f, 4.20f);

    /* Tensión bus: cercana a 5 V con pequeño rizo */
    *vbus = ag->vbus_center
            + 0.05f * sinf(2.0f * M_PI * 0.5f * t)
            + ruido(ag->vbus_noise);
    *vbus = constrain(*vbus, 4.50f, 5.50f);
}

/**
 * Ruido aleatorio uniforme en ±amp.
 */
static float ruido(float amp) {
    return amp * (2.0f * (float)random(1000) / 1000.0f - 1.0f);
}

/* ═══════════════════════════════════════════════
   ADS1115
   ═══════════════════════════════════════════════ */
static bool ads_init() {
   Wire.begin(ADS_SDA_PIN, ADS_SCL_PIN);
   Wire.setClock(ADS_I2C_HZ);

   ads.setGain(GAIN_ONE);

   if (!ads.begin(ADS_ADDR)) {
      Serial.println("[WARN] ADS1115 no detectado. v_bat seguirá en modo ficticio.");
      return false;
   }

   Serial.println("[INFO] ADS1115 detectado. v_bat en modo medicion real.");
   return true;
}

static void leer_vbat_ads(float* vbat) {
   for (int ch = 0; ch < 3; ch++) {
      int16_t raw16 = ads.readADC_SingleEnded(ch);
      if (raw16 < 0) {
         raw16 = 0;
      }

      

      vbat[ch] = ads.computeVolts(raw16);
   }
}

/* ═══════════════════════════════════════════════
   FIREBASE — HTTPS PATCH
   Actualiza el nodo /agentes con los datos de los 3 agentes.
   Usa PATCH para actualizar solo los campos enviados
   sin borrar otros nodos que pueda haber en la DB.
   ═══════════════════════════════════════════════ */
/* Construye URL completa para peticiones REST incluyendo token si existe */
static String build_url(const String& path) {
   return "https://" + String(FIREBASE_HOST) + path +
         (auth_token.length() > 0 ? "?auth=" + auth_token : "");
}

/* Inicia sesión en Firebase Authentication con email/password.
   Obtiene idToken y lo guarda en `auth_token`. */
static bool firebase_login(void) {
    Serial.print("[AUTH] Autenticando ESP32 en Firebase...");

    String url = "https://identitytoolkit.googleapis.com/v1/accounts"
                 ":signInWithPassword?key=" + String(FIREBASE_APIKEY);

    String body = "{\"email\":\"" + String(FIREBASE_EMAIL) +
                  "\",\"password\":\"" + String(FIREBASE_PASS) +
                  "\",\"returnSecureToken\":true}";

    HTTPClient http;
    http.begin(client, url);
    http.addHeader("Content-Type", "application/json");
    int code = http.POST(body);

   if (code == 200) {
      String response = http.getString();
      StaticJsonDocument<2048> doc;
      DeserializationError err = deserializeJson(doc, response);

      if (!err && doc["idToken"]) {
         auth_token = doc["idToken"].as<String>();
         token_ts   = millis();
         Serial.println(" OK (token: " + String(auth_token.length()) + " chars)");
         http.end();
         return true;
      }

      Serial.println(" FALLO — idToken no encontrado en respuesta");
      Serial.println("  Error JSON: " + String(err.c_str()));
      Serial.println("  Primeros 200 chars: " + response.substring(0, 200));
      http.end();
      return false;
   }

    Serial.printf(" FALLO (HTTP %d)\n", code);
    Serial.println("  Respuesta: " + http.getString().substring(0, 200));
    http.end();
    return false;
}

/* Renovar token si es necesario (llamar al inicio del loop) */
static void check_token_refresh(void) {
   const uint32_t TOKEN_REFRESH_MS = 2700000; /* 45 min */
   if (auth_token.length() == 0 || millis() - token_ts > TOKEN_REFRESH_MS) {
      Serial.println("[AUTH] Renovando token...");
      firebase_login();
   }
}

/* PATCH usando token (construye URL con build_url) */
static bool firebase_patch(const String& path, const String& body) {
   HTTPClient http;
   http.begin(client, build_url(path));
   http.addHeader("Content-Type", "application/json");
   http.addHeader("X-HTTP-Method-Override", "PATCH");
   int code = http.sendRequest("PATCH", body);
   bool ok = (code == 200);
   if (!ok) {
      Serial.printf("  [HTTP] Error %d: %s\n", code, http.getString().c_str());
   }
   http.end();
   return ok;
}

/* ═══════════════════════════════════════════════
   WIFI
   ═══════════════════════════════════════════════ */
static void wifi_connect() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("[WIFI] Conectando a %s", WIFI_SSID);

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
        Serial.print('.');
        delay(500);
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WIFI] OK — IP: %s\n\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\n[ERROR] No pudo conectar al WiFi. Verifica SSID y contraseña.");
        Serial.println("        El programa continuará intentando en cada ciclo.");
    }
}

