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
   LOOP — genera y envía datos cada SEND_INTERVAL_MS
   ═══════════════════════════════════════════════ */
void loop() {
    /* ── 1. Generar datos ficticios para los 3 agentes ── */
    float vbat[3], vbus[3], iout[3];

    for (int i = 0; i < 3; i++) {
        generar_datos(t_sim, &vbat[i], &vbus[i], &iout[i], &sims[i]);
    }

   if (ads_ok) {
      /* v_bat se reemplaza con medición real; v_bus e i_out siguen ficticios. */
     leer_vbat_ads(vbat);
   }

    t_sim += SEND_INTERVAL_MS / 1000.0f;

    /* ── 2. Construir JSON con ArduinoJson ── */
    /*
       Estructura enviada:
       {
         "agente1": { "v_bat": 4.02, "v_bus": 4.98, "i_out": 1.45 },
         "agente2": { "v_bat": 3.77, "v_bus": 5.01, "i_out": -0.32 },
         "agente3": { "v_bat": 3.51, "v_bus": 4.96, "i_out": -0.88 }
       }
    */
   StaticJsonDocument<512> doc;

    for (int i = 0; i < 3; i++) {
        JsonObject ag = doc.createNestedObject(sims[i].id);
        ag["v_bat"] = roundf(vbat[i] * 1000.0f) / 1000.0f;  /* 3 decimales */
        ag["v_bus"] = roundf(vbus[i] * 1000.0f) / 1000.0f;
        ag["i_out"] = roundf(iout[i] * 1000.0f) / 1000.0f;
    }

    String body;
    serializeJson(doc, body);

   /* ── 3. Enviar a Firebase ── */
   check_token_refresh();
   bool ok = firebase_patch(String(FIREBASE_PATH), body);

      /* ── 4. Imprimir resultado en consola ── */
   for (int i = 0; i < 3; i++) {
     Serial.printf("  %s |   %.3f   |   %.3f   |   %.3f   | %s\n",
                     sims[i].id, vbat[i], vbus[i], iout[i],
                     ok ? "OK" : "ERROR");
                  }


    if (!ok) {
        Serial.println("  [WARN] Fallo al enviar. Reintentando en el próximo ciclo.");
    }

    /* ── 5. Reconectar WiFi si se cae ── */
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WARN] WiFi desconectado. Reconectando...");
        wifi_connect();
    }

    delay(SEND_INTERVAL_MS);
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

