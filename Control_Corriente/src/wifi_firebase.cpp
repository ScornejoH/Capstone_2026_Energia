/*
 * wifi_firebase.cpp
 * ─────────────────────────────────────────────────────────────
 * WiFi, Firebase Auth y REST. Corre en Core 0.
 *
 * Lógica de parámetros:
 *   - La web escribe en /agente1/params los valores deseados
 *   - La ESP32 lee ese nodo cada PARAMS_READ_MS y aplica cambios
 *   - Si el nodo no existe, lo crea con valores por defecto
 *   - Telemetría completa se publica en /agente1/telemetria cada 2 s
 *   - Valores actuales (dashboard) en /agentes/agente1 cada 2 s
 * ─────────────────────────────────────────────────────────────
 */

#include "wifi_firebase.h"
#include "shared.h"
#include "control.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

static WiFiClientSecure secureClient;
static String           auth_token = "";
static uint32_t         token_ts   = 0;

#define PARAMS_READ_MS  5000
#define TELEMETRY_MS     1000
#define HISTORY_MAX     200
#define HISTORY_PRUNE    50
#define HFLOG_MS        2000   // subida del log de alta frecuencia (lote de ~100 muestras)

static uint32_t historyCount = 0;

/* ============================================================
   WIFI
============================================================ */
static const char* wifiStatusStr(wl_status_t s) {
  switch (s) {
    case WL_IDLE_STATUS:     return "IDLE";
    case WL_NO_SSID_AVAIL:  return "SSID no encontrado";
    case WL_CONNECTED:       return "Conectado";
    case WL_CONNECT_FAILED:  return "Fallo (contraseña?)";
    case WL_CONNECTION_LOST: return "Conexión perdida";
    case WL_DISCONNECTED:    return "Desconectado";
    default:                 return "Desconocido";
  }
}

void wifiConnect() {
  Serial.printf("[WiFi] Conectando a '%s'...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500);
    Serial.printf("[WiFi] Intento %d/30 — %s\n", i+1, wifiStatusStr(WiFi.status()));
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("[WiFi] OK — IP: %s\n", WiFi.localIP().toString().c_str());
  else
    Serial.printf("[WiFi] FAIL — %s\n", wifiStatusStr(WiFi.status()));
}

void wifiReconnectIfNeeded() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] Desconectado. Reconectando...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) delay(500);
  }
}

/* ============================================================
   FIREBASE AUTH
============================================================ */
bool firebaseLogin() {
  Serial.print("[AUTH] Autenticando...");
  String url  = "https://identitytoolkit.googleapis.com/v1/accounts"
                ":signInWithPassword?key=" + String(FIREBASE_APIKEY);
  String body = "{\"email\":\""      + String(FIREBASE_EMAIL)   +
                "\",\"password\":\"" + String(FIREBASE_PASS_FB) +
                "\",\"returnSecureToken\":true}";
  HTTPClient http;
  http.begin(secureClient, url);
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  if (code == 200) {
    StaticJsonDocument<2048> doc;
    if (!deserializeJson(doc, http.getString()) && doc["idToken"]) {
      auth_token = doc["idToken"].as<String>();
      token_ts   = millis();
      Serial.printf(" OK (%d chars)\n", auth_token.length());
      http.end(); return true;
    }
  }
  Serial.printf(" FALLO (HTTP %d)\n", code);
  http.end(); return false;
}

void checkTokenRefresh() {
  if (auth_token.length() == 0 || millis() - token_ts > TOKEN_REFRESH_MS)
    firebaseLogin();
}

/* ============================================================
   FIREBASE REST
============================================================ */
static String fbUrl(const char* path) {
  String url = "https://";
  url += FIREBASE_HOST;
  url += path;
  url += ".json";
  if (auth_token.length() > 0) { url += "?auth="; url += auth_token; }
  return url;
}

static String fbUrlQuery(const char* path, const char* query) {
  String url = "https://";
  url += FIREBASE_HOST;
  url += path;
  url += ".json?";
  url += query;
  if (auth_token.length() > 0) { url += "&auth="; url += auth_token; }
  return url;
}

bool fbPatch(const char* path, const String& body) {
  HTTPClient http;
  http.begin(secureClient, fbUrl(path));
  http.addHeader("Content-Type", "application/json");
  int code = http.sendRequest("PATCH", (uint8_t*)body.c_str(), body.length());
  http.end();
  return code == 200;
}

bool fbPut(const char* path, const String& body) {
  HTTPClient http;
  http.begin(secureClient, fbUrl(path));
  http.addHeader("Content-Type", "application/json");
  int code = http.PUT(body);
  http.end();
  return code == 200;
}

bool fbPost(const char* path, const String& body) {
  HTTPClient http;
  http.begin(secureClient, fbUrl(path));
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  http.end();
  return code == 200;
}

String fbGet(const char* path) {
  HTTPClient http;
  http.begin(secureClient, fbUrl(path));
  int    code = http.GET();
  String resp = (code == 200) ? http.getString() : "null";
  http.end();
  return resp;
}

bool fbDelete(const char* path) {
  HTTPClient http;
  http.begin(secureClient, fbUrl(path));
  int code = http.sendRequest("DELETE");
  http.end();
  return code == 200;
}

/* ============================================================
   PARÁMETROS — valores por defecto
============================================================ */
static void writeDefaultParams() {
  StaticJsonDocument<1024> doc;
  doc["run_mode"]   = 0;
  doc["target_v"]   = 5.0;
  doc["omega_v"]  = 62.83;
  doc["omega_i"]  = 314.16;
  doc["plant_vdc"] = 5.0;
  doc["kd_droop"]   = 2.0;
  doc["i_ref_max"]  = 5.0;
  doc["i_ref_min"]  = -5.0;
  doc["ov_trip"]    = 6.5;
  doc["uv_trip"]    = 2.0;
  doc["oc_trip"]    = 5.5;
  doc["acs_offset"]      = 2.5;
  doc["acs_sign"]        = 1;
  doc["acs_sensitivity"] = 0.185;
  doc["faults_enabled"]  = true;
  doc["pwm_period"] = 100;
  doc["ton_max"]    = 95.0;
  doc["min_off"]    = 3.0;
  String body; serializeJson(doc, body);
  fbPut("/agente1/params", body);
  Serial.println("[PARAMS] Nodo creado con valores por defecto");
}

/* ============================================================
   PARÁMETROS — leer y aplicar
============================================================ */
void readParams() {
  String resp = fbGet("/agente1/params");
  if (resp == "null" || resp.isEmpty()) {
    writeDefaultParams();
    return;
  }

  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, resp);
  if (err) {
    Serial.printf("[PARAMS] Error parseando JSON: %s (len=%u)\n", err.c_str(), resp.length());
    return;
  }

  bool gainsChanged = false;

  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(30)) == pdTRUE) {

    if (doc.containsKey("run_mode")) {
      requestedMode = doc["run_mode"].as<int>() ? RUN_ON : RUN_OFF;
    }
    if (doc.containsKey("target_v"))   targetV  = clampF(doc["target_v"],  0.5f, 10.0f);
    if (doc.containsKey("kd_droop"))   kdDroop  = clampF(doc["kd_droop"],  0.0f, 20.0f);
    if (doc.containsKey("i_ref_max"))  iRefMax  = clampF(doc["i_ref_max"], 0.0f, 10.0f);
    if (doc.containsKey("i_ref_min"))  iRefMin  = clampF(doc["i_ref_min"],-10.0f, 0.0f);
    if (doc.containsKey("ov_trip"))    overvoltageTrip  = clampF(doc["ov_trip"],  4.0f, 15.0f);
    if (doc.containsKey("uv_trip"))    undervoltageTrip = clampF(doc["uv_trip"],  0.0f,  4.0f);
    if (doc.containsKey("oc_trip"))    overcurrentTrip  = clampF(doc["oc_trip"],  0.1f, 10.0f);
    if (doc.containsKey("acs_offset"))      acsOffset      = clampF(doc["acs_offset"],      0.0f,  5.0f);
    if (doc.containsKey("acs_sign"))        acsSign        = doc["acs_sign"].as<float>() >= 0 ? 1.0f : -1.0f;
    if (doc.containsKey("acs_sensitivity")) acsSensitivity = clampF(doc["acs_sensitivity"], 0.001f, 1.0f);
    if (doc.containsKey("pwm_period")) pwmPeriodUs      = (uint32_t)clampF(doc["pwm_period"], 50.0f, 1000.0f);
    if (doc.containsKey("ton_max"))    tonMaxUs         = clampF(doc["ton_max"], 0.0f, 99.0f);
    if (doc.containsKey("min_off"))    minOffUs         = clampF(doc["min_off"], 0.0f, 20.0f);

    if (doc.containsKey("omega_v"))   { omegaV   = clampF(doc["omega_v"], 1.0f, 100000.0f); gainsChanged = true; }
    if (doc.containsKey("omega_i"))   { omegaI   = clampF(doc["omega_i"], 1.0f, 100000.0f); gainsChanged = true; }
    if (doc.containsKey("plant_vdc")) { plantVdc = clampF(doc["plant_vdc"], 0.5f, 100.0f);   gainsChanged = true; }

    if (doc.containsKey("faults_enabled")) faultsEnabled = doc["faults_enabled"].as<bool>();

    /* Fault acknowledge */
    if (doc.containsKey("fault_ack") && doc["fault_ack"].as<bool>()) {
      faultLatched  = false;
      faultCode     = FLT_NONE;
      requestedMode = RUN_OFF;
    }

    /* Calibración divisores — se aplica una sola vez y se borra */
    if (doc.containsKey("cal_bus")) {
      float real = doc["cal_bus"];
      if (rawVoltsA0 > 0.05f && real > 0.1f)
        divFactorBus = clampF(real / rawVoltsA0, 0.5f, 10.0f);
    }
    if (doc.containsKey("cal_bat")) {
      float real = doc["cal_bat"];
      if (rawVoltsA1 > 0.05f && real > 0.1f)
        divFactorBat = clampF(real / rawVoltsA1, 0.5f, 10.0f);
    }

    xSemaphoreGive(xMutex);
  }

  /* Recalcular Kp/Ki si cambió L o C */
  if (gainsChanged) recomputeGains();

  /* Limpiar campos de un solo uso */
  {
    String patch = "{";
    bool   need  = false;
    if (doc.containsKey("fault_ack") && doc["fault_ack"].as<bool>()) {
      patch += "\"fault_ack\":false"; need = true;
    }
    if (doc.containsKey("cal_bus")) {
      if (need) patch += ",";
      patch += "\"cal_bus\":null"; need = true;
    }
    if (doc.containsKey("cal_bat")) {
      if (need) patch += ",";
      patch += "\"cal_bat\":null"; need = true;
    }
    patch += "}";
    if (need) fbPatch("/agente1/params", patch);
  }
}

/* ============================================================
   TELEMETRÍA COMPLETA → /agente1/telemetria
============================================================ */
void publishTelemetry() {
  Snapshot s = takeSnapshot();
  const char* rmNames[] = {"OFF", "ON"};
  const char* tpNames[] = {"BUCK", "BOOST"};

  StaticJsonDocument<1280> doc;
  doc["v_bus"]        = serialized(String(s.v_bus,  3));
  doc["v_bat"]        = serialized(String(s.v_bat,  3));
  doc["i_out"]        = serialized(String(s.i_out,  3));
  doc["soc"]          = serialized(String(s.soc,    3));
  doc["i_ref"]        = serialized(String(s.i_ref,  3));
  doc["i_ref_v"]      = serialized(String(s.i_ref_v, 3));
  doc["i_ref_d"]      = serialized(String(s.i_ref_d, 3));
  doc["duty_cmd"]     = serialized(String(s.dutyCmd,     4));
  doc["duty_applied"] = serialized(String(s.dutyApplied, 4));
  doc["topology"]     = tpNames[s.activeTopology];
  doc["run_mode"]     = rmNames[s.runMode];
  doc["req_mode"]     = rmNames[s.requestedMode];
  doc["fault"]          = s.fault;
  doc["faults_enabled"] = s.faultsEnabled;
  {
    const char* fmsg;
    switch (s.faultCode) {
      case FLT_OVERVOLTAGE:  fmsg = "Protección de sobrevoltaje activada";  break;
      case FLT_UNDERVOLTAGE: fmsg = "Protección de subvoltaje activada";    break;
      case FLT_OVERCURRENT:  fmsg = "Protección de sobrecorriente activada"; break;
      case FLT_NO_ADS:       fmsg = "Protección por sensor ADS1115 desconectado"; break;
      default:               fmsg = "";                                     break;
    }
    doc["fault_code"] = s.faultCode;
    doc["fault_msg"]  = fmsg;
  }
  {
    const char* lvl = "ok";
    const char* msg = "";
    if (s.hwStatus == HW_NO_ADS) { lvl = "error"; msg = "ERROR: Sensor ADS1115 desconectado. Control inhibido."; }
    doc["hw_level"] = lvl;
    doc["hw_msg"]   = msg;
  }
  doc["target_v"]     = serialized(String(s.targetV,  2));
  doc["omega_v"]  = serialized(String(s.omegaV, 2));
  doc["omega_i"]  = serialized(String(s.omegaI, 2));
  doc["plant_vdc"] = serialized(String(s.plantVdc, 2));
  doc["kp_v"]     = serialized(String(s.kpV, 4));
  doc["ki_v"]     = serialized(String(s.kiV, 2));
  doc["kp_i"]     = serialized(String(s.kpI, 4));
  doc["ki_i"]     = serialized(String(s.kiI, 2));
  doc["kd_droop"]     = serialized(String(s.kdDroop,  3));
  doc["i_ref_max"]    = serialized(String(s.iRefMax,  2));
  doc["i_ref_min"]    = serialized(String(s.iRefMin,  2));
  doc["ov_trip"]      = serialized(String(s.ovTrip,   2));
  doc["uv_trip"]      = serialized(String(s.uvTrip,   2));
  doc["oc_trip"]      = serialized(String(s.ocTrip,   2));
  doc["pwm_period"]   = serialized(String(s.pwmPeriod, 0));
  doc["ton_max"]      = serialized(String(s.tonMax,   1));
  doc["min_off"]      = serialized(String(s.minOff,   1));
  /* Calibración — valores que muestra la web en la sección de sensores */
  doc["rawVoltsA0"]   = serialized(String(s.rawVoltsA0,  4));  // crudo bus  (A2 físico)
  doc["rawVoltsA1"]   = serialized(String(s.rawVoltsA1,  4));  // crudo bat  (A1 físico)
  doc["rawVoltsA3"]   = serialized(String(s.rawVoltsA3,  4));  // crudo ACS  (A3 físico)
  doc["divFactorBus"] = serialized(String(s.divFactorBus, 4));
  doc["divFactorBat"] = serialized(String(s.divFactorBat, 4));
  doc["acsOffset"]      = serialized(String(s.acsOffset,      4));
  doc["acsSign"]        = serialized(String(s.acsSign,        1));
  doc["acsSensitivity"] = serialized(String(s.acsSensitivity, 5));
  doc["ts"]           = (long)(millis() / 1000);

  String body; serializeJson(doc, body);
  fbPatch("/agente1/telemetria", body);
}

/* ============================================================
   DATOS ACTUALES PARA EL DASHBOARD → /agentes/agente1
============================================================ */
void publishAgente() {
  Snapshot s = takeSnapshot();

  /* Valores actuales */
  {
    StaticJsonDocument<128> doc;
    doc["v_bat"] = serialized(String(s.v_bat, 3));
    doc["v_bus"] = serialized(String(s.v_bus, 3));
    doc["i_out"] = serialized(String(s.i_out, 3));
    String b; serializeJson(doc, b);
    fbPatch("/agentes/agente1", b);
  }

  /* Historial */
  {
    StaticJsonDocument<128> doc;
    doc["v_bat"] = serialized(String(s.v_bat, 3));
    doc["v_bus"] = serialized(String(s.v_bus, 3));
    doc["i_out"] = serialized(String(s.i_out, 3));
    doc["ts"]    = (long)(millis() / 1000);
    String b; serializeJson(doc, b);
    fbPost("/history/agente1", b);
    historyCount++;
  }

  /* Poda de historial */
  if (historyCount > HISTORY_MAX) {
    String url = fbUrlQuery("/history/agente1",
      (String("orderBy=\"$key\"&limitToFirst=") + HISTORY_PRUNE).c_str());
    HTTPClient http;
    http.begin(secureClient, url);
    if (http.GET() == 200) {
      StaticJsonDocument<2048> doc;
      if (!deserializeJson(doc, http.getString())) {
        for (JsonPair kv : doc.as<JsonObject>())
          fbDelete((String("/history/agente1/") + kv.key().c_str()).c_str());
      }
    }
    http.end();
    historyCount -= HISTORY_PRUNE;
  }
}

/* ============================================================
   LOG DE ALTA FRECUENCIA → /loghf/agente1
   Vacía el ring buffer (muestreado a 50 Hz por Core 1) y sube el
   lote completo en una sola petición, en formato columnar compacto:
     { t0, base_ms, n, ms:[...], vbus:[...], vbat:[...],
       iout:[...], soc:[...], iref:[...] }
   Cada lote crea una clave nueva (POST/push). El usuario borra
   /loghf al terminar la sesión (≤ 1 h).
============================================================ */
static LogSample hfBatch[LOG_CAP];

static int appendFloatArray(char* b, int p, int cap, const char* name,
                            const LogSample* s, size_t n, int sel) {
  p += snprintf(b + p, cap - p, ",\"%s\":[", name);
  for (size_t i = 0; i < n && p < cap - 16; i++) {
    float v;
    switch (sel) {
      case 0:  v = s[i].v_bus; break;
      case 1:  v = s[i].v_bat; break;
      case 2:  v = s[i].i_out; break;
      case 3:  v = s[i].soc;   break;
      default: v = s[i].i_ref; break;
    }
    p += snprintf(b + p, cap - p, "%s%.3f", i ? "," : "", v);
  }
  p += snprintf(b + p, cap - p, "]");
  return p;
}

void publishHighFreqLog() {
  size_t n = logDrain(hfBatch, LOG_CAP);
  if (n == 0) return;

  static char b[16384];               // holgado para un drenaje completo (256 muestras)
  const int   cap  = sizeof(b);
  uint32_t    base = hfBatch[0].t_ms;
  int p = 0;

  p += snprintf(b + p, cap - p, "{\"t0\":%ld,\"base_ms\":%lu,\"n\":%u",
                (long)(millis() / 1000), (unsigned long)base, (unsigned)n);

  /* ms relativos al primer sample (uint16 cabe ~65 s) */
  p += snprintf(b + p, cap - p, ",\"ms\":[");
  for (size_t i = 0; i < n && p < cap - 16; i++)
    p += snprintf(b + p, cap - p, "%s%lu", i ? "," : "",
                  (unsigned long)(hfBatch[i].t_ms - base));
  p += snprintf(b + p, cap - p, "]");

  p = appendFloatArray(b, p, cap, "vbus", hfBatch, n, 0);
  p = appendFloatArray(b, p, cap, "vbat", hfBatch, n, 1);
  p = appendFloatArray(b, p, cap, "iout", hfBatch, n, 2);
  p = appendFloatArray(b, p, cap, "soc",  hfBatch, n, 3);
  p = appendFloatArray(b, p, cap, "iref", hfBatch, n, 4);
  p += snprintf(b + p, cap - p, "}");

  HTTPClient http;
  http.begin(secureClient, fbUrl("/loghf/agente1"));
  http.addHeader("Content-Type", "application/json");
  http.POST((uint8_t*)b, p);          // push: cada lote = clave nueva
  http.end();
}

/* ============================================================
   TAREA FREERTOS — Core 0
============================================================ */
void taskComms(void* pvParams) {
  secureClient.setInsecure();
  wifiConnect();

  if (WiFi.status() == WL_CONNECTED) {
    for (int i = 0; i < 5 && auth_token.length() == 0; i++) {
      if (i > 0) delay(3000);
      firebaseLogin();
    }
    if (auth_token.length() == 0)
      Serial.println("[WARN] Sin autenticación Firebase. Continuando offline.");
  }

  if (WiFi.status() == WL_CONNECTED) {
    readParams();
    publishTelemetry();
  }

  uint32_t lastTelMs    = millis();
  uint32_t lastParamsMs = millis();
  uint32_t lastHfMs     = millis();

  for (;;) {
    wifiReconnectIfNeeded();
    if (WiFi.status() == WL_CONNECTED) checkTokenRefresh();

    /* Propagar fault a Firebase */
    if (pendingFaultWrite && WiFi.status() == WL_CONNECTED) {
      pendingFaultWrite = false;
      fbPatch("/agente1/params", "{\"run_mode\":0}");
      publishTelemetry();
    }

    uint32_t now = millis();

    /* Leer parámetros cada 5 s */
    if (WiFi.status() == WL_CONNECTED && now - lastParamsMs >= PARAMS_READ_MS) {
      lastParamsMs = now;
      readParams();
    }

    /* Publicar telemetría y datos de agente cada 2 s */
    if (WiFi.status() == WL_CONNECTED && now - lastTelMs >= TELEMETRY_MS) {
      lastTelMs = now;
      publishTelemetry();
      publishAgente();
    }

    /* Subir el log de alta frecuencia (lote de ~100 muestras) cada 2 s */
    if (WiFi.status() == WL_CONNECTED && now - lastHfMs >= HFLOG_MS) {
      lastHfMs = now;
      publishHighFreqLog();
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
