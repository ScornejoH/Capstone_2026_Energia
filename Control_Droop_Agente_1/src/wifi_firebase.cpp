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
#include "can_bus.h"

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
  doc["omega_v"]  = 10.0;
  doc["omega_i"]  = 100.0;
  doc["zeta_v"]   = 0.707;
  doc["zeta_i"]   = 0.707;
  doc["plant_vdc"] = 5.0;
  doc["kd_droop"]   = 0.1;
  doc["i_ref_max"]  = 1.0;
  doc["i_ref_min"]  = -1.0;
  doc["ov_trip"]    = 7.5;
  doc["uv_trip"]    = 0.0;
  doc["oc_trip"]    = 1.5;
  doc["acs_offset"]      = 2.5;
  doc["acs_sign"]        = -1;
  doc["acs_sensitivity"] = 0.100;   // ACS712-20A por defecto (5A = 0.185)
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
    if (doc.containsKey("target_v"))   targetV  = clampF(doc["target_v"],  3.0f, 6.0f);
    if (doc.containsKey("kd_droop"))   kdDroop  = clampF(doc["kd_droop"],  0.0f, 20.0f);
    if (doc.containsKey("i_ref_max"))  iRefMax  = clampF(doc["i_ref_max"], 0.0f, 5.0f);
    if (doc.containsKey("i_ref_min"))  iRefMin  = clampF(doc["i_ref_min"],-5.0f, 0.0f);
    if (doc.containsKey("ov_trip"))    overvoltageTrip  = clampF(doc["ov_trip"],  4.0f, 8.0f);
    if (doc.containsKey("uv_trip"))    undervoltageTrip = clampF(doc["uv_trip"],  0.0f,  4.0f);
    if (doc.containsKey("oc_trip"))    overcurrentTrip  = clampF(doc["oc_trip"],  0.1f, 5.0f);
    if (doc.containsKey("acs_offset"))      acsOffset      = clampF(doc["acs_offset"],      0.0f,  5.0f);
    if (doc.containsKey("acs_sign"))        acsSign        = doc["acs_sign"].as<float>() >= 0 ? 1.0f : -1.0f;
    if (doc.containsKey("acs_sensitivity")) acsSensitivity = clampF(doc["acs_sensitivity"], 0.001f, 1.0f);
    if (doc.containsKey("pwm_period")) pwmPeriodUs      = (uint32_t)clampF(doc["pwm_period"], 50.0f, 1000.0f);
    if (doc.containsKey("ton_max"))    tonMaxUs         = clampF(doc["ton_max"], 0.0f, 99.0f);
    if (doc.containsKey("min_off"))    minOffUs         = clampF(doc["min_off"], 0.0f, 20.0f);

    if (doc.containsKey("omega_v"))   { omegaV   = clampF(doc["omega_v"], 1.0f, 100000.0f); gainsChanged = true; }
    if (doc.containsKey("omega_i"))   { omegaI   = clampF(doc["omega_i"], 1.0f, 100000.0f); gainsChanged = true; }
    if (doc.containsKey("zeta_v"))    { zetaV    = clampF(doc["zeta_v"], 0.1f, 5.0f);       gainsChanged = true; }
    if (doc.containsKey("zeta_i"))    { zetaI    = clampF(doc["zeta_i"], 0.1f, 5.0f);       gainsChanged = true; }
    if (doc.containsKey("plant_vdc")) { plantVdc = clampF(doc["plant_vdc"], 0.5f, 100.0f);   gainsChanged = true; }

    if (doc.containsKey("faults_enabled")) faultsEnabled = doc["faults_enabled"].as<bool>();

    /* Seguridad de conexión al bus (propia del gateway). */
    if (doc.containsKey("bus_chk_en"))  busChkEnable = doc["bus_chk_en"].as<bool>();
    if (doc.containsKey("bus_chk_tol")) busChkTol    = clampF(doc["bus_chk_tol"], 0.02f, 5.0f);

    /* Fault acknowledge */
    if (doc.containsKey("fault_ack") && doc["fault_ack"].as<bool>()) {
      faultLatched  = false;
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
    /* Cero del ACS con la corriente FILTRADA: corrige el offset para que
       i_out_filt→0 (robusto a ruido) ≡ offset = lectura cruda filtrada. */
    if (doc.containsKey("acs_zero") && doc["acs_zero"].as<bool>()) {
      acsOffset = clampF(acsOffset + acsSign * acsSensitivity * i_out_filt, 0.0f, 5.0f);
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
    if (doc.containsKey("acs_zero") && doc["acs_zero"].as<bool>()) {
      if (need) patch += ",";
      patch += "\"acs_zero\":false"; need = true;
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
  doc["i_out"]        = serialized(String(s.i_out_filt,  3));
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
  doc["fault_code"]     = s.faultCode;   // CAN_FAULT_* (1=OV 2=UV 3=OC 4=sensor)
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
  doc["zeta_v"]   = serialized(String(s.zetaV, 3));
  doc["zeta_i"]   = serialized(String(s.zetaI, 3));
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

  /* Valores actuales — MISMO esquema que /agentes/agente2 y agente3
     (v_bus, v_bat, i_out, soc, e_deliv, e_absorb, online, run_mode, topology,
     fault, fault_code, ts) para que el dashboard lea los 3 de forma uniforme. */
  {
    StaticJsonDocument<320> doc;
    doc["v_bat"]      = serialized(String(s.v_bat, 3));
    doc["v_bus"]      = serialized(String(s.v_bus, 3));
    doc["i_out"]      = serialized(String(s.i_out_filt, 3));
    doc["soc"]        = serialized(String(s.soc, 3));
    doc["e_deliv"]    = serialized(String(s.e_deliv, 4));
    doc["e_absorb"]   = serialized(String(s.e_absorb, 4));
    doc["online"]     = true;                       // el gateway es local: si publica, está en línea
    doc["age"]        = 0;                           // dato local siempre fresco (s desde el último refresco)
    doc["bus_disc"]   = busDisconnected;             // seguridad: desconectado del bus
    doc["run_mode"]   = (s.runMode == 1) ? "ON" : "OFF";
    doc["topology"]   = (s.activeTopology == 1) ? "BOOST" : "BUCK";
    doc["fault"]      = s.fault;
    doc["fault_code"] = s.faultCode;
    doc["ts"]         = (long)(millis() / 1000);
    String b; serializeJson(doc, b);
    fbPatch("/agentes/agente1", b);
  }

  /* Historial */
  {
    StaticJsonDocument<128> doc;
    doc["v_bat"] = serialized(String(s.v_bat, 3));
    doc["v_bus"] = serialized(String(s.v_bus, 3));
    doc["i_out"] = serialized(String(s.i_out_filt, 3));
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
   GATEWAY CAN → WiFi
   Publica en Firebase la telemetría recibida por CAN de los
   agentes 2 y 3.
============================================================ */
static void publishRelayedAgents() {
  for (uint8_t agent = 2; agent <= 2; agent++) {
    CanAgentData d;
    if (!canGetAgentData(agent, &d)) continue;

    StaticJsonDocument<320> doc;
    doc["v_bat"]      = serialized(String(d.v_bat, 3));
    doc["v_bus"]      = serialized(String(d.v_bus, 3));
    doc["i_out"]      = serialized(String(d.i_out, 3));
    doc["soc"]        = serialized(String(d.soc,   3));
    doc["e_deliv"]    = serialized(String(d.e_deliv,  4));
    doc["e_absorb"]   = serialized(String(d.e_absorb, 4));
    doc["online"]     = d.online;
    /* Edad del dato: segundos desde el último frame CAN recibido de este
       agente (-1 = nunca visto). El dashboard la usa para mostrar "hace Xs". */
    doc["age"]        = d.lastRxMs ? (int)((millis() - d.lastRxMs) / 1000) : -1;
    doc["bus_disc"]   = d.busDisc;                   // seguridad: desconectado del bus
    doc["run_mode"]   = d.running ? "ON" : "OFF";
    doc["topology"]   = d.topo ? "BOOST" : "BUCK";
    doc["fault"]      = d.fault;
    doc["fault_code"] = d.faultCode;
    doc["ts"]         = (long)(millis() / 1000);

    String body; serializeJson(doc, body);
    String path = String("/agentes/agente") + (int)agent;
    fbPatch(path.c_str(), body);
  }
}

/* Telemetría EXTENDIDA del agente 3 → /agentes/agente3 */
static void publishAgent3() {
  CanAgent3Data d;
  if (!canGetAgent3(&d)) return;

  StaticJsonDocument<640> doc;
  doc["v_bus"]      = serialized(String(d.v_bus5, 3));   // bus compartido
  doc["v_bat"]      = serialized(String(d.v_bat2s, 3));  // batería 2S
  doc["v_mid"]      = serialized(String(d.v_mid, 3));    // punto medio 2S (balance de celdas)
  doc["v_panel"]    = serialized(String(d.v_panel, 3));
  doc["i_out"]      = serialized(String(d.i_bb, 3));      // corriente buckboost (bus)
  doc["i_panel"]    = serialized(String(d.i_panel, 3));
  doc["i_bat"]      = serialized(String(d.i_bat, 3));
  doc["p_panel"]    = serialized(String(d.p_panel, 3));
  doc["soc"]        = serialized(String(d.soc, 3));
  doc["e_deliv"]    = serialized(String(d.e_deliv,  4));
  doc["e_absorb"]   = serialized(String(d.e_absorb, 4));
  doc["ton_cmd"]    = serialized(String(d.ton_cmd, 2));
  doc["pv_duty"]    = serialized(String(d.pv_duty, 4));
  doc["pv_vpv_ref"] = serialized(String(d.pv_vpv_ref, 3));
  doc["online"]     = d.online;
  doc["age"]        = d.lastRxMs ? (int)((millis() - d.lastRxMs) / 1000) : -1;
  doc["bus_disc"]   = d.busDisc;                   // seguridad: buckboost desconectado del bus
  doc["run_mode"]   = d.bb_run ? "ON" : "OFF";
  doc["topology"]   = d.topo ? "BOOST" : "BUCK";
  doc["pv_armed"]   = d.pv_armed;
  doc["pv_mode"]    = d.pv_mode;
  doc["fault"]      = d.bb_fault;
  doc["fault_code"] = d.bb_fault_code;
  doc["pv_fault"]      = d.pv_fault;
  doc["pv_fault_code"] = d.pv_fault_code;
  doc["ts"]         = (long)(millis() / 1000);

  String body; serializeJson(doc, body);
  fbPatch("/agentes/agente3", body);
}

/* Consenso distribuido del voltaje de bus (opción B): publica la estimación de
   cada agente (todas convergen al mismo valor), el promedio de red y la
   dispersión máxima entre estimaciones frescas. */
static void publishConsensus() {
  float v[4]; bool fr[4] = {false,false,false,false};
  for (uint8_t a = 1; a <= 3; a++) fr[a] = canGetConsensus(a, &v[a]);

  StaticJsonDocument<256> doc;
  float sum = 0.0f, mn = 1e9f, mx = -1e9f; int n = 0;
  for (uint8_t a = 1; a <= 3; a++) {
    if (!fr[a]) continue;
    doc[String("a") + a] = serialized(String(v[a], 3));
    sum += v[a]; if (v[a] < mn) mn = v[a]; if (v[a] > mx) mx = v[a]; n++;
  }
  if (n > 0) {
    doc["v"]      = serialized(String(sum / n, 3));   // valor de red (promedio de estimaciones)
    doc["spread"] = serialized(String(mx - mn, 3));   // desacuerdo residual entre estimaciones
  }
  doc["n"]  = n;
  doc["ts"] = (long)(millis() / 1000);
  String body; serializeJson(doc, body);
  fbPatch("/consenso", body);
}

/* ============================================================
   REPORTE DEL GATEWAY → /sistema
   El agente 1 declara, desde SU punto de vista (recepción CAN), qué agentes
   están conectados al bus, y publica un heartbeat propio. Así el dashboard
   distingue "se cayó un agente" (lo dice AG1) de "se cayó el gateway" (deja
   de avanzar el heartbeat). Es la fuente autoritativa de conectividad.
============================================================ */
static uint32_t gwHeartbeat = 0;
static void publishSystem() {
  CanAgentData  d2;  bool has2 = canGetAgentData(2, &d2);
  CanAgent3Data d3;  bool has3 = canGetAgent3(&d3);
  uint32_t now = millis();

  bool on2  = has2 && d2.online;
  bool on3  = has3 && d3.online;
  int  age2 = (has2 && d2.lastRxMs) ? (int)((now - d2.lastRxMs) / 1000) : -1;
  int  age3 = (has3 && d3.lastRxMs) ? (int)((now - d3.lastRxMs) / 1000) : -1;

  /* Lista legible de agentes que AG1 ve desconectados: "" | "2" | "3" | "2,3" */
  String offline = "";
  if (!on2) offline += offline.length() ? ",2" : "2";
  if (!on3) offline += offline.length() ? ",3" : "3";

  StaticJsonDocument<384> doc;
  doc["gw_online"] = true;
  doc["hb"]        = gwHeartbeat++;      // avanza en cada publicación → vida del gateway
  doc["offline"]   = offline;
  {
    JsonObject a2 = doc.createNestedObject("agente2");
    a2["online"] = on2;  a2["age"] = age2;
    JsonObject a3 = doc.createNestedObject("agente3");
    a3["online"] = on3;  a3["age"] = age3;
  }
  doc["ts"] = (long)(now / 1000);
  String body; serializeJson(doc, body);
  fbPatch("/sistema", body);
}

/* ============================================================
   GATEWAY WiFi → CAN
   Lee /agenteN/params (lo escribe la web) y, ante un cambio,
   retransmite el parámetro por CAN al agente 2 ó 3.
   Mapa clave Firebase → índice de parámetro CAN (CFG_*).
============================================================ */
struct CfgKey { const char* key; uint8_t idx; };
static const CfgKey CFG_KEYS[] = {
  {"target_v",  CFG_TARGET_V},  {"omega_v",   CFG_OMEGA_V},
  {"omega_i",   CFG_OMEGA_I},   {"plant_vdc", CFG_PLANT_VDC},
  {"kd_droop",  CFG_KD_DROOP},  {"i_ref_max", CFG_IREF_MAX},
  {"i_ref_min", CFG_IREF_MIN},  {"ov_trip",   CFG_OV_TRIP},
  {"uv_trip",   CFG_UV_TRIP},   {"oc_trip",   CFG_OC_TRIP},
  {"run_mode",  CFG_RUN_MODE},  {"acs_sens_signed", CFG_ACS_SENS},
};
static const int CFG_KEYS_LEN = sizeof(CFG_KEYS) / sizeof(CFG_KEYS[0]);

/* Cache del último valor enviado por [agente 2/3][índice] para enviar
   solo ante cambios. NAN = aún no enviado. */
static float cfgCache[2][16];
static bool  cfgCacheInit = false;

static void relayConfigToAgents() {
#if CAN_RELAY_CONFIG
  if (!cfgCacheInit) {
    for (int a = 0; a < 2; a++)
      for (int i = 0; i < 16; i++) cfgCache[a][i] = NAN;
    cfgCacheInit = true;
  }

  for (uint8_t agent = 2; agent <= 2; agent++) {
    String path = String("/agente") + (int)agent + "/params";
    String resp = fbGet(path.c_str());
    if (resp == "null" || resp.isEmpty()) continue;

    StaticJsonDocument<768> doc;
    if (deserializeJson(doc, resp)) continue;

    int ai = agent - 2;
    for (int k = 0; k < CFG_KEYS_LEN; k++) {
      if (!doc.containsKey(CFG_KEYS[k].key)) continue;
      float v   = doc[CFG_KEYS[k].key].as<float>();
      uint8_t i = CFG_KEYS[k].idx;
      if (isnan(cfgCache[ai][i]) || cfgCache[ai][i] != v) {
        if (canQueueConfig(agent, i, v)) cfgCache[ai][i] = v;
      }
    }

    /* fault_ack: one-shot. Se retransmite por CAN y se limpia en Firebase
       para que un nuevo ack pueda volver a dispararse. */
    if (doc.containsKey("fault_ack") && doc["fault_ack"].as<bool>()) {
      if (canQueueConfig(agent, CFG_FAULT_ACK, 1.0f)) {
        String p = String("/agente") + (int)agent + "/params";
        fbPatch(p.c_str(), "{\"fault_ack\":false}");
      }
    }

    /* Calibración one-shot: la presencia de la clave dispara el comando;
       se releva por CAN y se borra de Firebase para permitir re-disparo. */
    struct OneShot { const char* key; uint8_t idx; };
    static const OneShot ONESHOTS[] = {
      {"cal_bus", CFG_CAL_BUS}, {"cal_bat", CFG_CAL_BAT}, {"acs_zero", CFG_ACS_ZERO},
    };
    for (auto& os : ONESHOTS) {
      if (doc.containsKey(os.key)) {
        float v = doc[os.key].as<float>();
        if (canQueueConfig(agent, os.idx, v)) {
          String p = String("/agente") + (int)agent + "/params";
          fbPatch(p.c_str(), (String("{\"") + os.key + "\":null}").c_str());
        }
      }
    }

    /* Zeta: no cabe en el nibble → se releva por CONFIG extendida (dlc=5). */
    static float zCache[4] = { NAN, NAN, NAN, NAN };
    struct ZK { const char* key; uint8_t idx; int slot; };
    static const ZK ZKEYS[] = { {"zeta_v", C2_ZETA_V, 0}, {"zeta_i", C2_ZETA_I, 1},
                                {"bus_chk_en", C2_BUS_CHK_EN, 2}, {"bus_chk_tol", C2_BUS_CHK_TOL, 3} };
    for (auto& zk : ZKEYS) {
      if (!doc.containsKey(zk.key)) continue;
      float v = doc[zk.key].as<float>();
      if (isnan(zCache[zk.slot]) || zCache[zk.slot] != v) {
        if (canQueueConfigExt(agent, zk.idx, v)) zCache[zk.slot] = v;
      }
    }
  }
#endif
}

/* ============================================================
   GATEWAY WiFi → CAN  (AGENTE 3, protocolo extendido)
   Lee /agente3/params y retransmite por CAN como frames extendidos.
============================================================ */
static void relayConfig3() {
#if CAN_RELAY_CONFIG
  static float cache[80];
  static bool  init = false;
  static bool  prevOnline = false;
  if (!init) { for (int i = 0; i < 80; i++) cache[i] = NAN; init = true; }

  /* AG3 ya NO persiste en NVS: al arrancar/reconectar vuelve a sus defaults.
     Cuando el gateway lo ve (re)aparecer en el bus CAN, invalida el cache para
     RE-EMPUJAR toda la config de Firebase y dejar a AG3 configurado. */
  CanAgent3Data d3;
  bool online3 = canGetAgent3(&d3) && d3.online;
  if (online3 && !prevOnline) {
    for (int i = 0; i < 80; i++) cache[i] = NAN;
    Serial.println("[CFG] AG3 (re)conectado → re-empujando config desde Firebase");
  }
  prevOnline = online3;

  String resp = fbGet("/agente3/params");
  if (resp == "null" || resp.isEmpty()) return;
  StaticJsonDocument<2048> doc;
  if (deserializeJson(doc, resp)) return;

  struct K { const char* key; uint8_t idx; };
  static const K KEYS[] = {
    {"target_v",C3_TARGET_V},{"omega_v",C3_OMEGA_V},{"omega_i",C3_OMEGA_I},
    {"zeta_v",C3_ZETA_V},{"zeta_i",C3_ZETA_I},{"plant_vdc",C3_PLANT_VDC},
    {"kd_droop",C3_KD_DROOP},{"i_ref_max",C3_IREF_MAX},{"i_ref_min",C3_IREF_MIN},
    {"ov_trip",C3_OV_TRIP},{"uv_trip",C3_UV_TRIP},{"oc_trip",C3_OC_TRIP},{"run_mode",C3_RUN_MODE},
    {"bat_ov_trip",C3_BAT_OV_TRIP},{"bat_uv_trip",C3_BAT_UV_TRIP},{"soc_empty_v",C3_SOC_EMPTY_V},{"soc_full_v",C3_SOC_FULL_V},
    {"pwm_period",C3_PWM_PERIOD_US},{"ton_max",C3_TON_MAX_US},{"min_off",C3_MIN_OFF_US},{"softstart_s",C3_SOFTSTART_S},
    {"pv_mode",C3_PV_MODE},{"pv_fixed_duty",C3_PV_FIXED_DUTY},
    {"pv_vpv_ref",C3_PV_VPV_REF},{"pv_vpv_ref_min",C3_PV_VPV_REF_MIN},{"pv_vpv_ref_max",C3_PV_VPV_REF_MAX},
    {"pv_kp",C3_PV_KP},{"pv_ki",C3_PV_KI},{"pv_duty_min",C3_PV_DUTY_MIN},{"pv_duty_start",C3_PV_DUTY_START},
    {"pv_duty_max",C3_PV_DUTY_MAX},{"pv_duty_slew",C3_PV_DUTY_SLEW},{"pv_softstart_ms",C3_PV_SOFTSTART_MS},{"pv_ctrl_ms",C3_PV_CTRL_MS},
    {"pv_panel_trip_low",C3_PV_PANEL_TRIPLO},{"pv_bat_warn_high",C3_PV_BAT_WARN_HI},{"pv_bat_trip_high",C3_PV_BAT_TRIP_HI},
    {"pv_iin_max",C3_PV_IIN_MAX},{"pv_ibat_max",C3_PV_IBAT_MAX},{"pv_pin_max",C3_PV_PIN_MAX},{"pv_irev_max",C3_PV_IREV_MAX},
    {"pv_ibat_lim",C3_PV_IBAT_LIM},{"ibat_idle",C3_IBAT_IDLE},
    {"bus_chk_en",C3_BUS_CHK_EN},{"bus_chk_tol",C3_BUS_CHK_TOL},
    {"mppt_step_v",C3_MPPT_STEP_V},{"mppt_eps_power",C3_MPPT_EPS_POWER},{"mppt_min_power",C3_MPPT_MIN_POWER},{"mppt_period_ms",C3_MPPT_PERIOD_MS},
    {"mppt_reacq_ms",C3_MPPT_REACQ_MS},
    {"op_mode",C3_OP_MODE},
    {"div_bus5",C3_DIV_BUS5},{"div_bat2s",C3_DIV_BAT2S},{"div_panel",C3_DIV_PANEL},{"div_spare",C3_DIV_SPARE},
    {"isens_ibus",C3_ISENS_IBUS},{"isens_ipanel",C3_ISENS_IPANEL},{"isens_ibat",C3_ISENS_IBAT},{"kcl_eta",C3_KCL_ETA},
    {"imode_ibus",C3_IMODE_IBUS},{"imode_ipanel",C3_IMODE_IPANEL},{"imode_ibat",C3_IMODE_IBAT},
  };
  for (auto& k : KEYS) {
    if (!doc.containsKey(k.key)) continue;
    float v = doc[k.key].as<float>();
    if (isnan(cache[k.idx]) || cache[k.idx] != v) {
      if (canQueueConfigExt(3, k.idx, v)) cache[k.idx] = v;
    }
  }

  /* One-shots: la presencia dispara el comando; se borran tras relevar. */
  struct OS { const char* key; uint8_t idx; };
  static const OS ONESHOTS[] = {
    {"fault_ack",C3_FAULT_ACK},{"pv_fault_ack",C3_PV_FAULT_ACK},{"pv_arm",C3_PV_ARM},
    {"cal_bus5",C3_CAL_BUS5},{"cal_bat2s",C3_CAL_BAT2S},{"cal_panel",C3_CAL_PANEL},{"cal_spare",C3_CAL_SPARE},
    {"izero_ibus",C3_IZERO_IBUS},{"izero_ipanel",C3_IZERO_IPANEL},{"izero_ibat",C3_IZERO_IBAT},
    {"save",C3_SAVE_CFG},
  };
  for (auto& os : ONESHOTS) {
    if (!doc.containsKey(os.key)) continue;
    float v = doc[os.key].as<float>();
    if (canQueueConfigExt(3, os.idx, v))
      fbPatch("/agente3/params", (String("{\"") + os.key + "\":null}").c_str());
  }
#endif
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

    /* Leer parámetros cada 5 s (propios + relay de config a agentes 2/3) */
    if (WiFi.status() == WL_CONNECTED && now - lastParamsMs >= PARAMS_READ_MS) {
      lastParamsMs = now;
      readParams();
      relayConfigToAgents();
      relayConfig3();
    }

    /* Publicar telemetría propia y datos de agentes (propio + relay CAN) */
    if (WiFi.status() == WL_CONNECTED && now - lastTelMs >= TELEMETRY_MS) {
      lastTelMs = now;
      publishTelemetry();
      publishAgente();
      publishRelayedAgents();
      publishAgent3();
      publishConsensus();
      publishSystem();      // reporte de conectividad + heartbeat del gateway
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
