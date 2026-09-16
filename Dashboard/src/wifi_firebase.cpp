/*
 * wifi_firebase.cpp
 * ─────────────────────────────────────────────────────────────
 * WiFi, Firebase Auth y REST. Corre en Core 0.
 *
 * Lógica de parámetros:
 *   - La web escribe en /conversor/params los valores deseados
 *   - La ESP32 lee ese nodo cada PARAMS_READ_MS y aplica los valores
 *   - Si el nodo no existe, la ESP32 lo crea con los valores por defecto
 *   - La telemetría se publica en /conversor/telemetria cada 2 s
 * ─────────────────────────────────────────────────────────────
 */

#include "wifi_firebase.h"
#include "shared.h"
#include "can_bus.h"
#include "control.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

/* ============================================================
   ESTADO PRIVADO
============================================================ */
static WiFiClientSecure secureClient;
static String           auth_token = "";
static uint32_t         token_ts   = 0;

static float    t_sim        = 0.0f;
static uint32_t canRondas    = 0;

static bool     canOnline    = false;
static uint8_t  canFailCount = 0;
static uint32_t canLastRetry = 0;

#define ROUND_PERIOD_MS    1000
#define PARAMS_READ_MS     5000    // leer params desde Firebase cada 5 s
#define CAN_FAIL_THRESHOLD 3
#define CAN_RETRY_MS       10000

/* ============================================================
   ACTUALIZAR hwStatus COMBINANDO ADS Y CAN
============================================================ */
static void setCanStatus(bool canOk) {
  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
    bool adsOk = (hwStatus == HW_OK || hwStatus == HW_NO_CAN);
    if  (adsOk &&  canOk) hwStatus = HW_OK;
    else if  (adsOk && !canOk) hwStatus = HW_NO_CAN;
    else if (!adsOk &&  canOk) hwStatus = HW_NO_ADS;
    else                       hwStatus = HW_NO_CAN_NO_ADS;
    xSemaphoreGive(xMutex);
  }
}

/* ============================================================
   WIFI
============================================================ */
static const char* wifiStatusStr(wl_status_t s) {
  switch (s) {
    case WL_IDLE_STATUS:     return "IDLE (esperando)";
    case WL_NO_SSID_AVAIL:  return "SSID no encontrado (verifica nombre del hotspot)";
    case WL_SCAN_COMPLETED:  return "Escaneo completado";
    case WL_CONNECTED:       return "Conectado";
    case WL_CONNECT_FAILED:  return "Fallo de conexión (contraseña incorrecta?)";
    case WL_CONNECTION_LOST: return "Conexión perdida";
    case WL_DISCONNECTED:    return "Desconectado";
    default:                 return "Estado desconocido";
  }
}

void wifiConnect() {
  Serial.printf("[WiFi] Conectando a '%s'...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500);
    Serial.printf("[WiFi] Intento %d/30 — estado: %d (%s)\n",
                  i + 1, WiFi.status(), wifiStatusStr(WiFi.status()));
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("[WiFi] OK — IP: %s\n", WiFi.localIP().toString().c_str());
  else
    Serial.printf("[WiFi] FAIL — estado final: %d (%s)\n",
                  WiFi.status(), wifiStatusStr(WiFi.status()));
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
  Serial.print("[AUTH] Autenticando ESP32 en Firebase...");
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
  if (auth_token.length() == 0 || millis() - token_ts > TOKEN_REFRESH_MS) {
    Serial.println("[AUTH] Renovando token...");
    bool wasEmpty = (auth_token.length() == 0);
    if (firebaseLogin() && wasEmpty) {
      // Primera autenticación exitosa tras un fallo inicial:
      // publicar inmediatamente para no esperar el ciclo de 2/5 s
      readParams();
      publishTelemetry();
    }
  }
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

/* ============================================================
   PARÁMETROS — leer desde Firebase y aplicar
============================================================ */

// Escribe los valores por defecto en /conversor/params
static void writeDefaultParams() {
  StaticJsonDocument<512> doc;
  // Modo: 0=OFF 1=MANUAL 2=AUTO_FFPI
  doc["run_mode"]    = 0;
  doc["topology"]    = 1;           // 0=BUCK 1=BOOST
  doc["target_v"]    = 5.0;
  doc["kp"]          = 10.0;
  doc["ki"]          = 5.0;
  doc["ff_scale"]    = 1.5;
  doc["ton_max_us"]  = 95.0;
  doc["ton_hard_us"] = 99.0;
  doc["min_off_us"]  = 3.0;
  doc["slew_up"]     = 0.5;
  doc["slew_down"]   = 3.0;
  doc["manual_ton"]  = 10.0;
  String body; serializeJson(doc, body);
  fbPut("/conversor/params", body);
  Serial.println("[PARAMS] Nodo creado con valores por defecto");
}

// Lee /conversor/params y aplica los valores a las variables compartidas
void readParams() {
  String resp = fbGet("/conversor/params");

  if (resp == "null" || resp.isEmpty()) {
    writeDefaultParams();
    return;
  }

  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, resp)) {
    Serial.println("[PARAMS] Error parseando JSON");
    return;
  }

  if (xSemaphoreTake(xMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
    if (doc.containsKey("target_v"))    targetV          = clampF(doc["target_v"],    0.5f,  6.0f);
    if (doc.containsKey("kp"))          kp               = clampF(doc["kp"],          0.0f, 40.0f);
    if (doc.containsKey("ki"))          ki               = clampF(doc["ki"],          0.0f, 40.0f);
    if (doc.containsKey("ff_scale"))    ffScale          = clampF(doc["ff_scale"],    0.0f,  2.0f);
    if (doc.containsKey("ton_max_us"))  tonMaxUs         = clampF(doc["ton_max_us"],  0.0f, 95.0f);
    if (doc.containsKey("ton_hard_us")) tonHardMaxUs     = clampF(doc["ton_hard_us"], 0.0f, 99.0f);
    if (doc.containsKey("min_off_us"))  minOffUs         = clampF(doc["min_off_us"],  0.0f, 40.0f);
    if (doc.containsKey("slew_up"))     maxTonStepUpUs   = clampF(doc["slew_up"],    0.01f, 10.0f);
    if (doc.containsKey("slew_down"))   maxTonStepDownUs = clampF(doc["slew_down"],  0.01f, 20.0f);
    if (doc.containsKey("manual_ton"))  manualTonUs      = clampF(doc["manual_ton"],  0.0f, 95.0f);
    if (doc.containsKey("topology")) {
      int topo = doc["topology"] | 1;
      topology = (topo == 0) ? TOPO_BUCK : TOPO_BOOST;
    }
    if (doc.containsKey("run_mode")) {
      int rm = doc["run_mode"] | 0;
      if      (rm == 0) requestedMode = RUN_OFF;
      else if (rm == 1) requestedMode = RUN_MANUAL;
      else              requestedMode = RUN_AUTO_FFPI;
    }

    // Fault acknowledge: el usuario confirma que resolvió el error
    if (doc.containsKey("fault_ack") && doc["fault_ack"].as<bool>()) {
      faultLatched  = false;
      requestedMode = RUN_OFF;
    }

    // Calibración: recalcular divFactor con el voltaje real medido externamente
    // La web escribe cal_real_a0 / cal_real_a1 con el valor del multímetro
    if (doc.containsKey("cal_real_a0")) {
      float realV = doc["cal_real_a0"];
      if (rawVoltsA0 > 0.05f && realV > 0.1f)
        divFactorA0 = clampF(realV / rawVoltsA0, 0.5f, 10.0f);
    }
    if (doc.containsKey("cal_real_a1")) {
      float realV = doc["cal_real_a1"];
      if (rawVoltsA1 > 0.05f && realV > 0.1f)
        divFactorA1 = clampF(realV / rawVoltsA1, 0.5f, 10.0f);
    }

    xSemaphoreGive(xMutex);
  }

  // Limpiar calibración tras aplicarla — evita que se reaaplique en el próximo ciclo
  {
    String calPatch = "{";
    bool   needPatch = false;
    if (doc.containsKey("cal_real_a0")) { calPatch += "\"cal_real_a0\":null"; needPatch = true; }
    if (doc.containsKey("cal_real_a1")) {
      if (needPatch) calPatch += ",";
      calPatch += "\"cal_real_a1\":null";
      needPatch = true;
    }
    calPatch += "}";
    if (needPatch) {
      fbPatch("/conversor/params", calPatch);
      Serial.println("[CAL] Calibración aplicada y eliminada de Firebase");
    }
  }

  // Limpiar fault_ack en Firebase para que no se reprocese en el próximo ciclo
  if (doc.containsKey("fault_ack") && doc["fault_ack"].as<bool>()) {
    fbPatch("/conversor/params", "{\"fault_ack\":false}");
    Serial.println("[FAULT] Fault reconocido por usuario — faultLatched limpiado");
  }

  configFastCh();
}

/* ============================================================
   TELEMETRÍA — publicar estado del conversor
============================================================ */
void publishTelemetry() {
  Snapshot s = takeSnapshot();
  const char* rmNames[] = {"OFF", "MANUAL", "AUTO_FFPI"};
  const char* tpNames[] = {"BUCK", "BOOST"};
  const char* hwMsgs[]  = {
    "OK",
    "ADVERTENCIA: Modulo CAN desconectado. Control activo.",
    "ERROR: Sensor de voltaje (ADS1115) desconectado. Control inhibido.",
    "FALLA CRITICA: CAN y sensor de voltaje desconectados."
  };
  const char* hwLevels[] = { "ok", "warn", "error", "critical" };

  StaticJsonDocument<896> doc;
  doc["v_bus"]          = serialized(String(s.v_bus,    3));
  doc["v_ag1"]          = serialized(String(s.v_ag1,    3));
  doc["raw_a0"]         = serialized(String(s.rawVoltsA0, 4));
  doc["raw_a1"]         = serialized(String(s.rawVoltsA1, 4));
  doc["div_a0"]         = serialized(String(s.divFactorA0, 4));
  doc["div_a1"]         = serialized(String(s.divFactorA1, 4));
  float tonDisplay = (s.runMode == (int)RUN_MANUAL) ? s.manualTonUs : s.tonCmdUs;
  doc["ton_cmd"]        = serialized(String(tonDisplay,     3));
  doc["ton_applied"]    = serialized(String(s.tonAppliedUs, 3));
  doc["run_mode"]       = rmNames[s.runMode];
  doc["requested_mode"] = rmNames[s.requestedMode];
  doc["topology"]       = tpNames[s.topology];
  doc["fault"]          = s.fault;
  doc["kp"]             = serialized(String(s.kp,      3));
  doc["ki"]             = serialized(String(s.ki,      3));
  doc["ff_scale"]       = serialized(String(s.ffScale, 3));
  doc["target_v"]       = serialized(String(s.targetV, 3));
  int hw = s.hwStatus < 0 ? 0 : (s.hwStatus > 3 ? 3 : s.hwStatus);
  doc["hw_status"]      = hw;
  doc["hw_msg"]         = hwMsgs[hw];
  doc["hw_level"]       = hwLevels[hw];

  doc["ts"] = (long)(millis() / 1000);
  String body; serializeJson(doc, body);
  fbPatch("/conversor/telemetria", body);
}

bool fbDelete(const char* path) {
  HTTPClient http;
  http.begin(secureClient, fbUrl(path));
  int code = http.sendRequest("DELETE");
  http.end();
  return code == 200;
}

// URL con query params para limpiar historial
static String fbUrlQuery(const char* path, const char* query) {
  String url = "https://";
  url += FIREBASE_HOST;
  url += path;
  url += ".json?";
  url += query;
  if (auth_token.length() > 0) { url += "&auth="; url += auth_token; }
  return url;
}

// Elimina las N entradas más antiguas del historial de un agente
static void pruneHistory(const char* agente, int deleteCount) {
  if (deleteCount <= 0) return;

  // Obtener las N keys más antiguas (push keys son cronológicas)
  String url = fbUrlQuery(
    (String("/history/") + agente).c_str(),
    (String("orderBy=\"$key\"&limitToFirst=") + deleteCount).c_str()
  );

  HTTPClient http;
  http.begin(secureClient, url);
  int code = http.GET();
  if (code != 200) { http.end(); return; }

  String resp = http.getString();
  http.end();

  // Parsear las keys y borrar cada una
  // El JSON tiene forma: { "-OxxxKey1": {...}, "-OxxxKey2": {...} }
  StaticJsonDocument<2048> doc;
  if (deserializeJson(doc, resp)) return;

  for (JsonPair kv : doc.as<JsonObject>()) {
    String delPath = String("/history/") + agente + "/" + kv.key().c_str();
    fbDelete(delPath.c_str());
  }
}

/* Máximo de entradas en el historial por agente */
#define HISTORY_MAX_ENTRIES  200
#define HISTORY_PRUNE_BATCH   50   // cuántas borrar cuando se supera el límite

static uint32_t historyCount = 0;

/* ============================================================
   PUBLICAR DATOS AGENTE 2
============================================================ */
void publishAgente2(float vbat, float vbus, float iout) {
  {
    StaticJsonDocument<128> doc;
    doc["v_bat"] = serialized(String(vbat, 3));
    doc["v_bus"] = serialized(String(vbus, 3));
    doc["i_out"] = serialized(String(iout, 3));
    String b; serializeJson(doc, b);
    fbPatch("/agentes/agente2", b);
  }
  {
    StaticJsonDocument<128> doc;
    doc["v_bat"] = serialized(String(vbat, 3));
    doc["v_bus"] = serialized(String(vbus, 3));
    doc["i_out"] = serialized(String(iout, 3));
    doc["ts"]    = (long)(millis() / 1000);
    String b; serializeJson(doc, b);
    fbPost("/history/agente2", b);
    historyCount++;
  }

  // Limpiar entradas antiguas cuando se supera el límite
  if (historyCount > HISTORY_MAX_ENTRIES) {
    pruneHistory("agente2", HISTORY_PRUNE_BATCH);
    historyCount -= HISTORY_PRUNE_BATCH;
  }
}

/* ============================================================
   TAREA FREERTOS — Core 0
============================================================ */
void taskComms(void* pvParams) {
  secureClient.setInsecure();
  wifiConnect();
  /* ── Auth: reintentar hasta tener token válido ── */
  if (WiFi.status() == WL_CONNECTED) {
    int attempts = 0;
    while (auth_token.length() == 0 && attempts < 5) {
      if (attempts > 0) delay(3000);
      firebaseLogin();
      attempts++;
    }
    if (auth_token.length() == 0)
      Serial.println("[WARN] No se pudo autenticar tras 5 intentos. Continuando sin Firebase.");
  }

  /* ── CAN ── */
  randomSeed(esp_random());
  canOnline = canInit();
  setCanStatus(canOnline);
  Serial.printf("[CAN] MCP2515: %s\n", canOnline ? "OK" : "NO detectado");

  /* ── Leer params iniciales, publicar telemetría e inicializar contador historial ── */
  if (WiFi.status() == WL_CONNECTED) {
    readParams();
    publishTelemetry();

    // Contar entradas existentes en el historial para continuar desde el estado real
    HTTPClient httpCount;
    httpCount.begin(secureClient,
      fbUrlQuery("/history/agente2", "shallow=true"));
    if (httpCount.GET() == 200) {
      StaticJsonDocument<4096> doc;
      if (!deserializeJson(doc, httpCount.getString()))
        historyCount = (uint32_t)doc.as<JsonObject>().size();
      Serial.printf("[HIST] Entradas existentes: %lu\n", historyCount);
    }
    httpCount.end();
  }

  uint32_t lastCanMs    = millis();
  uint32_t lastTelMs    = millis();
  uint32_t lastParamsMs = millis();

  for (;;) {
    uint32_t now = millis();

    wifiReconnectIfNeeded();
    if (WiFi.status() == WL_CONNECTED) checkTokenRefresh();

    /* ── Propagar falla a Firebase (run_mode=OFF en /conversor/params) ── */
    if (pendingFaultWrite && WiFi.status() == WL_CONNECTED) {
      pendingFaultWrite = false;
      fbPatch("/conversor/params", "{\"run_mode\":0}");
      publishTelemetry();   // reflejar el estado OFF inmediatamente
      Serial.println("[FAULT] run_mode=OFF escrito en Firebase");
    }

    /* ── Reintento CAN ── */
    if (!canOnline && (now - canLastRetry >= CAN_RETRY_MS)) {
      canLastRetry = now;
      canOnline = canInit();
      if (canOnline) {
        canFailCount = 0;
        setCanStatus(true);
        Serial.println("[CAN] Reconectado");
      }
    }

    /* ── CAN + agente2 Firebase (cada 1 s) ── */
    if (now - lastCanMs >= ROUND_PERIOD_MS) {
      lastCanMs = now;
      float vbat, vbus, iout;
      simAgente2(t_sim, &vbat, &vbus, &iout);

      if (canOnline) {
        bool ok = canSendFloat(CAN_ID_VBAT, vbat)
               && canSendFloat(CAN_ID_VBUS, vbus)
               && canSendFloat(CAN_ID_IOUT, iout);
        canRondas++;
        Serial.printf("[CAN] #%lu vbat=%.3f vbus=%.3f iout=%.3f [%s]\n",
                      canRondas, vbat, vbus, iout, ok ? "OK" : "FAIL");
        if (!ok) {
          canFailCount++;
          if (canFailCount >= CAN_FAIL_THRESHOLD) {
            canOnline = false; canFailCount = 0; canLastRetry = now;
            setCanStatus(false);
            Serial.println("[CAN] Perdido");
          }
        } else { canFailCount = 0; }
      }

      if (WiFi.status() == WL_CONNECTED) publishAgente2(vbat, vbus, iout);
      t_sim += ROUND_PERIOD_MS / 1000.0f;
    }

    /* ── Leer parámetros desde Firebase (cada 5 s) ── */
    if (WiFi.status() == WL_CONNECTED && now - lastParamsMs >= PARAMS_READ_MS) {
      lastParamsMs = now;
      readParams();
    }

    /* ── Telemetría → Firebase (cada 2 s) ── */
    if (WiFi.status() == WL_CONNECTED && now - lastTelMs >= 2000) {
      lastTelMs = now;
      publishTelemetry();
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
