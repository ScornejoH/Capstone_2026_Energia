#pragma once
/*
 * wifi_firebase.h
 * ─────────────────────────────────────────────────────────────
 * Conexión WiFi y comunicación con Firebase Realtime Database.
 *
 * Rutas Firebase:
 *   GET   /agente1/params       → parámetros de control (web escribe)
 *   PUT   /agente1/params       → valores por defecto si no existe
 *   PATCH /agente1/telemetria   → telemetría completa (ESP32 escribe)
 *   PATCH /agentes/agente1      → valores actuales para el dashboard
 *   POST  /history/agente1      → historial
 * ─────────────────────────────────────────────────────────────
 */

#include <Arduino.h>

void wifiConnect();
void wifiReconnectIfNeeded();

bool firebaseLogin();
void checkTokenRefresh();

bool   fbPatch(const char* path, const String& body);
bool   fbPut  (const char* path, const String& body);
bool   fbPost (const char* path, const String& body);
String fbGet  (const char* path);
bool   fbDelete(const char* path);

void readParams();
void publishTelemetry();
void publishAgente();

void taskComms(void* pvParams);
