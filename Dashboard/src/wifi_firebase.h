#pragma once
/*
 * wifi_firebase.h
 * ─────────────────────────────────────────────────────────────
 * Conexión WiFi y comunicación con Firebase Realtime Database.
 *
 * Rutas Firebase:
 *   GET  /conversor/params      → parámetros de control (web escribe, ESP32 lee)
 *   PUT  /conversor/params      → valores por defecto si el nodo no existe
 *   PATCH /conversor/telemetria → estado del conversor (ESP32 escribe)
 *   PATCH /agentes/agente2      → datos simulados CAN
 *   POST  /history/agente2      → historial
 * ─────────────────────────────────────────────────────────────
 */

#include <Arduino.h>

/* ── WiFi ── */
void wifiConnect();
void wifiReconnectIfNeeded();

/* ── Firebase Auth ── */
bool firebaseLogin();
void checkTokenRefresh();

/* ── Firebase REST ── */
bool   fbPatch(const char* path, const String& body);
bool   fbPut  (const char* path, const String& body);
bool   fbPost (const char* path, const String& body);
String fbGet  (const char* path);

/* ── Lógica de negocio ── */
void readParams();          // lee /conversor/params y aplica variables
void publishAgente2(float vbat, float vbus, float iout);
void publishTelemetry();

/* ── Tarea FreeRTOS (Core 0) ── */
void taskComms(void* pvParams);
