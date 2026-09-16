/*
 * config.cpp — AGENTE 3
 * ─────────────────────────────────────────────────────────────
 * NVS DESHABILITADO a propósito. El agente 3 ya NO persiste su configuración
 * localmente: arranca con los valores por defecto (shared.cpp) y toda su
 * configuración llega desde Firebase, relevada por CAN desde el gateway
 * (agente 1). La "persistencia" real es la propia base de datos de Firebase.
 *
 * Se conservan las firmas saveConfig()/loadConfig() como no-ops para no tocar
 * los puntos de llamada ni el resto del proyecto.
 * ─────────────────────────────────────────────────────────────
 */
#include "config.h"
#include "shared.h"

void saveConfig() {
  /* Sin NVS: no se persiste nada. La configuración vive en Firebase. */
}

bool loadConfig() {
  /* Sin NVS: siempre arranca con defaults; Firebase re-empuja la config por CAN. */
  Serial.println("[CFG] NVS deshabilitado — configuración desde Firebase");
  return false;
}
