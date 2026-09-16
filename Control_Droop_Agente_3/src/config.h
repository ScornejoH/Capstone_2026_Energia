#pragma once
/*
 * config.h — AGENTE 3
 * NVS DESHABILITADO: la configuración llega solo desde Firebase (relevada por
 * CAN desde el agente 1). Estas firmas quedan como no-ops por compatibilidad.
 */
void saveConfig();   // no-op (sin NVS)
bool loadConfig();   // no-op: siempre false (arranca con defaults)
