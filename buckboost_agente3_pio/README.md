# Agente 3 integrado: buckboost bidireccional + boost PV MPPT

Firmware PlatformIO/Arduino para ESP32 Feather. Este proyecto integra en un solo ESP32:

- Buckboost bidireccional entre baterias 2S Li-ion nominal 7.4 V y bus de 5 V.
- Boost PV con MPPT que extrae potencia desde un panel y la inyecta al nodo de baterias/PV2S.
- Dos ADS1115 en el mismo bus I2C aislado por ISO1540.

El firmware arranca siempre con buckboost y boost PV en `OFF`.

## Resumen Para Otra IA

Este README debe bastar para retomar el proyecto en otra sesion sin leer el chat anterior.

Objetivo del sistema:

- El agente 3 tiene dos 18650 en serie, no en paralelo.
- La bateria total se trata como nodo `PV2S` o `VBAT_2S`.
- En modo BUCK, el buckboost descarga energia desde `PV2S` hacia `BUS5`.
- En modo BOOST, el buckboost toma energia desde `BUS5` y carga `PV2S`.
- El boost PV toma energia desde panel `VPV` y la entrega al nodo `PV2S`.
- La prioridad de seguridad es evitar sobrecargar las 18650: vigilar especialmente `IBAT`.

Proyectos relacionados en el workspace:

- `buckboost_agente3_pio`: firmware completo actual, buckboost + boost PV MPPT.
- `buckboost_agente3_voltage_only_pio`: firmware separado solo para buckboost agente 3, usado para probar control de voltaje sin MPPT.
- `buckboost_pio_adaptado`: base anterior de agentes 1/2, 3.7 V a 5 V.
- `boost_mppt_real_platformio`: proyecto original del boost PV con MPPT.

Decisiones importantes ya tomadas:

- El buckboost conserva los pines de agentes 1/2: `GPIO25` y `GPIO26`.
- El boost PV original usaba `GPIO25/GPIO26`, pero aqui no se pueden reutilizar porque ya son del buckboost.
- En este firmware completo el boost PV usa `GPIO27` como PWM y `GPIO14` como enable asumido.
- Si el hardware final no tiene enable dedicado para PV, cambiar `PIN_PVBOOST_EN` a `-1` en `src/main.cpp`.
- Las mediciones analogicas entran por ADS1115, no por ADC interno de ESP32.
- Los comandos del monitor serial se pueden encadenar en una sola linea separando con `;`.
- El firmware puede operar en `FULL`, `BUCKBOOST_ONLY` o `PV_ONLY` para probar conversores por separado antes de integrarlos con baterias.

## Archivos Y Entorno

- Proyecto: `buckboost_agente3_pio`.
- Archivo principal: `src/main.cpp`.
- Configuracion: `platformio.ini`.
- Entorno PlatformIO: `agente3_featheresp32`.
- Framework: Arduino ESP32.
- Monitor serial: `115200`.
- No se requieren librerias externas para ADS1115: se maneja por I2C a bajo nivel.

Comando de compilacion desde PowerShell:

```powershell
cd "C:\Users\diego\OneDrive\Documentos\New project\buckboost_agente3_pio"; C:\Users\diego\.platformio\penv\Scripts\pio.exe run
```

Compilar, cargar y abrir monitor:

```powershell
cd "C:\Users\diego\OneDrive\Documentos\New project\buckboost_agente3_pio"; C:\Users\diego\.platformio\penv\Scripts\pio.exe run; C:\Users\diego\.platformio\penv\Scripts\pio.exe run -t upload; C:\Users\diego\.platformio\penv\Scripts\pio.exe device monitor -b 115200
```

## Pines ESP32

I2C:

- `GPIO21`: SDA.
- `GPIO22`: SCL.
- Ambos ADS1115 comparten el mismo bus I2C a traves del ISO1540.

Buckboost bidireccional:

- `GPIO25`: `QH` / high-side, activo LOW.
- `GPIO26`: `QL` / low-side, activo HIGH.
- `PWM_CH_HIGH = 0`.
- `PWM_CH_LOW = 1`.

Boost PV:

- `GPIO27`: PWM del boost PV.
- `GPIO14`: enable del boost PV, activo HIGH segun firmware.
- `PWM_CH_PV = 2`.
- Si no existe enable fisico, usar `PIN_PVBOOST_EN = -1`.

Comparacion con agente 1/2:

| Funcion | Agente 1/2 | Agente 3 completo |
|---|---:|---:|
| I2C SDA | GPIO21 | GPIO21 |
| I2C SCL | GPIO22 | GPIO22 |
| Buckboost high-side | GPIO25 | GPIO25 |
| Buckboost low-side | GPIO26 | GPIO26 |
| Boost PV PWM | no existe | GPIO27 |
| Boost PV enable | no existe | GPIO14 |

## Topologias De Potencia

Buckboost:

- BUCK `PV2S -> BUS5`: conmuta `QH` con PWM activo bajo, `QL` queda OFF.
- BOOST `BUS5 -> PV2S`: conmuta `QL` con PWM activo alto, `QH` queda OFF.

Boost PV:

- Extrae potencia del panel.
- Entrega potencia al nodo `PV2S`, donde se conectan baterias, entrada/salida del buckboost y salida del boost PV.
- El MPPT regula indirectamente usando `VPV`, `IPV`, `VBAT_2S` e `IBAT`.

## Mapa De Mediciones ADS

ADS1/main, direccion default `0x48`:

| Canal | Senal | Tipo | Uso |
|---|---|---|---|
| A0 | `BUS5` | Voltaje | Bus de 5 V, salida BUCK y entrada BOOST |
| A1 | `VBAT_2S` / `PV2S` | Voltaje | Baterias 2S total / nodo de potencia |
| A2 | `spare` | Voltaje | Libre/sin uso por ahora |
| A3 | `Ibb_BUS5` | Corriente | Corriente del buckboost hacia/desde BUS5 |

ADS2/mppt, direccion default `0x49`:

| Canal | Senal | Tipo | Uso |
|---|---|---|---|
| A0 | `VPV` | Voltaje | Voltaje del panel |
| A1 | `VBAT_2S` / `PV2S` | Voltaje | Opcional/duplicado, no es la fuente primaria |
| A2 | `IPV` | Corriente | Corriente del panel |
| A3 | `IBAT` | Corriente | Corriente de bateria, positiva si carga |

El controlador buckboost usa lectura rapida continua del ADS necesario segun topologia:

- BUCK regula `BUS5` desde `ADS1 A0`.
- BOOST regula `PV2S` desde `ADS1 A1`.

El control PV lee en su ciclo de control `VPV`, `VBAT_2S`, `IPV` e `IBAT`.

## Problema Historico ADS Y Solucion

Se observo antes que `A0` parecia seguir/copia a `A1` o `A2`. Para evitar que se repita:

- Cada lectura de barrido usa single-shot.
- Al cambiar de canal se descarta una conversion.
- Se espera el bit `OS` del ADS1115 antes de leer.
- Se valida que el mux del registro de configuracion coincida con el canal pedido.
- Despues de leer todos los canales se reconfigura el canal rapido de control.

Diagnostico recomendado:

```text
off; adsraw; calstatus; read
```

Si las direcciones ADS estan invertidas:

```text
off; adsswap; adsraw; read
```

O fijarlas explicitamente:

```text
off; adsaddr main 0x48; adsaddr mppt 0x49; adsraw; read
```

## Divisores Resistivos

Los voltajes se leen en el ADS ya divididos. El factor por defecto es `3.12`, cercano a un divisor 33k/16k ideal:

```text
factor ideal = (33k + 16k) / 16k = 3.0625
```

Factores configurables:

- `divbus5`: aplica a `ADS1 A0`.
- `divbat2s` o `divpv2s`: aplica a `ADS1 A1`.
- `divspare`: aplica a `ADS1 A2`.
- `divpanel` o `divvpv`: aplica a `ADS2 A0`.

Comandos directos:

```text
divbus5 3.12; divbat2s 3.12; divspare 3.12; divpanel 3.12; savecfg
```

Calibracion con voltaje real medido externamente:

```text
off; cal bus5 5.000; cal bat2s 7.400; cal panel 6.000; savecfg
```

## Corrientes Y Convenciones

Sensores de corriente asumidos tipo ACS712-05B:

- Cero nominal: `2.50 V`.
- Sensibilidad nominal: `0.185 V/A`.
- Cada canal tiene `zero`, `sens`, `sign`, `alpha` y `deadband`.

Convenciones:

- `IBAT` positiva significa que la bateria se carga.
- `IPV` positiva significa que el panel entrega corriente al boost PV.
- `Ibb_BUS5` debe verificarse experimentalmente; si el signo queda invertido, usar `flip ibus`.
- Si un sensor queda invertido, usar `flip ibus`, `flip ipv` o `flip ibat`.

El limite mas importante para las dos 18650 en serie es:

```text
pv_ibatmax 1.0
```

Este valor limita corriente positiva de carga hacia baterias desde el boost PV.

## KCL Del Nodo PV2S

El nodo `PV2S` une bateria, buckboost y salida del boost PV.

El firmware estima:

```text
Iboost_est = eta * VPV * IPV / VBAT_2S
Ibb_in_2S = Iboost_est - IBAT
```

Donde:

- `Iboost_est` positiva inyecta corriente desde el boost PV al nodo 2S.
- `IBAT` positiva carga la bateria desde el nodo.
- `Ibb_in_2S` positiva entra al buckboost desde el nodo 2S.
- `eta` es eficiencia estimada del boost PV.

Ajuste:

```text
eta 0.90
```

Esta estimacion no reemplaza una medicion fisica directa de corriente de salida del boost, pero sirve para diagnostico y balance de potencia.

## Comandos Iniciales

Primer diagnostico seguro:

```text
off; calstatus; adsraw; read; switching
```

Reset completo de configuracion y lectura:

```text
off; factoryreset; adsraw; read; switching
```

Seleccion de modo de prueba:

```text
testmode full
testmode bb
testmode pv
```

Aliases aceptados:

```text
sysmode full
sysmode buckboost
sysmode mppt
only_bb
only_pv
```

El modo se guarda en NVS con `savecfg` y tambien se guarda automaticamente al usar `testmode ...`.

## Modos De Prueba Separada

`FULL`:

- Modo integrado normal.
- Permite buckboost y boost PV.
- Usar solo cuando el hardware completo ya este revisado.

```text
off; testmode full; read
```

`BUCKBOOST_ONLY`:

- Permite probar solo el buckboost.
- El boost PV queda forzado a `OFF`.
- `pv_arm`, `pv_mppt`, `pv_vpv`, `pv_fixed` y `pv_prearm` se rechazan.
- Util para regular `BUS5` desde fuente/baterias 2S sin activar el boost del panel.
- Requiere `ADS1`, porque el buckboost mide `BUS5` en `ADS1 A0` y `PV2S` en `ADS1 A1`.

```text
off; testmode bb; preset_buck; read; buck_ton 5; buck_ton 10; off
```

`PV_ONLY`:

- Permite probar solo el boost PV + MPPT.
- El buckboost queda forzado a `OFF`.
- `ton`, `auto`, `buck_ton`, `boost_ton` y prearm buckboost se rechazan.
- Util para validar el boost PV con carga/fuente controlada en el nodo de salida antes de integrarlo a baterias.
- Requiere ambos ADS: `ADS1 A1` para `VBAT_2S/PV2S` y `ADS2` para `VPV`, `IPV` e `IBAT`.

```text
off; testmode pv; read; pv_prearm; preset_mppt; pv_arm; pv_mppt
```

Para volver al comportamiento integrado:

```text
off; testmode full; read
```

Guardar parametros:

```text
savecfg
```

Cargar parametros desde NVS:

```text
off; loadcfg; read
```

## Buckboost

Comandos principales:

```text
mode buck
mode boost
preset_buck
preset_boost
ton 10
buck_ton 20
boost_ton 20
auto
off
clear_fault
```

Secuencia BUCK sugerida, de `PV2S` a `BUS5`:

```text
off; preset_buck; read; buck_ton 5; buck_ton 10; buck_ton 20; off
```

Luego control automatico:

```text
target 5.0; auto
```

Secuencia BOOST sugerida, de `BUS5` a `PV2S`:

```text
off; preset_boost; read; boost_ton 5; boost_ton 10; boost_ton 20; off
```

Luego control automatico:

```text
target 8.2; auto
```

Parametros buckboost utiles:

```text
kp 6; ki 1; ff 1.0; period 100; ctrlms 1.163; tonmax 85; minoff 8
```

Notas:

- `period 100` equivale a 10 kHz.
- `ctrlms` se limita internamente a la velocidad viable del ADS1115.
- El control BUCK fue probado antes con 10 kHz y Ton alrededor de 60 a 70 us para regular cerca de 5 V desde 7.4 V.

## Boost PV / MPPT

Comandos principales:

```text
pv_prearm
pv_arm
pv_disarm
pv_off
pv_sensors
pv_fixed 0.05
pv_vpv
pv_mppt
preset_mppt
```

Secuencia segura inicial:

```text
off; read; pv_prearm; preset_mppt; pv_arm; pv_mppt
```

Parametros utiles:

```text
pv_dmax 0.48
pv_duty 0.05
pv_vpvref 5.0
pv_vpvmin 4.2
pv_vpvmax 5.4
pv_kp 0.06
pv_ki 0.9
pv_slew 0.01
pv_ctrlms 50
pv_imax 3.0
pv_ibatmax 1.0
pv_pmax 16
pv_batwarn 8.45
pv_battrip 8.70
mppt_step 0.025
mppt_ms 1000
mppt_epsp 0.08
```

Antes de usar `pv_arm`, confirmar con osciloscopio:

- PWM PV sale por `GPIO27`.
- `GPIO14` realmente corresponde a enable si esta conectado.
- No hay conmutacion PV en `GPIO25/GPIO26`, porque esos son del buckboost.

## Calibracion De Voltajes

Comandos:

```text
cal bus5 <V>     ; ADS1 A0
cal bat2s <V>    ; ADS1 A1
cal spare <V>    ; ADS1 A2
cal panel <V>    ; ADS2 A0
cal vin <V>      ; segun topologia buckboost
cal vout <V>     ; segun topologia buckboost
```

Ejemplo:

```text
off; cal bus5 5.000; cal bat2s 7.420; cal panel 6.250; savecfg; read
```

## Calibracion De Corrientes

Modo recomendado:

```text
off; calmode on; zeroacs; calstatus; savecfg; calmode off
```

`calmode on`:

- Apaga buckboost y boost PV mediante apagado seguro.
- Mantiene ambas etapas en `OFF`.
- Inhibe temporalmente solo faults derivados de corriente (`IPV`, `IBAT`, `PPV`).
- No inhibe protecciones de voltaje.
- Bloquea `auto`, `ton` y `pv_arm`.

Comandos:

```text
calmode on
calmode off
calhold 60
zeroacs
zero ibus
zero ipv
zero ibat
flip ibus
flip ipv
flip ibat
cal ibus <A>
cal ipv <A>
cal ibat <A>
izero ibus <Vadc>
isens ibus <V/A>
ialpha all <0..1>
idead all <A>
```

Ejemplo de calibracion por corriente conocida:

```text
off; calmode on; zero ibat; cal ibat 0.500; calstatus; savecfg; calmode off
```

Ejemplo de ajuste manual directo:

```text
off; calmode on; izero ibat 2.502; isens ibat 0.185; idead ibat 0.04; ialpha ibat 0.30; savecfg; calmode off
```

## Persistencia NVS

Namespace:

```text
ag3_full
```

Magic actual:

```text
0xA6030003
```

Comandos:

```text
savecfg
loadcfg
factoryreset
```

Si se cambia el mapa de mediciones o semantica persistida, actualizar `CFG_MAGIC` en `src/main.cpp` para forzar defaults limpios.

## Apagado Seguro

El comando `off`, los cambios de topologia y los faults usan apagado seguro:

- El firmware reduce Ton/duty por pasos.
- Luego deja un retardo corto para descarga del inductor.
- Despues deshabilita completamente las salidas.

Ajustes:

```text
safe_bb_steps 12
safe_pv_steps 12
bb_discharge_us 1500
pv_discharge_us 1500
```

Comando recomendado:

```text
safe_bb_steps 12; safe_pv_steps 12; bb_discharge_us 1500; pv_discharge_us 1500; savecfg
```

## Protecciones Principales

Buckboost BOOST:

- `boost_pv2strip`: limite alto de `PV2S`.
- `boost_bus5min`: limite bajo de `BUS5`.

Buckboost BUCK:

- `buck_pv2smin`: minimo `PV2S`.
- `buck_pv2strip`: maximo `PV2S`.
- `buck_bus5trip`: maximo `BUS5`.

Boost PV:

- `pv_battrip`: maximo `VBAT_2S`.
- `pv_batwarn`: advertencia `VBAT_2S`.
- `pv_imax`: maximo `IPV`.
- `pv_ibatmax`: maximo `IBAT` positiva de carga.
- `pv_pmax`: maximo `PPV`.

Para dos 18650 en serie, mantener `pv_ibatmax` conservador hasta validar celda, BMS, temperatura y fuente.

## Diagnostico Rapido

Mapa y ADS:

```text
off; adsmap; adsraw; read
```

Calibracion:

```text
off; calstatus; calmode on; zeroacs; calstatus; calmode off
```

Buckboost BUCK sin MPPT:

```text
off; preset_buck; read; buck_ton 5; buck_ton 10; buck_ton 20; off
```

PV solo sensores:

```text
off; pv_sensors; read; pv_off
```

MPPT con prearm:

```text
off; read; pv_prearm; preset_mppt; pv_arm; pv_mppt
```

## Riesgos Y Pendientes De Hardware

Confirmar antes de pruebas con potencia:

- Que `GPIO27` es realmente PWM del boost PV.
- Que `GPIO14` es realmente enable del boost PV, o poner `PIN_PVBOOST_EN = -1`.
- Que `ADS1` y `ADS2` tienen direcciones distintas, normalmente `0x48` y `0x49`.
- Que los divisores reales corresponden a los factores configurados.
- Que los ACS712 estan alimentados correctamente y su cero queda cerca de 2.5 V.
- Que los signos de `IPV`, `IBAT` e `Ibb_BUS5` son coherentes.
- Que el buckboost no conmuta ambos MOSFET a la vez.
- Que el inductor y diodos no saturan ni se calientan bajo carga.
- Que la masa del osciloscopio esta conectada correctamente; antes aparecieron peaks grandes que disminuyeron al mover el cable de tierra.

Este firmware no reemplaza fusibles, BMS, limites de corriente externos ni supervision termica.
