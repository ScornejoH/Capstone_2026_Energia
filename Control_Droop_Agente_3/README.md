# Control Droop Agente 3

Registro breve de cambios para la prueba de hardware del AG3.

## Cambios realizados

- Se agregó calibración explícita de rango para los sensores de corriente entre 5A y 20A.
- La configuración por defecto de todos los sensores de corriente quedó en 5A, equivalente a 0.185 V/A según el ACS712.
- Se guardó y recuperó ese rango en NVS junto con la calibración existente.
- Se añadió control CAN para cambiar el rango de corriente sin tocar manualmente la sensibilidad.
- La calibración de cero de batería se ajustó para partir desde 50 mA en vez de 0 A.
- Se redujo la frecuencia de pedido de muestras del subsistema PV para aliviar el bus I2C.
- Se agregó un modo `I2C_DEBUG` para imprimir las lecturas crudas y escaladas del segundo ADS (VPV, IPV e IBAT).
- En el arranque ahora se imprime el estado de ADS1 y ADS2, y cuando `I2C_DEBUG` está activo se ve una línea por muestra válida del ADS2.
- Se mantuvieron las conversiones y protecciones del buckboost sin cambiar la lógica principal del lazo.

## Correcciones sobre la primera solución

- **El ADS2 ahora se lee SIEMPRE** (cada ~120 ms), aunque el PV esté OFF. Antes solo se leía en modos PV activos, por eso "no llegaban" las mediciones del segundo sensor con el PV apagado. Este era el origen real del problema, no solo la saturación.
- **Reloj I2C bajado de 400 kHz a 100 kHz** (`I2C_CLOCK_HZ` en `sensors.cpp`). Con dos ADS1115 en el mismo bus, 400 kHz dejaba al segundo sin responder. Es el arreglo de hardware para el ADS2 marginal.
- **`I2C_DEBUG` pasó de `const bool` a macro `#define`**; como estaba, el `#if I2C_DEBUG` de `sensors.cpp` nunca compilaba y el print por muestra no salía.
- **Glitch de 2.5 s explicado:** el gateway (AG1) solo reenvía config cuando cambia (no hay temporizador de 2.5 s). El glitch venía de que la lectura del ADS2 corre en la MISMA tarea que el lazo del buckboost y lo congela mientras convierte; si el ADS2 estaba marginal, cada `waitReady` agotaba hasta 8 ms. Al bajar el reloj (lecturas fiables) y decimar el muestreo, el buckboost deja de congelarse de forma errática.
- **Cero de i_bat e i_panel ahora usa la corriente FILTRADA** (consistente con i_bb), no una muestra cruda instantánea. i_bat se calibra a +50 mA; i_panel a 0.
- **Web:** los sensores de corriente usan un **selector 5A/20A** (`imode_*`) en vez de un campo de sensibilidad. El signo se invierte con ↔ SIGNO usando la sensibilidad del rango elegido.
- **Gateway:** se agregaron las claves `imode_ibus/ipanel/ibat` al relay del AG1 y los índices `C3_IMODE_*` a su `can_bus.h`.

## Estado actual

- Compilación verificada con PlatformIO (AG3 y AG1 → SUCCESS).
- Pendiente validar en hardware: (1) que el segundo sensor responda de forma estable a 100 kHz; (2) que el cero de batería quede en ~+50 mA; (3) que desaparezcan los glitches del bus DC.

## Notas para seguir trabajando

- Si el ADS2 sigue marginal a 100 kHz, revisar cableado/pull-ups del bus antes de bajar más el reloj.
- Si el buckboost necesita no congelarse durante las lecturas del ADS2, el siguiente paso es mover el muestreo del ADS2 a su propia tarea/core con un mutex del bus Wire (refactor mayor).
- Si hace falta más margen en I2C, exponer `PV_SENSE_PERIOD_MS` (cadencia del ADS2) como parámetro configurable por CAN.