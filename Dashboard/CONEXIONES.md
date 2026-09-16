# Microrred DC — Conexiones Físicas

Placa: **Adafruit Feather ESP32** (featheresp32)

---

## Resumen de periféricos

| Periférico       | Interfaz | Descripción                              |
|------------------|----------|------------------------------------------|
| ADS1115          | I2C      | ADC de 16 bits para medir voltajes       |
| MCP2515          | SPI      | Controlador CAN Bus 500 kbps             |
| Conversor Buck-Boost | PWM  | Dos señales de gate para los MOSFET      |

---

## 1. ADS1115 — Sensor de voltaje (I2C)

| Pin ADS1115 | Pin ESP32 | Descripción                  |
|-------------|-----------|------------------------------|
| VDD         | 3.3V      | Alimentación                 |
| GND         | GND       | Tierra                       |
| SDA         | GPIO 21   | Datos I2C                    |
| SCL         | GPIO 22   | Reloj I2C                    |
| ADDR        | GND       | Dirección I2C = 0x48         |
| ALRT        | —         | No conectado                 |

### Entradas analógicas del ADS1115

| Canal ADS | Variable medida       | Descripción                              |
|-----------|-----------------------|------------------------------------------|
| A0        | —                     | No usado (reservado)                     |
| A1        | V_AG1 (entrada)       | Voltaje en el nodo de entrada del conversor  |
| A2        | V_BUS (salida/bus)    | Voltaje en el bus DC / salida del conversor  |
| A3        | —                     | No usado (reservado)                     |

### Divisores de voltaje

Los canales A1 y A2 **no pueden conectarse directamente** al nodo de alta tensión
porque el ADS1115 solo admite hasta 4.096 V en modo single-ended (con PGA_4096).
Se usa un divisor resistivo para escalar el voltaje antes del ADS.

```
Nodo de alta tensión
        │
       [R1]
        │
        ├──── ADS1115 A1 o A2
        │
       [R2]
        │
       GND
```

El factor de divisor se configura en el firmware como `divFactorA0` y `divFactorA1`
(valor por defecto: **3.05**), y se puede calibrar desde la página web.

**Ejemplo con divisor 3:1** (R1 = 20kΩ, R2 = 10kΩ):
- Factor teórico = (R1 + R2) / R2 = 3.0
- Rango medible = 4.096 V × 3.0 = **12.3 V máximo**

---

## 2. MCP2515 — Módulo CAN Bus (SPI / VSPI)

El MCP2515 debe operar con un **cristal de 8 MHz** para que el bitrate
de 500 kbps configurado en el firmware sea correcto.

| Pin MCP2515 | Pin ESP32 | Descripción           |
|-------------|-----------|-----------------------|
| VCC         | 3.3V      | Alimentación (¹)      |
| GND         | GND       | Tierra                |
| SCK         | GPIO 18   | Reloj SPI             |
| SI (MOSI)   | GPIO 23   | Datos SPI entrada     |
| SO (MISO)   | GPIO 19   | Datos SPI salida      |
| CS          | GPIO 5    | Chip Select (activo bajo) |
| INT         | —         | No usado en este firmware |

> **(¹)** Algunos módulos MCP2515 de breakout incluyen un transceptor TJA1050
> que opera a **5V**. En ese caso alimentar VCC con 5V del Feather (pin USB/BAT)
> y usar un **divisor de voltaje o level shifter** en la línea MISO para bajar
> de 5V a 3.3V antes de llegar al GPIO 19 del ESP32. SCK, MOSI y CS pueden
> conectarse directamente porque el ESP32 en salida tolera la recepción a 5V
> en esas líneas, pero **MISO no**.

### Bus CAN

| Pin módulo CAN | Conexión              |
|----------------|-----------------------|
| CANH           | Bus CAN (línea High)  |
| CANL           | Bus CAN (línea Low)   |

Conectar **resistencia de terminación de 120 Ω** entre CANH y CANL en
cada extremo del bus (normalmente ya incluida en los módulos de breakout).

---

## 3. Conversor Buck-Boost — Señales PWM

Frecuencia PWM: **10 kHz** (periodo de 100 µs, configurable vía `pwmPeriodUs`).
Resolución: **10 bits** (1023 pasos).

| Señal    | Pin ESP32 | Canal LEDC | Descripción                          |
|----------|-----------|------------|--------------------------------------|
| HIGH PWM | GPIO 25   | Canal 0    | Gate del MOSFET high-side            |
| LOW PWM  | GPIO 26   | Canal 1    | Gate del MOSFET low-side             |

### Lógica de salida por topología

Las señales **no van directamente** al gate de los MOSFET. Se requiere un
**driver de gate** (ej. IR2110, TC4420, o similar) entre el ESP32 y los MOSFET.

| Topología | HIGH PWM (GPIO 25)         | LOW PWM (GPIO 26)         |
|-----------|----------------------------|---------------------------|
| BOOST     | Fijo en HIGH (MOSFET OFF)  | Señal PWM activa          |
| BUCK      | Señal PWM invertida        | Fijo en LOW (MOSFET OFF)  |
| OFF       | Fijo en HIGH (MOSFET OFF)  | Fijo en LOW (MOSFET OFF)  |

> En **modo BOOST**, el MOSFET controlado es el de low-side (GPIO 26).
> En **modo BUCK**, el MOSFET controlado es el de high-side (GPIO 25),
> con la señal invertida respecto al duty cycle calculado.

---

## 4. Alimentación

| Pin Feather ESP32 | Descripción                                    |
|-------------------|------------------------------------------------|
| USB               | 5V desde USB — para programar y alimentar      |
| BAT               | 3.7–4.2V desde LiPo (opcional)                |
| 3V                | Salida regulada 3.3V — para ADS1115 y MCP2515  |
| GND               | Tierra común de todo el sistema                |

---

## 5. Diagrama de conexión completo

```
                    ┌─────────────────────────────┐
                    │      Feather ESP32           │
                    │                             │
    ┌───────────┐   │  GPIO 21 (SDA) ─────────────┼──── SDA ┐
    │  ADS1115  │   │  GPIO 22 (SCL) ─────────────┼──── SCL │
    │           │   │  3V3           ─────────────┼──── VDD │ ADS1115
    │  A1 ──────┼───┤  (divisor R1/R2)            │    GND ─┼──── GND
    │  A2 ──────┼───┤  (divisor R1/R2)            │         ┘
    └───────────┘   │                             │
                    │  GPIO 18 (SCK) ─────────────┼──── SCK  ┐
    ┌───────────┐   │  GPIO 19 (MISO)─────────────┼──── SO   │
    │  MCP2515  │   │  GPIO 23 (MOSI)─────────────┼──── SI   │ MCP2515
    │  + TJA1050│   │  GPIO  5 (CS)  ─────────────┼──── CS   │
    │           │   │  3V3 (o 5V) ────────────────┼──── VCC  │
    │  CANH ────┼───┤── Bus CAN                   │    GND ──┼──── GND
    │  CANL ────┼───┤── Bus CAN                   │          ┘
    └───────────┘   │                             │
                    │  GPIO 25 ───────────────────┼──── Gate Driver ── MOSFET High
                    │  GPIO 26 ───────────────────┼──── Gate Driver ── MOSFET Low
                    │                             │
                    │  GND ───────────────────────┼──── GND común del sistema
                    └─────────────────────────────┘
```

---

## 6. Consideraciones de diseño

**Tierra común:** la GND del ESP32, ADS1115, MCP2515 y el lado de baja tensión
del conversor deben compartir una misma referencia de tierra. Ruido en la tierra
puede afectar las mediciones del ADS.

**Desacoplamiento:** colocar un condensador de 100 nF lo más cerca posible del
pin VDD de cada CI (ADS1115 y MCP2515) para filtrar ruido de alta frecuencia.

**Aislamiento:** en aplicaciones con voltajes de bus > 12V se recomienda usar
un optoacoplador o isolador digital (ej. ISO7241) entre el ESP32 y el driver
de gate, y un aislador I2C (ej. ISO1541) antes del ADS1115.

**Longitud de cables SPI:** mantener los cables del bus SPI (SCK, MOSI, MISO, CS)
lo más cortos posible (< 10 cm) para evitar reflexiones a 8 MHz.
