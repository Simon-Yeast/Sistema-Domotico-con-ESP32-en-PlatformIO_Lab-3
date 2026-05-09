# Sistema Domótico con ESP32 — Laboratorio 3

Sistema embebido para el control automático de **temperatura** e **iluminación** de una habitación, implementado sobre un ESP32 (AZ-Delivery DevKit V4) en lenguaje C con el framework **ESP-IDF** y orquestado mediante **PlatformIO**.

La temperatura de control se configura por consola serial mediante el comando `SET_TEMP:XX`. El sistema lee continuamente la temperatura y la iluminación ambiente, y actúa sobre tres elementos: una bombilla incandescente (calefactor), un motor paso a paso bipolar (ventilación) y dos LEDs de potencia (iluminación) regulados por PWM.

---

## Componentes

| Categoría | Elemento | Cantidad |
|-----------|----------|----------|
| Microcontrolador | ESP32 (AZ-Delivery DevKit V4) | 1 |
| Sensor térmico | LM35 | 1 |
| Sensor lumínico | Fotoresistencia (LDR) | 1 |
| Calefactor | Bombilla incandescente 12 V<sub>AC</sub> – 20 W | 1 |
| Ventilación | Motor paso a paso bipolar | 1 |
| Iluminación | LED de potencia 3.2 V – 3 W | 2 |
| Etapa de potencia | Relé + transistor NPN (driver de bombilla) | 1 |
| Etapa de potencia | Puente H (driver del motor) | 1 |
| Etapa de potencia | MOSFET de potencia (driver de LEDs) | 1 |
| Protección | Diodo flyback antiparalelo a la bobina del relé | 1 |
| Resistencias | 560 Ω (base BJT), 10 kΩ (pull-down BJT), pull-down LDR | varias |
| Alimentación | Fuente DC 12 V para el lado de potencia | 1 |

---

## Asignación de pines

| Función | GPIO | Tipo |
|---------|------|------|
| Relé / bombilla | **GPIO 18** | Salida digital |
| Motor IN1 | **GPIO 19** | Salida digital |
| Motor IN2 | **GPIO 21** | Salida digital |
| Motor IN3 | **GPIO 22** | Salida digital |
| Motor IN4 | **GPIO 23** | Salida digital |
| LEDs de potencia (PWM) | **GPIO 14** | Salida LEDC |
| LM35 | **GPIO 35** | Entrada ADC1_CH7 |
| Fotoresistencia | **GPIO 34** | Entrada ADC1_CH6 |
| UART de control (USB) | GPIO 1 / GPIO 3 | TX0 / RX0 |

---

## Lógica de control

### Temperatura

Sea **T** la temperatura medida y **Tc** la de control:

| Condición | Bombilla | Motor | Frecuencia |
|-----------|----------|-------|------------|
| `Tc − 1 ≤ T ≤ Tc + 1` | OFF | detenido | — |
| `T < Tc − 1` | ON | horario (calefacción) | 100 steps/s |
| `Tc + 1 < T < Tc + 3` | OFF | antihorario (ventilación) | 100 steps/s |
| `Tc + 3 ≤ T ≤ Tc + 5` | OFF | antihorario | 300 steps/s |
| `T > Tc + 5` | OFF | antihorario | 600 steps/s |

El motor se controla mediante una secuencia *full-step* de 4 fases con dos bobinas siempre energizadas, conmutada por un timer de hardware (`esp_timer`) cuya frecuencia se ajusta al *step rate* requerido.

### Iluminación

Sea **ni** el nivel de iluminación ambiente medido por la fotoresistencia (0 % a 100 %):

| Condición | Brillo de los LEDs | Duty (8 bits) |
|-----------|--------------------|---------------|
| `ni < 20 %` | 100 % | 255 |
| `20 % ≤ ni < 30 %` | 80 % | 204 |
| `30 % ≤ ni < 40 %` | 60 % | 153 |
| `40 % ≤ ni < 60 %` | 50 % | 128 |
| `60 % ≤ ni < 80 %` | 30 % | 76 |
| `ni ≥ 80 %` | 0 % | 0 |

La señal PWM se genera con el módulo **LEDC** del ESP32 a 5 kHz y 8 bits de resolución.

---

## Comunicación serial

- **Velocidad:** 115 200 baud, 8N1
- **Comando:** `SET_TEMP:XX` donde `XX` es la temperatura de control deseada en grados Celsius (rango admitido: 10 °C – 40 °C).

Hasta que no se reciba un `SET_TEMP:XX` válido, el sistema permanece en *fase de configuración inicial* y no acciona ningún elemento. Una vez configurada Tc, el sistema arranca y reporta su estado cada segundo en el formato:

```
[ESTADO] Tc=23.0C | T=22.85C | Luz=58.2% | LED=50% | Rele=OFF | Motor=CCW 100 steps/s
```

La temperatura de control puede modificarse en cualquier momento durante la operación reenviando un nuevo `SET_TEMP:XX`.

---

## Compilación y flasheo

Requisitos:

- [PlatformIO](https://platformio.org/) instalado (extensión de VS Code o CLI).
- Cable USB conectado al ESP32.

```bash
# Compilar
pio run

# Flashear al ESP32
pio run --target upload

# Abrir el monitor serial
pio device monitor
```

Configuración de PlatformIO (`platformio.ini`):

```ini
[env:az-delivery-devkit-v4]
platform = espressif32
board = az-delivery-devkit-v4
framework = espidf
monitor_speed = 115200
```

---

## Estructura del proyecto

```
.
├── platformio.ini
├── src/
│   └── main.c
└── README.md
```


```
   |\---/|
   | ,_, |
    \_`_/-..----.
 ___/ `   ' ,""+ \  
(__...'   __\    |`.___.';
  (_,...'(_,.`__)/'.....+
```
