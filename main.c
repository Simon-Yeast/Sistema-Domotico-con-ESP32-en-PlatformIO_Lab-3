#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "LAB3";

// ============================================================
// PINES
// ============================================================
#define ADC_LM335_CH    ADC1_CHANNEL_7   // GPIO35
#define ADC_LDR_CH      ADC1_CHANNEL_6   // GPIO34
#define PIN_RELAY       GPIO_NUM_23
#define PIN_LED_PWM     GPIO_NUM_14
#define PIN_STEP_IN1    GPIO_NUM_16
#define PIN_STEP_IN2    GPIO_NUM_17
#define PIN_STEP_IN3    GPIO_NUM_18
#define PIN_STEP_IN4    GPIO_NUM_19

/*
 * ADC_VREF: Con ADC_ATTEN_DB_11 el ESP32 mide linealmente hasta ~3.1 V.
 * El LM335 alimentado a 3.3V da ~2.96V a 23°C ambiente.
 */
#define ADC_VREF 3.1f

/*
 * TEMP_OFFSET_C: offset de calibración en grados Celsius.
 * El ADC del ESP32 no es perfectamente lineal en voltajes altos (>2.5V),
 * introduciendo un error sistemático. Con LM335 a 3.3V marcando -12°C
 * cuando la real es ~23°C → offset necesario = +35°C.
 *
 * Para recalibrar:
 *   1. Pon TEMP_OFFSET_C a 0.0f y flashea
 *   2. Lee T_raw en serial a temperatura ambiente conocida
 *   3. TEMP_OFFSET_C = T_real - T_raw
 */
#define TEMP_OFFSET_C 35.0f

// LEDC
#define LEDC_TIMER_NUM   LEDC_TIMER_0
#define LEDC_SPEED_MODE  LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_N   LEDC_CHANNEL_0
#define LEDC_FREQ_HZ     5000
#define LEDC_DUTY_RES    LEDC_TIMER_8_BIT  // 0-255

// UART
#define UART_PORT_NUM    UART_NUM_0
#define UART_BAUD_RATE   115200

// Tiempo máximo de espera en la fase de configuración inicial (ms)
// 0 = esperar indefinidamente hasta que el usuario escriba SET_TEMP
// Si prefieres un timeout, cámbialo (ej: 30000 = 30 segundos)
#define INIT_CONFIG_TIMEOUT_MS 0

/*
 * Secuencia de MEDIO PASO (8 pasos) para 28BYJ-48
 * Más suave, más torque y menos vibración que paso completo.
 *   Horario     (g_step_dir=1):  índice 0→1→2→...→7→0
 *   Antihorario (g_step_dir=-1): índice 0→7→6→...→1→0
 */
#define STEP_SEQ_LEN 8
static const uint8_t STEP_SEQ[STEP_SEQ_LEN][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};

// ============================================================
// ESTADO GLOBAL
// ============================================================
static SemaphoreHandle_t g_mutex = NULL;
static volatile float g_target_temp = 25.0f;
static volatile int   g_step_dir   = 1;   // 1=horario, -1=antihorario
static volatile uint8_t g_led_duty = 0;
static int g_step_index = 0;
static volatile int g_step_speed = 0;     // steps/s actuales
static esp_timer_handle_t g_stepper_timer = NULL;

/*
 * Flag de sistema listo: mientras sea 0, control_task espera.
 * Se pone a 1 cuando el usuario confirma (o acepta) la Tc inicial.
 */
static volatile int g_system_ready = 0;

// ============================================================
// PROTOTIPOS
// ============================================================
static void init_gpio(void);
static void init_adc(void);
static void init_ledc(void);
static void init_uart(void);
static void init_stepper_timer(void);

static float read_temperature_celsius(void);
static float read_light_percent(void);
static void  control_temperature(float T, float Tc);
static void  control_lighting(float light_pct);

static void apply_stepper_outputs(int index);
static void stop_stepper_outputs(void);
static void update_stepper_timer(int steps_per_sec);
static void stepper_timer_callback(void *arg);
static void set_led_duty(uint8_t duty);
static void uart_send_str(const char *str);
static int  process_command(const char *line);

static void control_task(void *arg);
static void serial_task(void *arg);

// ============================================================
// app_main
// ============================================================
void app_main(void) {
    g_mutex = xSemaphoreCreateMutex();

    init_gpio();
    init_adc();
    init_ledc();
    init_uart();
    init_stepper_timer();

    uart_send_str("\r\n\r\n========================================\r\n");
    uart_send_str("   SISTEMA DOMOTICO - LABORATORIO 3\r\n");
    uart_send_str("========================================\r\n");
    uart_send_str("FASE DE CONFIGURACION INICIAL\r\n");
    uart_send_str("Ingrese la temperatura de control:\r\n");
    uart_send_str("  SET_TEMP:XX  (rango: 10 a 40 grados C)\r\n");
    uart_send_str("Ejemplo: SET_TEMP:23\r\n");
    uart_send_str("Una vez configurada, el sistema arranca.\r\n");
    uart_send_str("----------------------------------------\r\n\r\n");

    xTaskCreate(control_task, "control_task", 4096, NULL, 5, NULL);
    xTaskCreate(serial_task,  "serial_task",  4096, NULL, 4, NULL);
}

// ============================================================
// INICIALIZACIÓN
// ============================================================
static void init_gpio(void) {
    gpio_config_t relay_cfg = {
        .pin_bit_mask = 1ULL << PIN_RELAY,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&relay_cfg);
    gpio_set_level(PIN_RELAY, 0);

    gpio_config_t step_cfg = {
        .pin_bit_mask = (1ULL << PIN_STEP_IN1) | (1ULL << PIN_STEP_IN2) |
                        (1ULL << PIN_STEP_IN3) | (1ULL << PIN_STEP_IN4),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&step_cfg);
    stop_stepper_outputs();
}

static void init_adc(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_LM335_CH, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC_LDR_CH,   ADC_ATTEN_DB_11);
}

static void init_ledc(void) {
    ledc_timer_config_t t = {
        .speed_mode      = LEDC_SPEED_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num       = LEDC_TIMER_NUM,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);

    ledc_channel_config_t c = {
        .gpio_num   = PIN_LED_PWM,
        .speed_mode = LEDC_SPEED_MODE,
        .channel    = LEDC_CHANNEL_N,
        .timer_sel  = LEDC_TIMER_NUM,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&c);
}

static void init_uart(void) {
    uart_config_t cfg = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_PORT_NUM, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &cfg);
}

static void init_stepper_timer(void) {
    esp_timer_create_args_t args = {
        .callback = stepper_timer_callback,
        .name     = "stepper"
    };
    esp_timer_create(&args, &g_stepper_timer);
}

// ============================================================
// SENSORES
// ============================================================

/*
 * LM335: V_out = 10 mV/K
 *   T[K]  = V / 0.010
 *   T[°C] = T[K] - 273.15
 *
 * ADC 12 bits, referencia real ~3.1 V con ATTEN_DB_11.
 *
 * Si la temperatura medida sigue desviada, mide el voltaje real
 * del LM335 con multímetro y ajusta ADC_VREF hasta que coincida.
 * Fórmula de ajuste fino: ADC_VREF = V_medido * 4095 / raw_leido
 */
static float read_temperature_celsius(void) {
    // Promedio de 16 lecturas para reducir ruido ADC
    int sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += adc1_get_raw(ADC_LM335_CH);
    }
    float raw = (float)(sum / 16);
    float v   = raw * (ADC_VREF / 4095.0f);
    float t_k = v / 0.010f;          // LM335: 10 mV/K
    float t_c = t_k - 273.15f;
    return t_c + TEMP_OFFSET_C;      // Corrección de no-linealidad del ADC
}

/*
 * LDR conectado: VCC → LDR → nodo_ADC → R_pulldown → GND
 *
 * Mucha luz → R_LDR baja → más voltaje → raw alto → pct alto
 * Poca luz  → R_LDR alta → menos voltaje → raw bajo → pct bajo
 *
 * LDR_RAW_MIN: raw leído con el sensor completamente tapado (~28% de 4095)
 * LDR_RAW_MAX: raw leído con luz máxima (~99% de 4095)
 * Se mapea este rango real al 0–100% para usar todo el rango útil.
 *
 * Para recalibrar:
 *   1. Tapa completamente el sensor → anota el raw (LDR_RAW_MIN)
 *   2. Pon luz máxima (flash)       → anota el raw (LDR_RAW_MAX)
 *   raw = pct_reportada * 4095 / 100
 */
#define LDR_RAW_MIN 1147.0f   // ~28% de 4095 = sensor completamente tapado
#define LDR_RAW_MAX 4055.0f   // ~99% de 4095 = luz máxima (flash)

static float read_light_percent(void) {
    int sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += adc1_get_raw(ADC_LDR_CH);
    }
    float raw = (float)(sum / 16);

    // Mapear rango real [LDR_RAW_MIN, LDR_RAW_MAX] → [0, 100]
    // e invertir: tapar (raw alto) = 0% luz, flash (raw bajo) = 100% luz
    float pct = (raw - LDR_RAW_MIN) / (LDR_RAW_MAX - LDR_RAW_MIN) * 100.0f;
    pct = 100.0f - pct;   // Inversión: más raw = menos luz ambiente percibida

    // Clamp: nunca salir de [0, 100]
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    return pct;
}

// ============================================================
// LÓGICA DE CONTROL DE TEMPERATURA
// ============================================================
/*
 * Según enunciado:
 *   Tc-1 <= T <= Tc+1            → todo apagado
 *   T < Tc-1                     → calefacción ON,  motor horario     100 steps/s
 *   Tc+1 < T < Tc+3              → calefacción OFF, motor antihorario 100 steps/s
 *   Tc+3 <= T <= Tc+5            → calefacción OFF, motor antihorario 300 steps/s
 *   T > Tc+5                     → calefacción OFF, motor antihorario 600 steps/s
 */
static void control_temperature(float T, float Tc) {
    if (T >= (Tc - 1.0f) && T <= (Tc + 1.0f)) {
        // Zona confort: todo apagado
        gpio_set_level(PIN_RELAY, 0);
        update_stepper_timer(0);
    }
    else if (T < (Tc - 1.0f)) {
        // Demasiado frío: calefacción ON, motor horario lento
        gpio_set_level(PIN_RELAY, 1);
        g_step_dir = 1;
        update_stepper_timer(100);
    }
    else if (T > (Tc + 1.0f) && T < (Tc + 3.0f)) {
        // Ligeramente caliente: ventilación baja
        gpio_set_level(PIN_RELAY, 0);
        g_step_dir = -1;
        update_stepper_timer(100);
    }
    else if (T >= (Tc + 3.0f) && T <= (Tc + 5.0f)) {
        // Caliente: ventilación media
        gpio_set_level(PIN_RELAY, 0);
        g_step_dir = -1;
        update_stepper_timer(300);
    }
    else {
        // T > Tc+5: muy caliente, ventilación máxima
        gpio_set_level(PIN_RELAY, 0);
        g_step_dir = -1;
        update_stepper_timer(600);
    }
}

// ============================================================
// LÓGICA DE CONTROL DE ILUMINACIÓN
// ============================================================
/*
 *   ni < 20%            → LEDs al 100%  (duty 255)
 *   20% <= ni < 30%     → LEDs al  80%  (duty 204)
 *   30% <= ni < 40%     → LEDs al  60%  (duty 153)
 *   40% <= ni < 60%     → LEDs al  50%  (duty 128)
 *   60% <= ni < 80%     → LEDs al  30%  (duty  76)
 *   ni >= 80%           → LEDs apagados (duty   0)
 */
static void control_lighting(float light_pct) {
    uint8_t duty;
    if      (light_pct < 20.0f) duty = 255;
    else if (light_pct < 30.0f) duty = 204;
    else if (light_pct < 40.0f) duty = 153;
    else if (light_pct < 60.0f) duty = 128;
    else if (light_pct < 80.0f) duty = 76;
    else                        duty = 0;

    set_led_duty(duty);
    g_led_duty = duty;
}

// ============================================================
// STEPPER
// ============================================================
static void apply_stepper_outputs(int index) {
    gpio_set_level(PIN_STEP_IN1, STEP_SEQ[index][0]);
    gpio_set_level(PIN_STEP_IN2, STEP_SEQ[index][1]);
    gpio_set_level(PIN_STEP_IN3, STEP_SEQ[index][2]);
    gpio_set_level(PIN_STEP_IN4, STEP_SEQ[index][3]);
}

static void stop_stepper_outputs(void) {
    gpio_set_level(PIN_STEP_IN1, 0);
    gpio_set_level(PIN_STEP_IN2, 0);
    gpio_set_level(PIN_STEP_IN3, 0);
    gpio_set_level(PIN_STEP_IN4, 0);
}

static void stepper_timer_callback(void *arg) {
    g_step_index = (g_step_index + g_step_dir + STEP_SEQ_LEN) % STEP_SEQ_LEN;
    apply_stepper_outputs(g_step_index);
}

static void update_stepper_timer(int steps_per_sec) {
    esp_timer_stop(g_stepper_timer);
    g_step_speed = steps_per_sec;
    if (steps_per_sec > 0) {
        esp_timer_start_periodic(g_stepper_timer, 1000000ULL / steps_per_sec);
    } else {
        stop_stepper_outputs();
    }
}

// ============================================================
// LEDC / UART
// ============================================================
static void set_led_duty(uint8_t duty) {
    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_N, duty);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_N);
}

static void uart_send_str(const char *str) {
    uart_write_bytes(UART_PORT_NUM, str, strlen(str));
}

// ============================================================
// PROCESAR COMANDO SET_TEMP
// Retorna 1 si el comando fue válido, 0 en caso contrario.
// ============================================================
static int process_command(const char *line) {
    if (strlen(line) > 9 && strncmp(line, "SET_TEMP:", 9) == 0) {
        float nt = atof(line + 9);
        if (nt >= 10.0f && nt <= 40.0f) {
            if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                g_target_temp = nt;
                xSemaphoreGive(g_mutex);
            }
            char r[64];
            snprintf(r, sizeof(r), "\r\n>> Temperatura de control: %.1f C\r\n", nt);
            uart_send_str(r);
            return 1;
        } else {
            uart_send_str("\r\n>> ERROR: valor fuera de rango (10-40 C)\r\n");
        }
    } else {
        uart_send_str("\r\n>> Comando no reconocido. Use: SET_TEMP:XX\r\n");
    }
    return 0;
}

// ============================================================
// TAREA DE CONTROL
// ============================================================
static void control_task(void *arg) {
    // Esperar a que el usuario configure la Tc antes de arrancar
    while (!g_system_ready) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    uart_send_str("\r\n[SISTEMA] Control activo.\r\n");
    uart_send_str("[SISTEMA] Puede cambiar Tc en cualquier momento con SET_TEMP:XX\r\n\r\n");

    char buf[160];
    while (1) {
        float T, Tc, L;

        T = read_temperature_celsius();
        L = read_light_percent();

        if (xSemaphoreTake(g_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            Tc = g_target_temp;
            xSemaphoreGive(g_mutex);
        } else {
            Tc = 25.0f;
        }

        control_temperature(T, Tc);
        control_lighting(L);

        // Duty → porcentaje legible
        uint8_t duty_pct = (uint8_t)(((uint32_t)g_led_duty * 100) / 255);

        // L está invertida internamente para que la lógica de LEDs sea correcta.
        // Para mostrar en serial se invierte de vuelta: flash=100%, tapado=0%
        float L_display = 100.0f - L;

        snprintf(buf, sizeof(buf),
            "[ESTADO] Tc=%.1fC | T=%.2fC | Luz=%.1f%% | LED=%d%% | Rele=%s | Motor=%s %d steps/s\r\n",
            Tc, T, L_display, duty_pct,
            gpio_get_level(PIN_RELAY) ? "ON" : "OFF",
            g_step_dir == 1 ? "CW" : "CCW",
            g_step_speed);
        uart_send_str(buf);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================
// TAREA SERIAL
//   - Fase 1: espera SET_TEMP inicial (bloquea el sistema)
//   - Fase 2: acepta SET_TEMP en cualquier momento
// ============================================================
static void serial_task(void *arg) {
    uint8_t data[256];
    char    line[64];
    int     idx = 0;

    while (1) {
        int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data) - 1,
                            pdMS_TO_TICKS(100));
        if (len <= 0) continue;

        for (int i = 0; i < len; i++) {
            char c = (char)data[i];
            // Eco local
            uart_write_bytes(UART_PORT_NUM, &c, 1);

            if (c == '\r' || c == '\n') {
                line[idx] = '\0';
                if (idx > 0) {
                    int ok = process_command(line);
                    // En la fase de configuración inicial, arrancar si el comando fue válido
                    if (!g_system_ready && ok) {
                        g_system_ready = 1;
                    }
                }
                idx = 0;
            } else if (idx < 63) {
                line[idx++] = c;
            }
        }
    }
}