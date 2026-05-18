#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include <math.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "mpu6050.h"

#include "Fusion.h"

// --- Periodo de amostragem (100 Hz) ---
#define SAMPLE_PERIOD (0.01f)

// --- I2C / MPU6050 ---
#define MPU_ADDRESS    0x68
#define I2C_SDA_GPIO   4
#define I2C_SCL_GPIO   5

// --- LED RGB (PWM) ---
#define LED_R_PIN      16
#define LED_G_PIN      17
#define LED_B_PIN      18
#define PWM_WRAP       255   // resolucao de 8 bits para os canais RGB

// --- Botao (mouse click manual) ---
#define BTN_PIN        14

// --- Pacote serial: SYNC | AXIS | LSB | MSB ---
#define SYNC_BYTE      0xFF
#define AXIS_X         0
#define AXIS_Y         1
#define AXIS_CLICK     2

// --- Parametros de movimento (yaw/pitch -> mouse) ---
#define MOUSE_GAIN     8       // multiplicador do delta de angulo
#define DEAD_ZONE      2       // graus/s abaixo disso vira 0
#define MOUSE_LIMIT    255

// --- Deteccao de click por gesto ("cutucar o ar") ---
// Aceleracao em g (1g = 9.8 m/s^2). Limiar para detectar empurrao no eixo X.
#define CLICK_THRESHOLD   1.5f
#define CLICK_DEBOUNCE_MS 500

#define PIN_MPU_TASK     6
#define PIN_FUSION_TASK  7
#define PIN_PWM_TASK     8
#define PIN_UART_TASK    9

// --- Suavizacao do LED (media movel da cor) ---
#define COLOR_WINDOW   5

// Struct dos dados crus da MPU enviada pela mpu_task
typedef struct mpu_data {
    int16_t accel[3];
    int16_t gyro[3];
} mpu_data_t;

// Struct de cor RGB enviada pela fusion_task para a pwm_task
typedef struct color {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} color_t;

// Struct de posicao do mouse (delta de movimento) enviada para a uart_task
typedef struct pos {
    int axis;
    int val;
} pos_t;

// Recursos compartilhados do FreeRTOS (globais por necessidade arquitetural)
QueueHandle_t xQueueMPU;
QueueHandle_t xQueueColor;
QueueHandle_t xQueuePos;
SemaphoreHandle_t xSemaphoreBtn;

// Inicializa I2C e tira a MPU6050 do modo sleep
static void mpu6050_init(void) {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

// Le 14 bytes consecutivos a partir de 0x3B: 6 acel + 2 temp + 6 gyro
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[14];
    uint8_t reg = 0x3B;

    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 14, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (int16_t)((buffer[i * 2] << 8) | buffer[(i * 2) + 1]);
    }
    *temp = (int16_t)((buffer[6] << 8) | buffer[7]);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (int16_t)((buffer[8 + i * 2] << 8) | buffer[8 + (i * 2) + 1]);
    }
}

// Configura um pino como saida PWM com wrap de 8 bits
static void pwm_setup_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_gpio_level(pin, 0);
    pwm_set_enabled(slice, true);
}

// ISR do botao: libera o semaforo para a fusion_task
static void btn_isr(uint gpio, uint32_t events) {
    (void)gpio;
    (void)events;
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(xSemaphoreBtn, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

// Le a MPU a 100 Hz e publica os dados crus em xQueueMPU



static void metrics_gpio_init(void) {
    gpio_init(PIN_MPU_TASK);
    gpio_set_dir(PIN_MPU_TASK, GPIO_OUT);
    gpio_put(PIN_MPU_TASK, 0);

    gpio_init(PIN_FUSION_TASK);
    gpio_set_dir(PIN_FUSION_TASK, GPIO_OUT);
    gpio_put(PIN_FUSION_TASK, 0);

    gpio_init(PIN_PWM_TASK);
    gpio_set_dir(PIN_PWM_TASK, GPIO_OUT);
    gpio_put(PIN_PWM_TASK, 0);

    gpio_init(PIN_UART_TASK);
    gpio_set_dir(PIN_UART_TASK, GPIO_OUT);
    gpio_put(PIN_UART_TASK, 0);
}


void mpu_task(void *p) {
    mpu6050_init();

    while (true) {
        gpio_put(PIN_MPU_TASK, 1);

        mpu_data_t data;
        int16_t temp;
        mpu6050_read_raw(data.accel, data.gyro, &temp);
        xQueueSend(xQueueMPU, &data, 0);

        gpio_put(PIN_MPU_TASK, 0);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Recebe dados crus, faz fusao com a Fusion lib, gera deltas de mouse,
// detecta click por gesto e mapeia orientacao em cor RGB.
void fusion_task(void *p) {
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    mpu_data_t data;
    float last_yaw = 0.0f;
    float last_pitch = 0.0f;
    bool first_sample = true;

    TickType_t last_click_tick = 0;

    int r_buffer[COLOR_WINDOW] = {0};
    int g_buffer[COLOR_WINDOW] = {0};
    int b_buffer[COLOR_WINDOW] = {0};
    int color_idx = 0;

    while (true) {
        if (xQueueReceive(xQueueMPU, &data, portMAX_DELAY)) {
            gpio_put(PIN_FUSION_TASK, 1);
            // Conversao para unidades fisicas (escalas padrao da MPU6050)
            FusionVector gyroscope = {
                .axis.x = data.gyro[0] / 131.0f,
                .axis.y = data.gyro[1] / 131.0f,
                .axis.z = data.gyro[2] / 131.0f,
            };
            FusionVector accelerometer = {
                .axis.x = data.accel[0] / 16384.0f,
                .axis.y = data.accel[1] / 16384.0f,
                .axis.z = data.accel[2] / 16384.0f,
            };

            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
            FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

            // --- Movimento do mouse a partir do delta de yaw/pitch ---
            if (first_sample) {
                last_yaw = euler.angle.yaw;
                last_pitch = euler.angle.pitch;
                first_sample = false;
            } else {
                float dyaw = euler.angle.yaw - last_yaw;
                float dpitch = euler.angle.pitch - last_pitch;
                last_yaw = euler.angle.yaw;
                last_pitch = euler.angle.pitch;

                int dx = (int)(dyaw * MOUSE_GAIN);
                int dy = (int)(dpitch * MOUSE_GAIN);

                if (dx > -DEAD_ZONE && dx < DEAD_ZONE) dx = 0;
                if (dy > -DEAD_ZONE && dy < DEAD_ZONE) dy = 0;
                if (dx > MOUSE_LIMIT)  dx = MOUSE_LIMIT;
                if (dx < -MOUSE_LIMIT) dx = -MOUSE_LIMIT;
                if (dy > MOUSE_LIMIT)  dy = MOUSE_LIMIT;
                if (dy < -MOUSE_LIMIT) dy = -MOUSE_LIMIT;

                if (dx != 0) {
                    pos_t px = {.axis = AXIS_X, .val = dx};
                    xQueueSend(xQueuePos, &px, 0);
                }
                if (dy != 0) {
                    pos_t py = {.axis = AXIS_Y, .val = dy};
                    xQueueSend(xQueuePos, &py, 0);
                }
            }

            // --- Deteccao de click por gesto: pico de aceleracao no eixo X ---
            float acc_x = data.accel[0] / 16384.0f;
            TickType_t now = xTaskGetTickCount();
            TickType_t since_last = now - last_click_tick;
            if (acc_x > CLICK_THRESHOLD && since_last > pdMS_TO_TICKS(CLICK_DEBOUNCE_MS)) {
                last_click_tick = now;
                xSemaphoreGive(xSemaphoreBtn);
            }

            // --- Mapa orientacao -> cor RGB (com media movel) ---
            // pitch positivo -> verde, roll negativo -> azul, roll positivo -> vermelho
            int r_raw = (int)(euler.angle.roll * 4.0f);
            int b_raw = (int)(-euler.angle.roll * 4.0f);
            int g_raw = (int)(euler.angle.pitch * 4.0f);
            if (r_raw < 0) r_raw = 0;
            if (b_raw < 0) b_raw = 0;
            if (g_raw < 0) g_raw = 0;
            if (r_raw > 255) r_raw = 255;
            if (g_raw > 255) g_raw = 255;
            if (b_raw > 255) b_raw = 255;

            r_buffer[color_idx] = r_raw;
            g_buffer[color_idx] = g_raw;
            b_buffer[color_idx] = b_raw;
            color_idx = (color_idx + 1) % COLOR_WINDOW;

            int r_sum = 0, g_sum = 0, b_sum = 0;
            for (int i = 0; i < COLOR_WINDOW; i++) {
                r_sum += r_buffer[i];
                g_sum += g_buffer[i];
                b_sum += b_buffer[i];
            }
            color_t c = {
                .r = (uint8_t)(r_sum / COLOR_WINDOW),
                .g = (uint8_t)(g_sum / COLOR_WINDOW),
                .b = (uint8_t)(b_sum / COLOR_WINDOW),
            };
            xQueueSend(xQueueColor, &c, 0);
            gpio_put(PIN_FUSION_TASK, 0);
        }
    }
}

// Recebe cor da xQueueColor e atualiza os 3 canais PWM do LED RGB
void pwm_task(void *p) {
    pwm_setup_pin(LED_R_PIN);
    pwm_setup_pin(LED_G_PIN);
    pwm_setup_pin(LED_B_PIN);

    color_t c;
    while (true) {
        if (xQueueReceive(xQueueColor, &c, portMAX_DELAY)) {
            gpio_put(PIN_PWM_TASK, 1);

            pwm_set_gpio_level(LED_R_PIN, c.r);
            pwm_set_gpio_level(LED_G_PIN, c.g);
            pwm_set_gpio_level(LED_B_PIN, c.b);

            gpio_put(PIN_PWM_TASK, 0);
        }
    }
}
// Drena xQueuePos (posicao) e xSemaphoreBtn (click) e envia pacotes serializados
// no formato: SYNC (0xFF) | AXIS | LSB | MSB
void uart_task(void *p) {
    pos_t pos_data;
    while (true) {
        gpio_put(PIN_UART_TASK, 1);

        if (xSemaphoreTake(xSemaphoreBtn, 0) == pdTRUE) {
            putchar_raw(SYNC_BYTE);
            putchar_raw((uint8_t)AXIS_CLICK);
            putchar_raw(1);
            putchar_raw(0);
        }

        if (xQueueReceive(xQueuePos, &pos_data, pdMS_TO_TICKS(10)) == pdTRUE) {
            int16_t v16 = (int16_t)pos_data.val;
            uint8_t lsb = (uint8_t)(v16 & 0xFF);
            uint8_t msb = (uint8_t)((v16 >> 8) & 0xFF);
            putchar_raw(SYNC_BYTE);
            putchar_raw((uint8_t)pos_data.axis);
            putchar_raw(lsb);
            putchar_raw(msb);
        }

        gpio_put(PIN_UART_TASK, 0);
    }
}

int main() {
    stdio_init_all();

    // Botao com pull-up interno + IRQ na borda de descida
    gpio_init(BTN_PIN);
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_pull_up(BTN_PIN);
    gpio_set_irq_enabled_with_callback(BTN_PIN, GPIO_IRQ_EDGE_FALL, true, &btn_isr);

    // Filas e semaforo
    xQueueMPU     = xQueueCreate(16, sizeof(mpu_data_t));
    xQueueColor   = xQueueCreate(8,  sizeof(color_t));
    xQueuePos     = xQueueCreate(32, sizeof(pos_t));
    xSemaphoreBtn = xSemaphoreCreateBinary();

    xTaskCreate(mpu_task,    "mpu",    8192, NULL, 1, NULL);
    xTaskCreate(fusion_task, "fusion", 8192, NULL, 1, NULL);
    xTaskCreate(pwm_task,    "pwm",    4096, NULL, 1, NULL);
    xTaskCreate(uart_task,   "uart",   4096, NULL, 2, NULL);

    vTaskStartScheduler();

    while (true) {
        ;
    }
}