#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <math.h>
#include <zephyr/drivers/uart.h>
#include <stdio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>

/* ================= UART ================= */

#define UART_RS485_NODE DT_NODELABEL(usart1)
#define UART_SERIAL_NODE DT_NODELABEL(usart2)

static const struct device *uart_rs485 =
    DEVICE_DT_GET(UART_RS485_NODE);

static const struct device *uart_serial =
    DEVICE_DT_GET(UART_SERIAL_NODE);

/* ================= RS485 DE ================= */

#define RS485_DE_NODE DT_NODELABEL(gpioa)
#define RS485_DE_PIN 12

static const struct device *gpioa = DEVICE_DT_GET(RS485_DE_NODE);

/* ================= ADC ================= */

static const struct device *adc_pt1000 =
    DEVICE_DT_GET(DT_NODELABEL(ads1220_0));

static const struct device *adc_lm35 =
    DEVICE_DT_GET(DT_NODELABEL(ads1220_1));

static const struct device *adc_stm32 =
    DEVICE_DT_GET(DT_NODELABEL(adc1));

/* ================= PWM ================= */

#define PWM_NODE DT_NODELABEL(pwm2)

static const struct device *pwm_dev =
    DEVICE_DT_GET(PWM_NODE);

/* ================= INA219 ================= */

static const struct device *ina219_0 =
    DEVICE_DT_GET(DT_NODELABEL(ina219_0));

static const struct device *ina219_1 =
    DEVICE_DT_GET(DT_NODELABEL(ina219_1));

/* ================= ADS1220 CONST ================= */

#define ADS1220_FULL_SCALE 8388607.0f
#define ADS1220_VREF_V     2.048f

#define ADS1220_PT_GAIN    4.0f
#define RREF_OHM           5085.0f

#define PT1000_R0 1000.0f
#define PT_A 3.9083e-3f
#define PT_B -5.775e-7f

#define ADS1220_LM35_GAIN  2.0f
#define LM35_MV_PER_C      10.0f

/* ================= RS485 RX ================= */

static char rx_buffer[32];
static int rx_pos = 0;

static int current_value = 0;

/* ================= CONTROL ================= */

static float target_temp = 37.0;
static bool control_enabled = false;

/* ================= RS485 SEND ================= */

void rs485_send(const char *data)
{
    gpio_pin_set(gpioa, RS485_DE_PIN, 1);

    while (*data) {
        uart_poll_out(uart_rs485, *data++);
    }

    while (!uart_irq_tx_complete(uart_rs485)) {}

    gpio_pin_set(gpioa, RS485_DE_PIN, 0);
}

/* ================= UART RX CALLBACK ================= */

static void uart_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(dev))
        return;

    while (uart_irq_rx_ready(dev)) {

        if (uart_fifo_read(dev, &c, 1) <= 0)
            return;

        if (c == '\n' || c == '\r') {

            rx_buffer[rx_pos] = 0;

            if (rx_pos > 0) {
                current_value = atoi(rx_buffer);
                target_temp = current_value / 10.0f;
                control_enabled = true;
            }

            rx_pos = 0;
            return;
        }

        if (rx_pos < sizeof(rx_buffer) - 1) {
            rx_buffer[rx_pos++] = c;
        }
    }
}

/* ================= SERIAL PRINT ================= */

void serial_send(const char *str)
{
    while (*str) {
        uart_poll_out(uart_serial, *str++);
    }
}

/* ================= MAIN ================= */

int main(void)
{
    printk("SYSTEM START\n");

    gpio_pin_configure(gpioa, RS485_DE_PIN, GPIO_OUTPUT);
    gpio_pin_set(gpioa, RS485_DE_PIN, 0);

    uart_irq_callback_user_data_set(uart_rs485, uart_cb, NULL);
    uart_irq_rx_enable(uart_rs485);

    int32_t buf_pt, buf_lm;
    int16_t stm32_buf;

    struct adc_sequence seq_stm32 = {
        .channels = BIT(16),
        .buffer = &stm32_buf,
        .buffer_size = sizeof(stm32_buf),
        .resolution = 12,
    };

    int64_t last_rs485 = 0;

    while (1) {

        uint32_t period = 10000000;

        /* ================= STM32 ADC ================= */

        if (adc_read(adc_stm32, &seq_stm32) == 0) {

            float voltage = (stm32_buf * 3.3f) / 4095.0f;

            printf("STM32 ADC raw: %d\n", stm32_buf);
            printf("Voltage: %.3f V\n", voltage);
        }

        /* ================= ADS1220 ================= */

        ads1220_trigger(adc_pt1000);
        ads1220_trigger(adc_lm35);

        ads1220_fetch(adc_pt1000, &buf_pt);
        ads1220_fetch(adc_lm35, &buf_lm);

        /* ================= PT1000 ================= */

        float vin_pt = ((float)buf_pt / ADS1220_FULL_SCALE) *
                       (ADS1220_VREF_V / ADS1220_PT_GAIN);

        float r_rtd = ((float)buf_pt * RREF_OHM /
                      (-ADS1220_PT_GAIN)) /
                      ADS1220_FULL_SCALE;

        float ratio = r_rtd / PT1000_R0;

        float temp_pt = (-PT_A + sqrtf(PT_A * PT_A -
                         4 * PT_B * (1 - ratio))) /
                         (2 * PT_B);

        printf("PT1000 Temp: %.3f C\n", temp_pt);

        /* ================= PWM CONTROL ================= */

        float pwm_percent = 70;

        if (control_enabled) {

            float error = target_temp - temp_pt;

            if (error > 0) {
                pwm_percent = error * 10.0;
                if (pwm_percent > 100)
                    pwm_percent = 100;
            } else {
                pwm_percent = 0;
            }
        }

        uint32_t pulse0 = (period * pwm_percent) / 100;
        uint32_t pulse1 = (period * 20) / 100;

        pwm_set(pwm_dev, 2, period, pulse0, 0);
        pwm_set(pwm_dev, 4, period, pulse1, 0);

        printf("PWM: %.1f %%\n", pwm_percent);

        /* ================= LM35 ================= */

        float vin_lm = ((float)buf_lm / ADS1220_FULL_SCALE) *
                       (ADS1220_VREF_V / ADS1220_LM35_GAIN);

        float temp_lm = (vin_lm * 1000.0f) / LM35_MV_PER_C;

        printf("LM35 Temp: %.3f C\n\n", temp_lm);

        /* ================= INA219 ================= */

        struct sensor_value bus_voltage;
        struct sensor_value current;
        struct sensor_value power;

        if (sensor_sample_fetch(ina219_0) == 0) {

            sensor_channel_get(ina219_0,
                               SENSOR_CHAN_VOLTAGE,
                               &bus_voltage);

            sensor_channel_get(ina219_0,
                               SENSOR_CHAN_CURRENT,
                               &current);

            sensor_channel_get(ina219_0,
                               SENSOR_CHAN_POWER,
                               &power);

            char buf[128];

            snprintf(buf, sizeof(buf),
                     "PWR0 %d.%06dV %d.%06dA %d.%06dW\r\n",
                     bus_voltage.val1, bus_voltage.val2,
                     current.val1, current.val2,
                     power.val1, power.val2);

            serial_send(buf);
        }

        if (sensor_sample_fetch(ina219_1) == 0) {

            sensor_channel_get(ina219_1,
                               SENSOR_CHAN_VOLTAGE,
                               &bus_voltage);

            sensor_channel_get(ina219_1,
                               SENSOR_CHAN_CURRENT,
                               &current);

            sensor_channel_get(ina219_1,
                               SENSOR_CHAN_POWER,
                               &power);

            char buf[128];

            snprintf(buf, sizeof(buf),
                     "PWR1 %d.%06dV %d.%06dA %d.%06dW\r\n",
                     bus_voltage.val1, bus_voltage.val2,
                     current.val1, current.val2,
                     power.val1, power.val2);

            serial_send(buf);
        }

        /* ================= RS485 STATUS ================= */

        if (k_uptime_get() - last_rs485 > 1000) {

            char buf[32];

            snprintf(buf, sizeof(buf),
                     "TSET %.1f\n",
                     target_temp);

            rs485_send(buf);

            last_rs485 = k_uptime_get();
        }

        k_sleep(K_MSEC(200));
    }

    return 0;
}