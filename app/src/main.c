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

/* ================= RS485 ================= */
static const uint8_t *tx_buf;
static size_t tx_len;
static size_t tx_pos;
static bool tx_active = false;





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

/* ================= PID ================= */
typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float prev_error;

    float out_min;
    float out_max;
} PID;

float target_pt = 37.2;
float target_lm = 37.0;

float pid_compute(PID *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    pid->integral += error * dt;

    // anti-windup
    if (pid->integral > pid->out_max) pid->integral = pid->out_max;
    if (pid->integral < pid->out_min) pid->integral = pid->out_min;

    float derivative = (error - pid->prev_error) / dt;

    float output = pid->Kp * error +
                   pid->Ki * pid->integral +
                   pid->Kd * derivative;

    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    pid->prev_error = error;

    return output;
}

PID pid_pt1000 = {
    .Kp = 4.0,
    .Ki = 0.3,
    .Kd = 0.0,
    .out_min = 0,
    .out_max = 100
};

PID pid_lm35 = {
    .Kp = 4.0,
    .Ki = 0.3,
    .Kd = 0.0,
    .out_min = 0,
    .out_max = 100
};

/* ================= RS485 RX ================= */

static char rx_buffer[32];
static int rx_pos = 0;

static int current_value = 0;

/* ================= CONTROL ================= */

static float target_temp = 37.0;
static bool control_enabled = false;

/* ================= RS485 SEND ================= */
void rs485_send_irq(const uint8_t *data, size_t len)
{
    if (tx_active) return; // optional: arba queue

    tx_buf = data;
    tx_len = len;
    tx_pos = 0;
    tx_active = true;

    uart_irq_tx_enable(uart_rs485);
}

/* ================= UART RX CALLBACK ================= */

static void uart_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(dev))
        return;

    /* ================= RX ================= */
    while (uart_irq_rx_ready(dev)) {

        if (uart_fifo_read(dev, &c, 1) <= 0)
            break;

        // End of line → parse
        if (c == '\n' || c == '\r') {

            rx_buffer[rx_pos] = '\0';

            if (rx_pos > 0) {

                float t_pt, t_lm;

                if (sscanf(rx_buffer, "%f,%f", &t_pt, &t_lm) == 2) {

                    target_pt = t_pt;
                    target_lm = t_lm;
                    control_enabled = true;

                    printk("NEW SETPOINTS: PT=%.2f LM=%.2f\n",
                           target_pt, target_lm);

                } else {
                    printk("UART parse error: %s\n", rx_buffer);
                }
            }

            rx_pos = 0;
            continue;
        }

        // Store character
        if (rx_pos < sizeof(rx_buffer) - 1) {
            rx_buffer[rx_pos++] = c;
        } else {
            rx_pos = 0; // overflow reset
        }
    }

    /* ================= TX ================= */
    if (uart_irq_tx_ready(dev) && tx_active) {

        while (tx_pos < tx_len) {
            int sent = uart_fifo_fill(dev, &tx_buf[tx_pos], tx_len - tx_pos);
            if (sent <= 0) {
                break;
            }
            tx_pos += sent;
        }

        if (tx_pos >= tx_len) {
            uart_irq_tx_disable(dev);
            tx_active = false;
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

        uint32_t period = 1000000;   // 1 ms = 1,000,000 ns    1000 Hz
        //uint32_t period = 10000000;  // 10 ms = 10,000,000 ns  100 Hz
        //uint32_t period = 100000000; // 100 ms = 100,000,000 ns 10 Hz
        //uint32_t period = 1000000000; // 1 s = 1,000,000,000 ns
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

        float r_rtd = (fabsf((float)buf_pt) * RREF_OHM /
                      (ADS1220_PT_GAIN)) /
                      ADS1220_FULL_SCALE;

        float ratio = r_rtd / PT1000_R0;

        float temp_pt = (-PT_A + sqrtf(PT_A * PT_A -
                         4 * PT_B * (1 - ratio))) /
                         (2 * PT_B);

        printf("PT1000 Temp: %.3f C\n", temp_pt);
        printf("Raw PT1000: %d\n", buf_pt);

         /* ================= LM35 ================= */

        float vin_lm = ((float)buf_lm / ADS1220_FULL_SCALE) *
                       (ADS1220_VREF_V / ADS1220_LM35_GAIN);

        float temp_lm = (vin_lm * 1000.0f) / LM35_MV_PER_C;

        printf("LM35 Temp: %.3f C\n\n", temp_lm);

        /* ================= PWM CONTROL ================= */

        float pwm_pt1000 = 0;
        float pwm_lm35 = 0;

        float dt = 0.2; // 200 ms loop

        if (1) {

        static float temp_pt_f = 0.0f;
        static bool temp_pt_f_init = false;

        temp_pt_f =  temp_pt;
        

        pwm_pt1000 = pid_compute(&pid_pt1000,
                           target_pt,
                           temp_pt_f,
                           dt);

        pwm_lm35 = pid_compute(&pid_lm35,
                           target_lm,
                           temp_lm,
                           dt);
        }


    //uint32_t pulse0 = (period * pwm_pt1000) / 100;
    //uint32_t pulse1 = (period * pwm_lm35) / 100;

    uint32_t pulse_pt1000 = (period * 0) / 100;
    uint32_t pulse_lm35 = (period * 40) / 100;

    pwm_set(pwm_dev, 2, period, pulse_pt1000, 0);
    pwm_set(pwm_dev, 4, period, pulse_lm35, 0);

    printf("PWM_PT1000: %.1f %%\n", pwm_pt1000);
    printf("PWM_LM35: %.1f %%\n\n", pwm_lm35);
       
        /* ================= INA219 ================= 

        struct sensor_value bus_voltage;
        struct sensor_value current;
        struct sensor_value power;

        if (sensor_sample_fetch(ina219_0) != 0) {
            printk("INA219_0 FAIL\n");
        }

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
        */
        /* ================= RS485 STATUS ================= */
        static char rs485_buf[64];

        if (k_uptime_get() - last_rs485 > 1000) {

            if (!tx_active) {   // labai svarbu

                printk("QUEUE RS485\n");

                snprintf(rs485_buf, sizeof(rs485_buf),
                        "SET PT=%.2f LM=%.2f\n",
                        target_pt,
                        target_lm);

                rs485_send_irq((uint8_t *)rs485_buf,
                            strlen(rs485_buf));

                last_rs485 = k_uptime_get();
            }
        }

        k_sleep(K_MSEC(200));
    }

    return 0;
}