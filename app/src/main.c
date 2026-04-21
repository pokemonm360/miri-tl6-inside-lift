#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/spinlock.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ================= Hardware ================= */

#define UART_RS485_NODE DT_NODELABEL(usart1)
#define UART_SERIAL_NODE DT_NODELABEL(usart2)
#define LED1_NODE        DT_NODELABEL(led1)
#define LED2_NODE        DT_NODELABEL(led2)
#define PWM_NODE         DT_NODELABEL(pwm2)

static const struct device *uart_rs485 = DEVICE_DT_GET(UART_RS485_NODE);
static const struct device *uart_serial = DEVICE_DT_GET(UART_SERIAL_NODE);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
static const struct device *adc_pt1000 = DEVICE_DT_GET(DT_NODELABEL(ads1220_0));
static const struct device *adc_lm35 = DEVICE_DT_GET(DT_NODELABEL(ads1220_1));
static const struct device *adc_stm32 = DEVICE_DT_GET(DT_NODELABEL(adc1));
static const struct device *pwm_dev = DEVICE_DT_GET(PWM_NODE);
static const struct device *ina219_0 = DEVICE_DT_GET(DT_NODELABEL(ina219_0));
static const struct device *ina219_1 = DEVICE_DT_GET(DT_NODELABEL(ina219_1));

/* ================= Constants ================= */

#define ADS1220_FULL_SCALE 8388607.0f
#define ADS1220_VREF_V     2.048f
#define ADS1220_PT_GAIN    4.0f
#define ADS1220_LM35_GAIN  2.0f
#define RREF_OHM           5085.0f
#define PT1000_R0          1000.0f
#define PT_A               3.9083e-3f
#define PT_B              -5.775e-7f
#define LM35_MV_PER_C      10.0f

#define PWM_PERIOD_NS      1000000U
#define PWM_CHANNEL_PT1000 2U
#define PWM_CHANNEL_LM35   4U /* Real PCB uses channel 1. */

#define CONTROL_PERIOD_MS      200
#define TELEMETRY_PERIOD_MS    200
#define RS485_STATUS_PERIOD_MS 1000
#define LED1_BLINK_MS         1000
#define LED2_BLINK_MS         4000
#define MIN_CONTROL_DT_S      0.05f
#define MAX_CONTROL_DT_S      1.0f

#define CONTROL_STACK_SIZE    3072
#define TELEMETRY_STACK_SIZE  3072
#define CONTROL_PRIORITY      4
#define TELEMETRY_PRIORITY    7

/* ================= Types ================= */

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float out_min;
    float out_max;
} PID;

struct temperature_readings {
    int16_t stm32_raw;
    int32_t pt1000_raw;
    int32_t lm35_raw;
    float stm32_voltage;
    float pt1000_temp_c;
    float lm35_temp_c;
};

struct power_measurement {
    struct sensor_value voltage;
    struct sensor_value current;
    struct sensor_value power;
    bool valid;
};

struct led_blink_state {
    int64_t last_toggle_ms;
    bool state;
};

struct control_targets {
    float target_pt;
    float target_lm;
    bool enabled;
};

struct control_snapshot {
    uint32_t sequence;
    int64_t timestamp_ms;
    float dt_s;
    float target_pt;
    float target_lm;
    float pwm_pt1000;
    float pwm_lm35;
    struct temperature_readings readings;
};

struct rs485_tx_state {
    const uint8_t *buf;
    size_t len;
    size_t pos;
    bool active;
};

/* ================= Shared State ================= */

static PID pid_pt1000 = {
    .Kp = 300.0f, //800.0f
    .Ki = 50.0f, //50.0f
    .Kd = 0.0f,
    .out_min = 0.0f,
    .out_max = 100.0f,
};

static PID pid_lm35 = {
    .Kp = 60.0f, //300.0f,
    .Ki = 5.0f, //5.0f,
    .Kd = 0.0f,
    .out_min = 0.0f,
    .out_max = 100.0f,
};

static struct control_targets g_targets = {
    .target_pt = 37.2f,
    .target_lm = 37.0f,
    .enabled = true,
};

static struct control_snapshot g_snapshot;
static struct rs485_tx_state g_rs485_tx;

static struct k_spinlock control_lock;
static struct k_spinlock rs485_lock;

K_TIMER_DEFINE(control_timer, NULL, NULL);
K_TIMER_DEFINE(telemetry_timer, NULL, NULL);

K_THREAD_STACK_DEFINE(control_stack, CONTROL_STACK_SIZE);
K_THREAD_STACK_DEFINE(telemetry_stack, TELEMETRY_STACK_SIZE);

static struct k_thread control_thread_data;
static struct k_thread telemetry_thread_data;

/* ================= Helpers ================= */

int ads1220_trigger(const struct device *dev);
int ads1220_fetch(const struct device *dev, int32_t *value);

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }

    return value;
}

#define INTEGRAL_ENABLE_ERROR 5.0f   // °C

#define INTEGRAL_ENABLE_ERROR 5.0f   // °C

static float pid_compute(PID *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    /* Apsauga nuo blogo dt */
    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - pid->prev_error) / dt;
    }

    float p_term = pid->Kp * error;
    float i_term = pid->Ki * pid->integral;
    float d_term = pid->Kd * derivative;

    float output_unsat = p_term + i_term + d_term;
    float output = clampf(output_unsat, pid->out_min, pid->out_max);

    /* --- Integratoriaus įjungimo sąlyga --- */
    bool integral_enabled = (fabsf(error) < INTEGRAL_ENABLE_ERROR);

    /* --- Anti-windup logika --- */
    bool upper_sat = (output_unsat > pid->out_max);
    bool lower_sat = (output_unsat < pid->out_min);

    if (integral_enabled &&
        ((!upper_sat && !lower_sat) ||
         (upper_sat && error < 0.0f) ||
         (lower_sat && error > 0.0f))) {

        pid->integral += error * dt;

        /* Teisingas integralo ribojimas pagal Ki */
        if (pid->Ki > 0.0f) {
            float i_max = pid->out_max / pid->Ki;
            float i_min = pid->out_min / pid->Ki;

            /* Jei išėjimas 0–100%, leidžiam simetriškai */
            if (pid->out_min == 0.0f) {
                i_min = -i_max;
            }

            pid->integral = clampf(pid->integral, i_min, i_max);
        }
    }

    pid->prev_error = error;
    return output;
}

static float compute_dt_s(int64_t now_ms, int64_t *last_ms)
{
    float dt_s = (float)CONTROL_PERIOD_MS / 1000.0f;

    if (*last_ms != 0) {
        dt_s = (float)(now_ms - *last_ms) / 1000.0f;
        dt_s = clampf(dt_s, MIN_CONTROL_DT_S, MAX_CONTROL_DT_S);
    }

    *last_ms = now_ms;
    return dt_s;
}

static void serial_send(const char *str)
{
    while (*str != '\0') {
        uart_poll_out(uart_serial, *str++);
    }
}

static void rs485_send_irq(const uint8_t *data, size_t len)
{
    bool should_enable_tx = false;
    k_spinlock_key_t key = k_spin_lock(&rs485_lock);

    if (!g_rs485_tx.active) {
        g_rs485_tx.buf = data;
        g_rs485_tx.len = len;
        g_rs485_tx.pos = 0U;
        g_rs485_tx.active = true;
        should_enable_tx = true;
    }

    k_spin_unlock(&rs485_lock, key);

    if (should_enable_tx) {
        uart_irq_tx_enable(uart_rs485);
    }
}

static bool parse_setpoints(const char *buffer)
{
    float new_target_pt;
    float new_target_lm;

    if (sscanf(buffer, "%f,%f", &new_target_pt, &new_target_lm) != 2) {
        return false;
    }

    k_spinlock_key_t key = k_spin_lock(&control_lock);
    g_targets.target_pt = new_target_pt;
    g_targets.target_lm = new_target_lm;
    g_targets.enabled = true;
    k_spin_unlock(&control_lock, key);

    printk("NEW SETPOINTS: PT=%.2f LM=%.2f\n", new_target_pt, new_target_lm);
    return true;
}

static void handle_uart_rx(const struct device *dev)
{
    static char rx_buffer[32];
    static size_t rx_pos;
    uint8_t c;

    while (uart_irq_rx_ready(dev)) {
        if (uart_fifo_read(dev, &c, 1) <= 0) {
            break;
        }

        if (c == '\n' || c == '\r') {
            rx_buffer[rx_pos] = '\0';

            if (rx_pos > 0U && !parse_setpoints(rx_buffer)) {
                printk("UART parse error: %s\n", rx_buffer);
            }

            rx_pos = 0U;
            continue;
        }

        if (rx_pos < sizeof(rx_buffer) - 1U) {
            rx_buffer[rx_pos++] = (char)c;
        } else {
            rx_pos = 0U;
        }
    }
}

static void handle_uart_tx(const struct device *dev)
{
    bool disable_tx = false;

    while (uart_irq_tx_ready(dev)) {
        const uint8_t *buf;
        size_t len;
        size_t pos;
        int sent;

        k_spinlock_key_t key = k_spin_lock(&rs485_lock);

        if (!g_rs485_tx.active) {
            k_spin_unlock(&rs485_lock, key);
            break;
        }

        buf = g_rs485_tx.buf;
        len = g_rs485_tx.len;
        pos = g_rs485_tx.pos;
        k_spin_unlock(&rs485_lock, key);

        sent = uart_fifo_fill(dev, &buf[pos], len - pos);
        if (sent <= 0) {
            break;
        }

        key = k_spin_lock(&rs485_lock);
        g_rs485_tx.pos += (size_t)sent;
        if (g_rs485_tx.pos >= g_rs485_tx.len) {
            g_rs485_tx.active = false;
            disable_tx = true;
        }
        k_spin_unlock(&rs485_lock, key);

        if (disable_tx) {
            break;
        }
    }

    if (disable_tx) {
        uart_irq_tx_disable(dev);
    }
}

static void uart_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    if (!uart_irq_update(dev)) {
        return;
    }

    handle_uart_rx(dev);
    handle_uart_tx(dev);
}

static bool init_leds(void)
{
    if (!gpio_is_ready_dt(&led1) || !gpio_is_ready_dt(&led2)) {
        printk("LED GPIO not ready!\n");
        return false;
    }

    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
    return true;
}

static bool init_devices(void)
{
    if (!device_is_ready(uart_rs485) || !device_is_ready(uart_serial)) {
        printk("UART device not ready!\n");
        return false;
    }

    if (!device_is_ready(adc_pt1000) || !device_is_ready(adc_lm35) ||
        !device_is_ready(adc_stm32)) {
        printk("ADC device not ready!\n");
        return false;
    }

    if (!device_is_ready(pwm_dev)) {
        printk("PWM device not ready!\n");
        return false;
    }

    if (!device_is_ready(ina219_0) || !device_is_ready(ina219_1)) {
        printk("INA219 device not ready!\n");
        return false;
    }

    return init_leds();
}

static bool read_stm32_adc(struct temperature_readings *readings,
                           struct adc_sequence *seq_stm32)
{
    if (adc_read(adc_stm32, seq_stm32) != 0) {
        return false;
    }

    readings->stm32_voltage = (readings->stm32_raw * 3.3f) / 4095.0f;
    return true;
}

static void trigger_external_adcs(void)
{
    ads1220_trigger(adc_pt1000);
    ads1220_trigger(adc_lm35);
}

static void fetch_external_adc_samples(struct temperature_readings *readings)
{
    ads1220_fetch(adc_pt1000, &readings->pt1000_raw);
    ads1220_fetch(adc_lm35, &readings->lm35_raw);
}

static float pt1000_raw_to_temp_c(int32_t raw)
{
    float r_rtd = (fabsf((float)raw) * RREF_OHM / ADS1220_PT_GAIN) /
                  ADS1220_FULL_SCALE;
    float ratio = r_rtd / PT1000_R0;
    float discriminant = PT_A * PT_A - 4.0f * PT_B * (1.0f - ratio);

    if (discriminant < 0.0f) {
        discriminant = 0.0f;
    }

    return (-PT_A + sqrtf(discriminant)) / (2.0f * PT_B);
}

static float lm35_raw_to_temp_c(int32_t raw)
{
    float vin_lm = ((float)raw / ADS1220_FULL_SCALE) *
                   (ADS1220_VREF_V / ADS1220_LM35_GAIN);

    return (vin_lm * 1000.0f) / LM35_MV_PER_C;
}

static void read_temperatures(struct temperature_readings *readings,
                              struct adc_sequence *seq_stm32)
{
    if (!read_stm32_adc(readings, seq_stm32)) {
        readings->stm32_voltage = 0.0f;
    }

    trigger_external_adcs();
    fetch_external_adc_samples(readings);

    readings->pt1000_temp_c = pt1000_raw_to_temp_c(readings->pt1000_raw);
    readings->lm35_temp_c = lm35_raw_to_temp_c(readings->lm35_raw);
}

static void apply_pwm_outputs(float pwm_pt1000, float pwm_lm35)
{
    uint32_t pulse_pt1000 = (uint32_t)((PWM_PERIOD_NS * pwm_pt1000) / 100.0f);
    uint32_t pulse_lm35 = (uint32_t)((PWM_PERIOD_NS * pwm_lm35) / 100.0f);

    pwm_set(pwm_dev, PWM_CHANNEL_PT1000, PWM_PERIOD_NS, pulse_pt1000, 0);
    pwm_set(pwm_dev, PWM_CHANNEL_LM35, PWM_PERIOD_NS, pulse_lm35, 0);
}

static void capture_snapshot(const struct temperature_readings *readings,
                             int64_t now_ms,
                             float dt_s,
                             float pwm_pt1000,
                             float pwm_lm35,
                             float target_pt,
                             float target_lm)
{
    static uint32_t next_sequence = 0;
    k_spinlock_key_t key = k_spin_lock(&control_lock);

    g_snapshot.sequence = next_sequence++;
    g_snapshot.timestamp_ms = now_ms;
    g_snapshot.dt_s = dt_s;
    g_snapshot.target_pt = target_pt;
    g_snapshot.target_lm = target_lm;
    g_snapshot.pwm_pt1000 = pwm_pt1000;
    g_snapshot.pwm_lm35 = pwm_lm35;
    g_snapshot.readings = *readings;

    k_spin_unlock(&control_lock, key);
}

static struct control_targets get_control_targets(void)
{
    struct control_targets targets;
    k_spinlock_key_t key = k_spin_lock(&control_lock);

    targets = g_targets;

    k_spin_unlock(&control_lock, key);
    return targets;
}

static struct control_snapshot get_control_snapshot(void)
{
    struct control_snapshot snapshot;
    k_spinlock_key_t key = k_spin_lock(&control_lock);

    snapshot = g_snapshot;

    k_spin_unlock(&control_lock, key);
    return snapshot;
}

static bool read_power_measurement(const struct device *dev,
                                   struct power_measurement *measurement)
{
    if (sensor_sample_fetch(dev) != 0) {
        return false;
    }

    sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, &measurement->voltage);
    sensor_channel_get(dev, SENSOR_CHAN_CURRENT, &measurement->current);
    sensor_channel_get(dev, SENSOR_CHAN_POWER, &measurement->power);
    return true;
}

static bool rs485_tx_is_active(void)
{
    bool active;
    k_spinlock_key_t key = k_spin_lock(&rs485_lock);

    active = g_rs485_tx.active;

    k_spin_unlock(&rs485_lock, key);
    return active;
}

static void publish_rs485_status(const struct control_snapshot *snapshot)
{
    static char rs485_buf[64];

    if (rs485_tx_is_active()) {
        return;
    }

    snprintf(rs485_buf, sizeof(rs485_buf),
             "SET PT=%.2f LM=%.2f\n",
             snapshot->target_pt,
             snapshot->target_lm);

    rs485_send_irq((const uint8_t *)rs485_buf, strlen(rs485_buf));
}

static void update_led(const struct gpio_dt_spec *led,
                       struct led_blink_state *blink,
                       int64_t now_ms,
                       int64_t interval_ms)
{
    if ((now_ms - blink->last_toggle_ms) < interval_ms) {
        return;
    }

    blink->state = !blink->state;
    gpio_pin_set_dt(led, blink->state);
    blink->last_toggle_ms = now_ms;
}

static void update_leds(int64_t now_ms,
                        struct led_blink_state *led1_blink,
                        struct led_blink_state *led2_blink)
{
    update_led(&led1, led1_blink, now_ms, LED1_BLINK_MS);
    update_led(&led2, led2_blink, now_ms, LED2_BLINK_MS);
}

static void log_snapshot(const struct control_snapshot *snapshot,
                         const struct power_measurement *power0,
                         const struct power_measurement *power1)
{
    char buf[320];
    double power0_voltage = NAN;
    double power0_current = NAN;
    double power0_power = NAN;
    double power1_voltage = NAN;
    double power1_current = NAN;
    double power1_power = NAN;

    if (power0->valid) {
        power0_voltage = sensor_value_to_double(&power0->voltage);
        power0_current = sensor_value_to_double(&power0->current);
        power0_power = sensor_value_to_double(&power0->power);
    }

    if (power1->valid) {
        power1_voltage = sensor_value_to_double(&power1->voltage);
        power1_current = sensor_value_to_double(&power1->current);
        power1_power = sensor_value_to_double(&power1->power);
    }

    snprintf(buf, sizeof(buf),
             "TEL,%lu,%lld,%.3f,%.3f,%.3f,%.2f,%.2f,%.1f,%.1f,%d,%d,%d,%d,%.6f,%.6f,%.6f,%d,%.6f,%.6f,%.6f\r\n",
             (unsigned long)snapshot->sequence,
             snapshot->timestamp_ms,
             snapshot->dt_s,
             snapshot->readings.pt1000_temp_c,
             snapshot->readings.lm35_temp_c,
             snapshot->target_pt,
             snapshot->target_lm,
             snapshot->pwm_pt1000,
             snapshot->pwm_lm35,
             snapshot->readings.pt1000_raw,
             snapshot->readings.lm35_raw,
             snapshot->readings.stm32_raw,
             power0->valid ? 1 : 0,
             power0_voltage,
             power0_current,
             power0_power,
             power1->valid ? 1 : 0,
             power1_voltage,
             power1_current,
             power1_power);

    serial_send(buf);
}

static void control_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    struct temperature_readings readings = {0};
    struct adc_sequence seq_stm32 = {
        .channels = BIT(16),
        .buffer = &readings.stm32_raw,
        .buffer_size = sizeof(readings.stm32_raw),
        .resolution = 12,
    };
    int64_t last_control_ms = 0;

    while (1) {
        struct control_targets targets;
        int64_t now_ms;
        float dt_s;
        float pwm_pt1000 = 0.0f;
        float pwm_lm35 = 0.0f;

        k_timer_status_sync(&control_timer);

        now_ms = k_uptime_get();
        dt_s = compute_dt_s(now_ms, &last_control_ms);
        targets = get_control_targets();

        read_temperatures(&readings, &seq_stm32);

        if (targets.enabled) {
            pwm_pt1000 = pid_compute(&pid_pt1000,
                                     targets.target_pt,
                                     readings.pt1000_temp_c,
                                     dt_s);

            pwm_lm35 = pid_compute(&pid_lm35,
                                   targets.target_lm,
                                   readings.lm35_temp_c,
                                   dt_s);
            //pwm_pt1000 = 60.0f;   // force 60%
            //pwm_lm35   = 0.0f;    // force 0%    
        }

        apply_pwm_outputs(pwm_pt1000, pwm_lm35);
        capture_snapshot(&readings,
                         now_ms,
                         dt_s,
                         pwm_pt1000,
                         pwm_lm35,
                         targets.target_pt,
                         targets.target_lm);
    }
}

static void telemetry_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    struct led_blink_state led1_blink = {0};
    struct led_blink_state led2_blink = {0};
    int64_t last_rs485_status_ms = 0;

    while (1) {
        struct control_snapshot snapshot;
        struct power_measurement power0 = {0};
        struct power_measurement power1 = {0};
        int64_t now_ms;

        k_timer_status_sync(&telemetry_timer);

        now_ms = k_uptime_get();
        snapshot = get_control_snapshot();

        power0.valid = read_power_measurement(ina219_0, &power0);
        power1.valid = read_power_measurement(ina219_1, &power1);

        log_snapshot(&snapshot, &power0, &power1);
        update_leds(now_ms, &led1_blink, &led2_blink);

        if ((now_ms - last_rs485_status_ms) >= RS485_STATUS_PERIOD_MS) {
            publish_rs485_status(&snapshot);
            last_rs485_status_ms = now_ms;
        }
    }
}

/* ================= Main ================= */

int main(void)
{
    printk("SYSTEM START\n");

    if (!init_devices()) {
        return 0;
    }

    uart_irq_callback_user_data_set(uart_rs485, uart_cb, NULL);
    uart_irq_rx_enable(uart_rs485);

    k_timer_start(&control_timer, K_NO_WAIT, K_MSEC(CONTROL_PERIOD_MS));
    k_timer_start(&telemetry_timer, K_NO_WAIT, K_MSEC(TELEMETRY_PERIOD_MS));

    k_thread_create(&control_thread_data,
                    control_stack,
                    K_THREAD_STACK_SIZEOF(control_stack),
                    control_thread,
                    NULL, NULL, NULL,
                    CONTROL_PRIORITY,
                    0,
                    K_NO_WAIT);

    k_thread_create(&telemetry_thread_data,
                    telemetry_stack,
                    K_THREAD_STACK_SIZEOF(telemetry_stack),
                    telemetry_thread,
                    NULL, NULL, NULL,
                    TELEMETRY_PRIORITY,
                    0,
                    K_NO_WAIT);

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
