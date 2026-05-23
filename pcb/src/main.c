#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/printk.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Hardware */
#define UART_RS485_NODE  DT_NODELABEL(usart1)
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
static const struct device *ina236_0 = DEVICE_DT_GET(DT_NODELABEL(ina236_0));
static const struct device *ina236_1 = DEVICE_DT_GET(DT_NODELABEL(ina236_1));

/* Constants */
#define ADS1220_FULL_SCALE 8388607.0f
#define ADS1220_VREF_V     2.048f
#define ADS1220_PT_GAIN    4.0f
#define ADS1220_LM35_GAIN  4.0f
#define RREF_OHM           4990.0f
#define PT1000_R0          1000.0f
#define PT_A               3.9083e-3f
#define PT_B              -5.775e-7f
#define LM35_MV_PER_C      10.0f

#define PT1000_OFFSET_C 0.0f
#define LM35_OFFSET_C   0.0f

#define PWM_PERIOD_NS      1000000U
#define PWM_CHANNEL_PT1000 1U
#define PWM_CHANNEL_LM35   2U
#define PWM_MAX_PERCENT    100.0f

#define CONTROL_PERIOD_MS      200
#define TELEMETRY_PERIOD_MS    200
#define RS485_STATUS_PERIOD_MS 1000
#define LED1_BLINK_MS         1000
#define LED2_BLINK_MS         4000
#define MIN_CONTROL_DT_S      0.05f
#define MAX_CONTROL_DT_S      1.0f
#define INTEGRAL_ENABLE_ERROR 5.0f

#define CONTROL_STACK_SIZE   3072
#define TELEMETRY_STACK_SIZE 3072
#define CONTROL_PRIORITY     4
#define TELEMETRY_PRIORITY   7

int ads1220_trigger(const struct device *dev);
int ads1220_fetch(const struct device *dev, int32_t *value);

struct pi_controller {
    float kp;
    float ki;
    float integral;
};

struct temperatures {
    int16_t stm32_raw;
    int32_t pt_raw;
    int32_t lm_raw;
    float stm32_v;
    float pt_c;
    float lm_c;
};

struct status {
    uint32_t seq;
    int64_t time_ms;
    float dt_s;
    float target_pt;
    float target_lm;
    float pwm_pt;
    float pwm_lm;
    struct temperatures temp;
};

struct rs485_tx {
    const uint8_t *buf;
    size_t len;
    size_t pos;
    bool busy;
};

static struct pi_controller pi_pt = {.kp = 300.0f, .ki = 50.0f};
static struct pi_controller pi_lm = {.kp = 60.0f, .ki = 5.0f};
static float target_pt = 25.2f;
static float target_lm = 25.0f;
static struct status last_status;
static struct rs485_tx rs485_tx;
static struct k_spinlock target_lock;
static struct k_spinlock rs485_lock;

K_TIMER_DEFINE(control_timer, NULL, NULL);
K_TIMER_DEFINE(telemetry_timer, NULL, NULL);
K_MUTEX_DEFINE(status_mutex);
K_THREAD_STACK_DEFINE(control_stack, CONTROL_STACK_SIZE);
K_THREAD_STACK_DEFINE(telemetry_stack, TELEMETRY_STACK_SIZE);

static struct k_thread control_thread_data;
static struct k_thread telemetry_thread_data;

static float clampf(float value, float min, float max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

static float pi_update(struct pi_controller *pi, float target, float measured,
                       float dt_s)
{
    float error = target - measured;
    float output;
    float new_integral = pi->integral;

    if (fabsf(error) < INTEGRAL_ENABLE_ERROR) {
        new_integral += error * dt_s;
        if (pi->ki > 0.0f) {
            float limit = 100.0f / pi->ki;
            new_integral = clampf(new_integral, -limit, limit);
        }
    }

    output = clampf((pi->kp * error) + (pi->ki * new_integral), 0.0f,
                    PWM_MAX_PERCENT);

    if ((output > 0.0f && output < PWM_MAX_PERCENT) ||
        (output == PWM_MAX_PERCENT && error < 0.0f) ||
        (output == 0.0f && error > 0.0f)) {
        pi->integral = new_integral;
    }

    return output;
}

static float control_dt(int64_t now_ms, int64_t *last_ms) //Apskaiciuojamas realus laikas integracijai, o ne fiksuotas 200ms
{
    float dt_s = (float)CONTROL_PERIOD_MS / 1000.0f;

    if (*last_ms != 0) {
        dt_s = (float)(now_ms - *last_ms) / 1000.0f;
        dt_s = clampf(dt_s, MIN_CONTROL_DT_S, MAX_CONTROL_DT_S);
    }

    *last_ms = now_ms;
    return dt_s;
}

static float pt1000_compensate(float t)
{
    //float err = (-0.0003f * t * t) + (0.0173f * t) -0.0225f; pradine lygtis kur daugmaz ok
    //float err = (-0.0008f * t * t) + (0.0539f * t) - 0.6158f;
    //float err = (-0.0007f * t * t) + (0.0505f * t) - 0.5457f;
    float err = (-0.00003f * t * t) + (-0.0041f * t) + 0.3738f;
    return t - err;
}

static float lm35_compensate(float t)
{
    //float err = (-0.0012f * t * t) + (0.0583f * t) - 1.1303f; pradine lygtis kur daugmaz ok
    //float err = (0.0017f * t * t) - (0.1433f * t) + 2.1326f;
    //float err = (0.001f * t * t) - (0.0884f * t) + 1.0116f;
    float err = (0.0002f * t * t) - (0.0327f * t) + 0.124f;
    return t - err;
}

static float pt1000_temp_c(int32_t raw)
{
    float r_rtd = (fabsf((float)raw) * RREF_OHM / ADS1220_PT_GAIN) /
                  ADS1220_FULL_SCALE;
    float ratio = r_rtd / PT1000_R0;
    float d = PT_A * PT_A - 4.0f * PT_B * (1.0f - ratio);

    if (d < 0.0f) {
        d = 0.0f;
    }

    return ((-PT_A + sqrtf(d)) / (2.0f * PT_B)) - PT1000_OFFSET_C;
}

static float lm35_temp_c(int32_t raw)
{
    float vin = ((float)raw / ADS1220_FULL_SCALE) *
                (ADS1220_VREF_V / ADS1220_LM35_GAIN);

    return ((vin * 1000.0f) / LM35_MV_PER_C) - LM35_OFFSET_C;
}

static void read_temperatures(struct temperatures *t,
                              struct adc_sequence *stm32_seq)
{
    t->stm32_v = 0.0f;
    if (adc_read(adc_stm32, stm32_seq) == 0) {
        t->stm32_v = ((float)t->stm32_raw * 3.3f) / 4095.0f;
    }

    ads1220_trigger(adc_pt1000);
    ads1220_trigger(adc_lm35);
    ads1220_fetch(adc_pt1000, &t->pt_raw);
    ads1220_fetch(adc_lm35, &t->lm_raw);

    t->pt_c = pt1000_compensate(pt1000_temp_c(t->pt_raw));
    t->lm_c = lm35_compensate(lm35_temp_c(t->lm_raw));
}

static void set_pwm(float pt_percent, float lm_percent)
{
    uint32_t pt_pulse = (uint32_t)((PWM_PERIOD_NS * pt_percent) / 100.0f);
    uint32_t lm_pulse = (uint32_t)((PWM_PERIOD_NS * lm_percent) / 100.0f);

    pwm_set(pwm_dev, PWM_CHANNEL_PT1000, PWM_PERIOD_NS, pt_pulse, 0);
    pwm_set(pwm_dev, PWM_CHANNEL_LM35, PWM_PERIOD_NS, lm_pulse, 0);
}

static void save_status(const struct status *status)
{
    k_mutex_lock(&status_mutex, K_FOREVER);
    last_status = *status;
    k_mutex_unlock(&status_mutex);
}

static struct status get_status(void)
{
    struct status status;

    k_mutex_lock(&status_mutex, K_FOREVER);
    status = last_status;
    k_mutex_unlock(&status_mutex);

    return status;
}

static void get_targets(float *pt, float *lm) //Naudojam spinlock, nes ISR negalim naudot mutex, o ir set_targets gali but iskviestas is ISR, tai reikia apsaugot nuo konkurencijos
{
    k_spinlock_key_t key = k_spin_lock(&target_lock);

    *pt = target_pt;
    *lm = target_lm;

    k_spin_unlock(&target_lock, key);
}

static void set_targets(float pt, float lm)
{
    k_spinlock_key_t key = k_spin_lock(&target_lock);

    target_pt = pt;
    target_lm = lm;

    k_spin_unlock(&target_lock, key);
}

static void serial_send(const char *str)
{
    while (*str != '\0') {
        uart_poll_out(uart_serial, *str++);
    }
}

static bool rs485_is_busy(void)
{
    bool busy;
    k_spinlock_key_t key = k_spin_lock(&rs485_lock);

    busy = rs485_tx.busy;

    k_spin_unlock(&rs485_lock, key);
    return busy;
}

static void rs485_send(const char *text)
{
    bool start_tx = false;
    k_spinlock_key_t key = k_spin_lock(&rs485_lock);

    if (!rs485_tx.busy) {
        rs485_tx.buf = (const uint8_t *)text;
        rs485_tx.len = strlen(text);
        rs485_tx.pos = 0U;
        rs485_tx.busy = true;
        start_tx = true;
    }

    k_spin_unlock(&rs485_lock, key);

    if (start_tx) {
        uart_irq_tx_enable(uart_rs485);
    }
}

static bool parse_setpoints(const char *text)
{
    float pt;
    float lm;

    if (sscanf(text, "%f,%f", &pt, &lm) != 2) {
        return false;
    }

    set_targets(pt, lm);
    printk("NEW SETPOINTS: PT=%.2f LM=%.2f\n", pt, lm);
    return true;
}

static void uart_rx(const struct device *dev)
{
    static char buf[32];
    static size_t pos;
    uint8_t c;

    while (uart_irq_rx_ready(dev) && uart_fifo_read(dev, &c, 1) > 0) {
        if (c == '\n' || c == '\r') {
            buf[pos] = '\0';
            if (pos > 0U && !parse_setpoints(buf)) {
                printk("UART parse error: %s\n", buf);
            }
            pos = 0U;
        } else if (pos < sizeof(buf) - 1U) {
            buf[pos++] = (char)c;
        } else {
            pos = 0U;
        }
    }
}

static void rs485_uart_tx_irq(const struct device *dev)
{
    k_spinlock_key_t key;

    while (uart_irq_tx_ready(dev)) {
        key = k_spin_lock(&rs485_lock);

        if (!rs485_tx.busy) {
            k_spin_unlock(&rs485_lock, key);
            uart_irq_tx_disable(dev);
            return;
        }

        int sent = uart_fifo_fill(dev, &rs485_tx.buf[rs485_tx.pos],
                                  rs485_tx.len - rs485_tx.pos);
        if (sent > 0) {
            rs485_tx.pos += (size_t)sent;
            rs485_tx.busy = rs485_tx.pos < rs485_tx.len;
        }

        k_spin_unlock(&rs485_lock, key);

        if (sent <= 0) {
            return;
        }
    }
}

static void uart_cb(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    if (uart_irq_update(dev)) {
        uart_rx(dev);
        rs485_uart_tx_irq(dev);
    }
}

static bool init_devices(void)
{
    if (!device_is_ready(uart_rs485) || !device_is_ready(uart_serial) ||
        !device_is_ready(adc_pt1000) || !device_is_ready(adc_lm35) ||
        !device_is_ready(adc_stm32) || !device_is_ready(pwm_dev)) {
        printk("Required device not ready\n");
        return false;
    }

    if (!gpio_is_ready_dt(&led1) || !gpio_is_ready_dt(&led2)) {
        printk("LED GPIO not ready\n");
        return false;
    }

    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);

    if (!device_is_ready(ina236_0)) {
        printk("INA236_0 not ready, telemetry will show invalid power 0\n");
    }
    if (!device_is_ready(ina236_1)) {
        printk("INA236_1 not ready, telemetry will show invalid power 1\n");
    }

    return true;
}

static bool read_power(const struct device *dev, struct sensor_value *voltage,
                       struct sensor_value *current, struct sensor_value *power)
{
    return device_is_ready(dev) &&
           sensor_sample_fetch(dev) == 0 &&
           sensor_channel_get(dev, SENSOR_CHAN_VOLTAGE, voltage) == 0 &&
           sensor_channel_get(dev, SENSOR_CHAN_CURRENT, current) == 0 &&
           sensor_channel_get(dev, SENSOR_CHAN_POWER, power) == 0;
}

static void log_telemetry(const struct status *s)
{
    char buf[320];
    struct sensor_value v0;
    struct sensor_value c0;
    struct sensor_value p0;
    struct sensor_value v1;
    struct sensor_value c1;
    struct sensor_value p1;
    bool power0_ok = read_power(ina236_0, &v0, &c0, &p0);
    bool power1_ok = read_power(ina236_1, &v1, &c1, &p1);

    snprintf(buf, sizeof(buf),
             "TEL,%lu,%lld,%.3f,%.3f,%.3f,%.2f,%.2f,%.1f,%.1f,%d,%d,%d,%d,%.6f,%.6f,%.6f,%d,%.6f,%.6f,%.6f\r\n",
             (unsigned long)s->seq,
             s->time_ms,
             s->dt_s,
             s->temp.pt_c,
             s->temp.lm_c,
             s->target_pt,
             s->target_lm,
             s->pwm_pt,
             s->pwm_lm,
             s->temp.pt_raw,
             s->temp.lm_raw,
             s->temp.stm32_raw,
             power0_ok ? 1 : 0,
             power0_ok ? sensor_value_to_double(&v0) : NAN,
             power0_ok ? sensor_value_to_double(&c0) : NAN,
             power0_ok ? sensor_value_to_double(&p0) : NAN,
             power1_ok ? 1 : 0,
             power1_ok ? sensor_value_to_double(&v1) : NAN,
             power1_ok ? sensor_value_to_double(&c1) : NAN,
             power1_ok ? sensor_value_to_double(&p1) : NAN);

    serial_send(buf);
}

static void control_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    static uint32_t seq;
    int64_t last_ms = 0;
    struct status s = {0};
    struct adc_sequence stm32_seq = {
        .channels = BIT(16),
        .buffer = &s.temp.stm32_raw,
        .buffer_size = sizeof(s.temp.stm32_raw),
        .resolution = 12,
    };

    while (true) {
        k_timer_status_sync(&control_timer);

        s.seq = seq++;
        s.time_ms = k_uptime_get();
        s.dt_s = control_dt(s.time_ms, &last_ms);
        get_targets(&s.target_pt, &s.target_lm);

        read_temperatures(&s.temp, &stm32_seq);
        s.pwm_pt = pi_update(&pi_pt, s.target_pt, s.temp.pt_c, s.dt_s);
        s.pwm_lm = pi_update(&pi_lm, s.target_lm, s.temp.lm_c, s.dt_s);

        set_pwm(s.pwm_pt, s.pwm_lm);
        save_status(&s);
    }
}

static void telemetry_thread(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    bool led1_on = false;
    bool led2_on = false;
    int64_t last_led1_ms = 0;
    int64_t last_led2_ms = 0;
    int64_t last_rs485_ms = 0;
    static char rs485_buf[64];

    while (true) {
        int64_t now_ms;
        struct status s;

        k_timer_status_sync(&telemetry_timer);

        now_ms = k_uptime_get();
        s = get_status();
        log_telemetry(&s);

        if ((now_ms - last_led1_ms) >= LED1_BLINK_MS) {
            led1_on = !led1_on;
            gpio_pin_set_dt(&led1, led1_on);
            last_led1_ms = now_ms;
        }

        if ((now_ms - last_led2_ms) >= LED2_BLINK_MS) {
            led2_on = !led2_on;
            gpio_pin_set_dt(&led2, led2_on);
            last_led2_ms = now_ms;
        }

        if (!rs485_is_busy() &&
            (now_ms - last_rs485_ms) >= RS485_STATUS_PERIOD_MS) {
            snprintf(rs485_buf, sizeof(rs485_buf), "SET PT=%.2f LM=%.2f\n",
                     s.target_pt, s.target_lm);
            rs485_send(rs485_buf);
            last_rs485_ms = now_ms;
        }
    }
}

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

    k_thread_create(&control_thread_data, control_stack,
                    K_THREAD_STACK_SIZEOF(control_stack), control_thread,
                    NULL, NULL, NULL, CONTROL_PRIORITY, 0, K_NO_WAIT);

    k_thread_create(&telemetry_thread_data, telemetry_stack,
                    K_THREAD_STACK_SIZEOF(telemetry_stack), telemetry_thread,
                    NULL, NULL, NULL, TELEMETRY_PRIORITY, 0, K_NO_WAIT);

    k_sleep(K_FOREVER);
    return 0;
}
