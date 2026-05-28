#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/modbus/modbus.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/printk.h>

#include <stm32_ll_rcc.h>

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* Hardware */
#define UART_RS485_NODE  DT_NODELABEL(usart1)
#define UART_SERIAL_NODE DT_NODELABEL(usart2)
#define LED1_NODE        DT_NODELABEL(led1)
#define PWM_NODE         DT_NODELABEL(pwm2)
#define MODBUS_NODE      DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_modbus_serial)

static const struct device *uart_rs485 = DEVICE_DT_GET(UART_RS485_NODE);
static const struct device *uart_serial = DEVICE_DT_GET(UART_SERIAL_NODE);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
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
#define LED1_BLINK_MS         1000
#define MIN_CONTROL_DT_S      0.05f
#define MAX_CONTROL_DT_S      1.0f
#define INTEGRAL_ENABLE_ERROR 5.0f
#define LID_CLOSED_THRESHOLD_MV 2200

#define MODBUS_UNIT_ID 1
#define MODBUS_BAUD    115200

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

struct power_sample {
    bool ok;
    int32_t voltage_mv;
    int32_t current_ua;
    int32_t power_uw;
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
    struct power_sample power[2];
};

static struct pi_controller pi_pt = {.kp = 300.0f, .ki = 50.0f};
static struct pi_controller pi_lm = {.kp = 60.0f, .ki = 5.0f};
static float target_pt = 25.2f;
static float target_lm = 25.0f;
static struct status last_status;
static struct k_spinlock target_lock;

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
    struct power_sample power[2];

    k_mutex_lock(&status_mutex, K_FOREVER);
    power[0] = last_status.power[0];
    power[1] = last_status.power[1];
    last_status = *status;
    last_status.power[0] = power[0];
    last_status.power[1] = power[1];
    k_mutex_unlock(&status_mutex);
}

static void save_power_status(const struct power_sample power[2])
{
    k_mutex_lock(&status_mutex, K_FOREVER);
    last_status.power[0] = power[0];
    last_status.power[1] = power[1];
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

static void report_lse_status(void)
{
#if defined(RCC_BDCR_LSEON) && defined(RCC_BDCR_LSERDY)
    char buf[96];
    uint32_t bdcr = RCC->BDCR;

    snprintf(buf, sizeof(buf), "BDCR=0x%08lx LSEON=%d LSERDY=%d\r\n",
             (unsigned long)bdcr,
             (bdcr & RCC_BDCR_LSEON) != 0U,
             (bdcr & RCC_BDCR_LSERDY) != 0U);
    serial_send(buf);
    printk("%s", buf);
#endif
}

static bool lse_ready(void)
{
#if defined(RCC_BDCR_LSERDY)
    return (RCC->BDCR & RCC_BDCR_LSERDY) != 0U;
#else
    return false;
#endif
}

static uint16_t u32_reg(uint32_t value, uint16_t word)
{
    return (uint16_t)(value >> (word * 16U));
}

static uint16_t i32_reg(int32_t value, uint16_t word)
{
    return u32_reg((uint32_t)value, word);
}

static uint16_t f_to_s16_reg(float value, float scale)
{
    return (uint16_t)(int16_t)(value * scale);
}

static bool lid_closed_from_mv(int32_t mv)
{
    return mv >= LID_CLOSED_THRESHOLD_MV;
}

static int discrete_input_rd(uint16_t addr, bool *state)
{
    struct status s = get_status();
    int32_t lid_mv = (int32_t)(s.temp.stm32_v * 1000.0f);

    switch (addr) {
    case 0:
        *state = lid_closed_from_mv(lid_mv);
        return 0;
    case 1:
        *state = s.power[0].ok;
        return 0;
    case 2:
        *state = s.power[1].ok;
        return 0;
    case 3:
        *state = lse_ready();
        return 0;
    default:
        return -ENOTSUP;
    }
}

static int input_reg_rd(uint16_t addr, uint16_t *reg)
{
    struct status s = get_status();
    int32_t stm32_mv = (int32_t)(s.temp.stm32_v * 1000.0f);

    switch (addr) {
    case 0:
        *reg = 1U;
        return 0;
    case 1:
        *reg = u32_reg(s.seq, 0);
        return 0;
    case 2:
        *reg = u32_reg(s.seq, 1);
        return 0;
    case 3:
        *reg = u32_reg((uint32_t)s.time_ms, 0);
        return 0;
    case 4:
        *reg = u32_reg((uint32_t)s.time_ms, 1);
        return 0;
    case 10:
        *reg = f_to_s16_reg(s.temp.pt_c, 100.0f);
        return 0;
    case 11:
        *reg = f_to_s16_reg(s.temp.lm_c, 100.0f);
        return 0;
    case 12:
        *reg = (uint16_t)(int16_t)stm32_mv;
        return 0;
    case 13:
        *reg = i32_reg(s.temp.pt_raw, 0);
        return 0;
    case 14:
        *reg = i32_reg(s.temp.pt_raw, 1);
        return 0;
    case 15:
        *reg = i32_reg(s.temp.lm_raw, 0);
        return 0;
    case 16:
        *reg = i32_reg(s.temp.lm_raw, 1);
        return 0;
    case 20:
        *reg = f_to_s16_reg(s.pwm_pt, 10.0f);
        return 0;
    case 21:
        *reg = f_to_s16_reg(s.pwm_lm, 10.0f);
        return 0;
    case 22:
        *reg = f_to_s16_reg(s.target_pt, 100.0f);
        return 0;
    case 23:
        *reg = f_to_s16_reg(s.target_lm, 100.0f);
        return 0;
    case 30:
    case 31:
        *reg = i32_reg(s.power[0].voltage_mv, addr - 30);
        return 0;
    case 32:
    case 33:
        *reg = i32_reg(s.power[0].current_ua, addr - 32);
        return 0;
    case 34:
    case 35:
        *reg = i32_reg(s.power[0].power_uw, addr - 34);
        return 0;
    case 40:
    case 41:
        *reg = i32_reg(s.power[1].voltage_mv, addr - 40);
        return 0;
    case 42:
    case 43:
        *reg = i32_reg(s.power[1].current_ua, addr - 42);
        return 0;
    case 44:
    case 45:
        *reg = i32_reg(s.power[1].power_uw, addr - 44);
        return 0;
    default:
        return -ENOTSUP;
    }
}

static int holding_reg_rd(uint16_t addr, uint16_t *reg)
{
    float pt;
    float lm;

    get_targets(&pt, &lm);

    switch (addr) {
    case 0:
        *reg = f_to_s16_reg(pt, 100.0f);
        return 0;
    case 1:
        *reg = f_to_s16_reg(lm, 100.0f);
        return 0;
    default:
        return -ENOTSUP;
    }
}

static int holding_reg_wr(uint16_t addr, uint16_t reg)
{
    float pt;
    float lm;
    float value = (float)(int16_t)reg / 100.0f;

    get_targets(&pt, &lm);

    switch (addr) {
    case 0:
        set_targets(value, lm);
        return 0;
    case 1:
        set_targets(pt, value);
        return 0;
    default:
        return -ENOTSUP;
    }
}

static struct modbus_user_callbacks mbs_cbs = {
    .discrete_input_rd = discrete_input_rd,
    .input_reg_rd = input_reg_rd,
    .holding_reg_rd = holding_reg_rd,
    .holding_reg_wr = holding_reg_wr,
};

static const struct modbus_iface_param server_param = {
    .mode = MODBUS_MODE_RTU,
    .server = {
        .user_cb = &mbs_cbs,
        .unit_id = MODBUS_UNIT_ID,
    },
    .serial = {
        .baud = MODBUS_BAUD,
        .parity = UART_CFG_PARITY_NONE,
    },
};

static int init_modbus_server(void)
{
    const char iface_name[] = DEVICE_DT_NAME(MODBUS_NODE);
    int iface = modbus_iface_get_by_name(iface_name);
    int err;

    if (iface < 0) {
        printk("Failed to get Modbus iface %s: %d\n", iface_name, iface);
        return iface;
    }

    err = modbus_init_server(iface, server_param);
    if (err != 0) {
        return err;
    }

    /* Modbus configures the UART with flow control disabled. Restore STM32
     * hardware driver-enable mode so PA12 drives the RS-485 transceiver DE.
     */
    const struct uart_config rs485_cfg = {
        .baudrate = MODBUS_BAUD,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_2,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_RS485,
    };

    return uart_configure(uart_rs485, &rs485_cfg);
}

static bool init_devices(void)
{
    if (!device_is_ready(uart_rs485) || !device_is_ready(uart_serial) ||
        !device_is_ready(adc_pt1000) || !device_is_ready(adc_lm35) ||
        !device_is_ready(adc_stm32) || !device_is_ready(pwm_dev)) {
        printk("Required device not ready\n");
        return false;
    }

    if (!gpio_is_ready_dt(&led1)) {
        printk("LED GPIO not ready\n");
        return false;
    }

    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);

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

static struct power_sample read_power_sample(const struct device *dev)
{
    struct sensor_value voltage;
    struct sensor_value current;
    struct sensor_value power;
    struct power_sample sample = {0};

    sample.ok = read_power(dev, &voltage, &current, &power);
    if (sample.ok) {
        sample.voltage_mv = (int32_t)(sensor_value_to_double(&voltage) * 1000.0);
        sample.current_ua = (int32_t)(sensor_value_to_double(&current) * 1000000.0);
        sample.power_uw = (int32_t)(sensor_value_to_double(&power) * 1000000.0);
    }

    return sample;
}

static void log_telemetry(const struct status *s)
{
    char buf[320];

    snprintf(buf, sizeof(buf),
             "TEL,%lu,%lld,%.3f,%.3f,%.3f,%.2f,%.2f,%.1f,%.1f,%d,%d,%d,%d,%.3f,%.6f,%.6f,%d,%.3f,%.6f,%.6f\r\n",
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
             s->power[0].ok ? 1 : 0,
             s->power[0].ok ? (double)s->power[0].voltage_mv / 1000.0 : NAN,
             s->power[0].ok ? (double)s->power[0].current_ua / 1000000.0 : NAN,
             s->power[0].ok ? (double)s->power[0].power_uw / 1000000.0 : NAN,
             s->power[1].ok ? 1 : 0,
             s->power[1].ok ? (double)s->power[1].voltage_mv / 1000.0 : NAN,
             s->power[1].ok ? (double)s->power[1].current_ua / 1000000.0 : NAN,
             s->power[1].ok ? (double)s->power[1].power_uw / 1000000.0 : NAN);

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
    int64_t last_led1_ms = 0;

    while (true) {
        int64_t now_ms;
        struct status s;
        struct power_sample power[2];

        k_timer_status_sync(&telemetry_timer);

        now_ms = k_uptime_get();
        power[0] = read_power_sample(ina236_0);
        power[1] = read_power_sample(ina236_1);
        save_power_status(power);

        s = get_status();
        log_telemetry(&s);

        if ((now_ms - last_led1_ms) >= LED1_BLINK_MS) {
            led1_on = !led1_on;
            gpio_pin_set_dt(&led1, led1_on);
            last_led1_ms = now_ms;
        }
    }
}

int main(void)
{
    printk("SYSTEM START\n");

    if (!init_devices()) {
        return 0;
    }

    k_sleep(K_SECONDS(2));
    report_lse_status();

    int modbus_err = init_modbus_server();

    if (modbus_err != 0) {
        char buf[64];

        snprintf(buf, sizeof(buf), "Modbus RTU init failed: %d\r\n", modbus_err);
        serial_send(buf);
        printk("%s", buf);
        return 0;
    }
    serial_send("Modbus RTU server ready\r\n");

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
