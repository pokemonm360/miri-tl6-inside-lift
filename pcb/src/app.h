#pragma once

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>

#include <stdbool.h>
#include <stdint.h>

/* Devicetree nodes */
#define UART_RS485_NODE  DT_NODELABEL(usart1)
#define UART_SERIAL_NODE DT_NODELABEL(usart2)
#define LED1_NODE        DT_NODELABEL(led1)
#define PWM_NODE         DT_NODELABEL(pwm2)
#define MODBUS_NODE      DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_modbus_serial)

/* Measurement constants */
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

/* Control and telemetry */
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

/* Modbus RTU */
#define MODBUS_UNIT_ID 1
#define MODBUS_BAUD    115200

#define CONTROL_STACK_SIZE   3072
#define TELEMETRY_STACK_SIZE 3072
#define CONTROL_PRIORITY     4
#define TELEMETRY_PRIORITY   7

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

extern const struct device *const app_uart_rs485;
extern const struct device *const app_uart_serial;
extern const struct device *const app_adc_pt1000;
extern const struct device *const app_adc_lm35;
extern const struct device *const app_adc_stm32;
extern const struct device *const app_pwm_dev;
extern const struct device *const app_ina236_0;
extern const struct device *const app_ina236_1;

bool app_board_init(void);
void app_led1_set(bool on);
void app_pwm_set(float pt_percent, float lm_percent);
void app_report_lse_status(void);
bool app_lse_ready(void);
void app_serial_send(const char *str);

struct status app_status_get(void);
void app_status_save_control(const struct status *status);
void app_status_save_power(const struct power_sample power[2]);
void app_targets_get(float *pt, float *lm);
void app_targets_set(float pt, float lm);

void app_read_temperatures(struct temperatures *t, struct adc_sequence *stm32_seq);
struct power_sample app_read_power_sample(const struct device *dev);

int app_modbus_server_init(void);
void app_control_start(void);
void app_telemetry_start(void);
