#include "app.h"

#include <zephyr/kernel.h>

#include <math.h>

static struct pi_controller pi_pt = {.kp = 300.0f, .ki = 50.0f};
static struct pi_controller pi_lm = {.kp = 60.0f, .ki = 5.0f};

K_TIMER_DEFINE(control_timer, NULL, NULL);
K_THREAD_STACK_DEFINE(control_stack, CONTROL_STACK_SIZE);

static struct k_thread control_thread_data;

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

static float control_dt(int64_t now_ms, int64_t *last_ms)
{
    float dt_s = (float)CONTROL_PERIOD_MS / 1000.0f;

    if (*last_ms != 0) {
        dt_s = (float)(now_ms - *last_ms) / 1000.0f;
        dt_s = clampf(dt_s, MIN_CONTROL_DT_S, MAX_CONTROL_DT_S);
    }

    *last_ms = now_ms;
    return dt_s;
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
        app_targets_get(&s.target_pt, &s.target_lm);

        app_read_temperatures(&s.temp, &stm32_seq);
        s.pwm_pt = pi_update(&pi_pt, s.target_pt, s.temp.pt_c, s.dt_s);
        s.pwm_lm = pi_update(&pi_lm, s.target_lm, s.temp.lm_c, s.dt_s);

        app_pwm_set(s.pwm_pt, s.pwm_lm);
        app_status_save_control(&s);
    }
}

void app_control_start(void)
{
    k_timer_start(&control_timer, K_NO_WAIT, K_MSEC(CONTROL_PERIOD_MS));
    k_thread_create(&control_thread_data, control_stack,
                    K_THREAD_STACK_SIZEOF(control_stack), control_thread,
                    NULL, NULL, NULL, CONTROL_PRIORITY, 0, K_NO_WAIT);
}
