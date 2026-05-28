#include "app.h"

#include <zephyr/kernel.h>

#include <math.h>
#include <stdio.h>

K_TIMER_DEFINE(telemetry_timer, NULL, NULL);
K_THREAD_STACK_DEFINE(telemetry_stack, TELEMETRY_STACK_SIZE);

static struct k_thread telemetry_thread_data;

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

    app_serial_send(buf);
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
        power[0] = app_read_power_sample(app_ina236_0);
        power[1] = app_read_power_sample(app_ina236_1);
        app_status_save_power(power);

        s = app_status_get();
        log_telemetry(&s);

        if ((now_ms - last_led1_ms) >= LED1_BLINK_MS) {
            led1_on = !led1_on;
            app_led1_set(led1_on);
            last_led1_ms = now_ms;
        }
    }
}

void app_telemetry_start(void)
{
    k_timer_start(&telemetry_timer, K_NO_WAIT, K_MSEC(TELEMETRY_PERIOD_MS));
    k_thread_create(&telemetry_thread_data, telemetry_stack,
                    K_THREAD_STACK_SIZEOF(telemetry_stack), telemetry_thread,
                    NULL, NULL, NULL, TELEMETRY_PRIORITY, 0, K_NO_WAIT);
}
