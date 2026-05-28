#include "app.h"

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>

static struct status last_status;
static float target_pt = 25.2f;
static float target_lm = 25.0f;
static struct k_spinlock target_lock;

K_MUTEX_DEFINE(status_mutex);

struct status app_status_get(void)
{
    struct status status;

    k_mutex_lock(&status_mutex, K_FOREVER);
    status = last_status;
    k_mutex_unlock(&status_mutex);

    return status;
}

void app_status_save_control(const struct status *status)
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

void app_status_save_power(const struct power_sample power[2])
{
    k_mutex_lock(&status_mutex, K_FOREVER);
    last_status.power[0] = power[0];
    last_status.power[1] = power[1];
    k_mutex_unlock(&status_mutex);
}

void app_targets_get(float *pt, float *lm)
{
    k_spinlock_key_t key = k_spin_lock(&target_lock);

    *pt = target_pt;
    *lm = target_lm;

    k_spin_unlock(&target_lock, key);
}

void app_targets_set(float pt, float lm)
{
    k_spinlock_key_t key = k_spin_lock(&target_lock);

    target_pt = pt;
    target_lm = lm;

    k_spin_unlock(&target_lock, key);
}
