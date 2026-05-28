#include "app.h"

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>

#include <math.h>

int ads1220_trigger(const struct device *dev);
int ads1220_fetch(const struct device *dev, int32_t *value);

static float pt1000_compensate(float t)
{
    float err = (-0.00003f * t * t) + (-0.0041f * t) + 0.3738f;

    return t - err;
}

static float lm35_compensate(float t)
{
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

void app_read_temperatures(struct temperatures *t, struct adc_sequence *stm32_seq)
{
    t->stm32_v = 0.0f;
    if (adc_read(app_adc_stm32, stm32_seq) == 0) {
        t->stm32_v = ((float)t->stm32_raw * 3.3f) / 4095.0f;
    }

    ads1220_trigger(app_adc_pt1000);
    ads1220_trigger(app_adc_lm35);
    ads1220_fetch(app_adc_pt1000, &t->pt_raw);
    ads1220_fetch(app_adc_lm35, &t->lm_raw);

    t->pt_c = pt1000_compensate(pt1000_temp_c(t->pt_raw));
    t->lm_c = lm35_compensate(lm35_temp_c(t->lm_raw));
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

struct power_sample app_read_power_sample(const struct device *dev)
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
