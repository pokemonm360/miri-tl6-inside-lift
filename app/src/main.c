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


static const struct device *uart1 =
    DEVICE_DT_GET(DT_NODELABEL(usart1));

/* Abu ADS1220 */
static const struct device *adc_pt1000 =
    DEVICE_DT_GET(DT_NODELABEL(ads1220_0));

static const struct device *adc_lm35 =
    DEVICE_DT_GET(DT_NODELABEL(ads1220_1));

static const struct device *adc_stm32 =
    DEVICE_DT_GET(DT_NODELABEL(adc1));

#define PWM_NODE DT_NODELABEL(pwm2)

static const struct device *pwm_dev =
    DEVICE_DT_GET(PWM_NODE);

static const struct device *ina219_0 =
    DEVICE_DT_GET(DT_NODELABEL(ina219_0));
static const struct device *ina219_1 =
    DEVICE_DT_GET(DT_NODELABEL(ina219_1));

/* ==== ADS1220 konstantos ==== */
#define ADS1220_FULL_SCALE 8388607.0f   // 2^23 - 1
#define ADS1220_VREF_V     2.048f

/* PT1000 nustatymai */
#define ADS1220_PT_GAIN    4.0f
#define RREF_OHM           5085.0f

#define PT1000_R0 1000.0f
#define PT_A 3.9083e-3f
#define PT_B -5.775e-7f

/* LM35 nustatymai */
#define ADS1220_LM35_GAIN  2.0f   //
#define LM35_MV_PER_C      10.0f  // 10mV / °C



void uart1_send_string(const char *str)
{
    while (*str) {
        uart_poll_out(uart1, *str++);
    }
}


int main(void)
{   
    if (!device_is_ready(uart1)) {
        return -ENODEV;
    }
    uart1_send_string("test\r\n");

    printk("MAIN START\n");

    if (!device_is_ready(adc_pt1000) ||
        !device_is_ready(adc_lm35)) {
        printk("ADC device not ready\n");
        return -ENODEV;
    }

    if (!device_is_ready(adc_stm32)) {
    printk("STM32 ADC not ready\n");
    return -ENODEV;
    }

    int32_t buf_pt;
    int32_t buf_lm;
    int16_t stm32_buf;
    int ret;

    struct adc_sequence seq_pt = {
        .channels    = BIT(0),
        .buffer      = &buf_pt,
        .buffer_size = sizeof(buf_pt),
        .resolution  = 24,
    };

    struct adc_sequence seq_lm = {
        .channels    = BIT(0),
        .buffer      = &buf_lm,
        .buffer_size = sizeof(buf_lm),
        .resolution  = 24,
    };

    struct adc_sequence seq_stm32 = {
        .channels = BIT(16),
        .buffer = &stm32_buf,
        .buffer_size = sizeof(stm32_buf),
        .resolution = 12,
    };

    printk("ADS1220 ready\n");

    if (!device_is_ready(ina219_0) ||  !device_is_ready(ina219_1)) {
        printk("INA219 not ready\n");
        return -ENODEV;
    }


    while (1) {


        uint32_t period = 1000000; // 1 ms = 1 kHz

        uint32_t pulse0 = (period * 70) / 100;
        uint32_t pulse1 = (period * 20) / 100;

        pwm_set(pwm_dev, 2, period, pulse0, 0);
        pwm_set(pwm_dev, 4, period, pulse1, 0);
       
        printk("CH1=70%%  CH2=20%%\n");

        uart1_send_string("test\r\n");

        ret = adc_read(adc_stm32, &seq_stm32);
        if (ret == 0) {
            printk("STM32 ADC raw: %d\n", stm32_buf);
            float voltage = (stm32_buf * 3.3f) / 4095.0f;
            printf("Voltage: %.3f V\n", voltage);
        }

        /* =========================
           ===== PT1000 DALIS ======
           ========================= */
        ret = adc_read(adc_pt1000, &seq_pt);
        if (ret == 0) {

            float vin_pt = ((float)buf_pt / ADS1220_FULL_SCALE) *
                           (ADS1220_VREF_V / ADS1220_PT_GAIN);

            float r_rtd = ((float)buf_pt * RREF_OHM /
                          (-ADS1220_PT_GAIN)) /
                          ADS1220_FULL_SCALE;

            float ratio = r_rtd / PT1000_R0;

            float temp_pt = (-PT_A + sqrtf(PT_A * PT_A -
                             4 * PT_B * (1 - ratio))) /
                             (2 * PT_B);

            printf("PT1000 raw: %d\n", buf_pt);
            printf("PT1000 Temp: %.3f C\n\n", temp_pt);
        }

        /* =========================
           ===== LM35 DALIS ========
           ========================= */
        ret = adc_read(adc_lm35, &seq_lm);
        if (ret == 0) {

            printf("LM35 raw: %d\n", buf_lm);

            /* Įtampa pagal 2.048V reference */
            float vin_lm = ((float)buf_lm / ADS1220_FULL_SCALE) *
                           (ADS1220_VREF_V / ADS1220_LM35_GAIN);

            printf("LM35 Voltage: %.6f V\n", vin_lm);

            /* Temperatūra (10mV/°C) */
            float temp_lm = (vin_lm * 1000.0f) / LM35_MV_PER_C;

            printf("LM35 Temp: %.3f C\n\n", temp_lm);
        }

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

            printf("INA219_0 Bus: %d.%06d V\n",
                bus_voltage.val1,
                bus_voltage.val2);

            printf("INA219_0 Current: %d.%06d A\n",
                current.val1,
                current.val2);

            printf("INA219_0 Power: %d.%06d W\n\n",
                power.val1,
                power.val2);
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

            printf("INA219_1 Bus: %d.%06d V\n",
                bus_voltage.val1,
                bus_voltage.val2);

            printf("INA219_1 Current: %d.%06d A\n",
                current.val1,
                current.val2);

            printf("INA219_1 Power: %d.%06d W\n\n",
                power.val1,
                power.val2);
        }

        k_msleep(1000);
    }

    return 0;
}
