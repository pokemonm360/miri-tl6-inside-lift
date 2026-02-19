#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>
#include <math.h>

#define LED0_NODE DT_ALIAS(led0)

#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "led0 alias not defined"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* Abu ADS1220 */
static const struct device *adc_pt1000 =
    DEVICE_DT_GET(DT_NODELABEL(ads1220_0));

static const struct device *adc_lm35 =
    DEVICE_DT_GET(DT_NODELABEL(ads1220_1));

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

int main(void)
{
    if (!device_is_ready(led.port)) {
        return -ENODEV;
    }

    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

    printk("MAIN START\n");

    if (!device_is_ready(adc_pt1000) ||
        !device_is_ready(adc_lm35)) {
        printk("ADC device not ready\n");
        return -ENODEV;
    }

    int32_t buf_pt;
    int32_t buf_lm;
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

    printk("ADS1220 ready\n");

    while (1) {

        gpio_pin_toggle_dt(&led);

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

        k_msleep(1000);
    }

    return 0;
}
