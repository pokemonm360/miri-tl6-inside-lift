#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <stdlib.h>


#define LED0_NODE DT_ALIAS(led0)

#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "led0 alias not defined"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(ads1220));

static struct adc_channel_cfg ch_cfg = {
    .gain             = ADC_GAIN_2,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = 0,
    .differential     = false,
    .input_positive   = 0,
};

#define ADS1220_FULL_SCALE 8388607.0f
#define ADS1220_VREF_V     2.048f
#define ADS1220_GAIN       1.0f

int main(void)
{
    if (!device_is_ready(led.port)) {
        return -ENODEV;
    }

    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

    printk("MAIN START\n");

    int ret;
    int32_t buf;

    struct adc_sequence seq = {
        .channels    = BIT(0),
        .buffer      = &buf,
        .buffer_size = sizeof(buf),
        .resolution  = 24,
    };

    if (!device_is_ready(adc_dev)) {
        printk("ADC device not ready\n");
        return -ENODEV;
    }

    ret = adc_channel_setup(adc_dev, &ch_cfg);
    if (ret < 0) {
        printk("Channel setup failed (%d)\n", ret);
        return ret;
    }

    printk("ADS1220 ready\n");

    while (1) {

        /* LED toggle kiekvieną ciklą */
        gpio_pin_toggle_dt(&led);

        ret = adc_read(adc_dev, &seq);
        if (ret < 0) {
            printk("ADC read failed (%d)\n", ret);
            continue;
        }

        printk("ADC raw: %d\n", buf);

        float vin = ((float)buf / ADS1220_FULL_SCALE) *
                    (ADS1220_VREF_V / ADS1220_GAIN);

        /* konvertuojam į mikrovoltus */
        int vin_uV = (int)(vin * 1000000.0f);

        printk("Vin = %d.%06d V\n",
            vin_uV / 1000000,
            abs(vin_uV % 1000000));


        k_msleep(1000);
    }

    return 0;
}
