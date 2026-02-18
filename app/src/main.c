#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>

static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(ads1220));

static struct adc_channel_cfg ch_cfg = {
    .gain             = ADC_GAIN_1,
    .reference        = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME_DEFAULT,
    .channel_id       = 0,
    .differential     = false,
    .input_positive   = 0,  /* AIN0 — adjust to your wiring */
};

#define ADS1220_FULL_SCALE 8388607.0f
#define ADS1220_VREF_V     2.048f   // internal ref
#define ADS1220_GAIN       1.0f

int main(void)
{
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

    ret = adc_read(adc_dev, &seq);
    if (ret < 0) {
        printk("ADC read failed (%d)\n", ret);
        return ret;
    }

    printk("ADC raw value: %d\n", buf);
    
    float vin = ((float)buf / 8388607.0f) * 2.048f;

    int vin_mV = (int)(vin * 1000.0f);

    printk("Vin = %d.%03d V\n", vin_mV / 1000, vin_mV % 1000);


    return 0;
}