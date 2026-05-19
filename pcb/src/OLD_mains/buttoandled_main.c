#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

//LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

static const int32_t sleep_time_ms = 100;
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(button_blue), gpios);
static const struct gpio_dt_spec ledas = GPIO_DT_SPEC_GET(DT_ALIAS(led_ukas), gpios);


int main(void)
{
    int ret;
    int state;

    if(!gpio_is_ready_dt(&btn)) {
        printk("ERROR: button not ready\n");
        return 0;
    }

    ret = gpio_pin_configure_dt(&btn, GPIO_INPUT);
    if(ret < 0) {
        printk("ERROR: failed to configure button pin\n");
        return 0;
    }

    printk("Button spec flags: 0x%x\n", btn.dt_flags);

    if (!gpio_is_ready_dt(&ledas)) {
        printk("ERROR: LED not ready\n");
        return 0;
    }

    ret = gpio_pin_configure_dt(&ledas, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        printk("ERROR: failed to configure LED pin\n");
        return 0;
    }

    while(1){
        state = gpio_pin_get_dt(&btn);
        if(state < 0) {
            printk("ERROR %d: failed to read button state\n", state);
        
        }
        else {
            printk("INFO: button state is %d\n", state);
            if(state) {
                gpio_pin_set_dt(&ledas, 1);
            }
            else {
                gpio_pin_set_dt(&ledas, 0);
            }
        }

        k_msleep(sleep_time_ms);
    }

    return 0;
}