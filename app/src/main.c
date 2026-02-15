#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

//LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

static const int32_t sleep_time_ms = 1000;
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_ALIAS(button_blue), gpios);

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

    while(1){
        state = gpio_pin_get_dt(&btn);
        if(state < 0) {
            printk("ERROR %d: failed to read button state\n", state);
        
        }
        else {
            printk("INFO: button state is %d\n", state);
        }

        k_msleep(sleep_time_ms);
    }

    return 0;
}