#include "app.h"

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>

#include <stm32_ll_rcc.h>

#include <stdio.h>

const struct device *const app_uart_rs485 = DEVICE_DT_GET(UART_RS485_NODE);
const struct device *const app_uart_serial = DEVICE_DT_GET(UART_SERIAL_NODE);
const struct device *const app_adc_pt1000 = DEVICE_DT_GET(DT_NODELABEL(ads1220_0));
const struct device *const app_adc_lm35 = DEVICE_DT_GET(DT_NODELABEL(ads1220_1));
const struct device *const app_adc_stm32 = DEVICE_DT_GET(DT_NODELABEL(adc1));
const struct device *const app_pwm_dev = DEVICE_DT_GET(PWM_NODE);
const struct device *const app_ina236_0 = DEVICE_DT_GET(DT_NODELABEL(ina236_0));
const struct device *const app_ina236_1 = DEVICE_DT_GET(DT_NODELABEL(ina236_1));

static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(LED1_NODE, gpios);

bool app_board_init(void)
{
    if (!device_is_ready(app_uart_rs485) ||
        !device_is_ready(app_uart_serial) ||
        !device_is_ready(app_adc_pt1000) ||
        !device_is_ready(app_adc_lm35) ||
        !device_is_ready(app_adc_stm32) ||
        !device_is_ready(app_pwm_dev)) {
        printk("Required device not ready\n");
        return false;
    }

    if (!gpio_is_ready_dt(&led1)) {
        printk("LED GPIO not ready\n");
        return false;
    }

    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);

    if (!device_is_ready(app_ina236_0)) {
        printk("INA236_0 not ready, telemetry will show invalid power 0\n");
    }
    if (!device_is_ready(app_ina236_1)) {
        printk("INA236_1 not ready, telemetry will show invalid power 1\n");
    }

    return true;
}

void app_led1_set(bool on)
{
    gpio_pin_set_dt(&led1, on);
}

void app_pwm_set(float pt_percent, float lm_percent)
{
    uint32_t pt_pulse = (uint32_t)((PWM_PERIOD_NS * pt_percent) / 100.0f);
    uint32_t lm_pulse = (uint32_t)((PWM_PERIOD_NS * lm_percent) / 100.0f);

    pwm_set(app_pwm_dev, PWM_CHANNEL_PT1000, PWM_PERIOD_NS, pt_pulse, 0);
    pwm_set(app_pwm_dev, PWM_CHANNEL_LM35, PWM_PERIOD_NS, lm_pulse, 0);
}

void app_serial_send(const char *str)
{
    while (*str != '\0') {
        uart_poll_out(app_uart_serial, *str++);
    }
}

void app_report_lse_status(void)
{
#if defined(RCC_BDCR_LSEON) && defined(RCC_BDCR_LSERDY)
    char buf[96];
    uint32_t bdcr = RCC->BDCR;

    snprintf(buf, sizeof(buf), "BDCR=0x%08lx LSEON=%d LSERDY=%d\r\n",
             (unsigned long)bdcr,
             (bdcr & RCC_BDCR_LSEON) != 0U,
             (bdcr & RCC_BDCR_LSERDY) != 0U);
    app_serial_send(buf);
    printk("%s", buf);
#endif
}

bool app_lse_ready(void)
{
#if defined(RCC_BDCR_LSERDY)
    return (RCC->BDCR & RCC_BDCR_LSERDY) != 0U;
#else
    return false;
#endif
}
