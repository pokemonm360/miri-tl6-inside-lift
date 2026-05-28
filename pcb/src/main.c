#include "app.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <stdio.h>

int main(void)
{
    int modbus_err;

    printk("SYSTEM START\n");

    if (!app_board_init()) {
        return 0;
    }

    k_sleep(K_SECONDS(2));
    app_report_lse_status();

    modbus_err = app_modbus_server_init();
    if (modbus_err != 0) {
        char buf[64];

        snprintf(buf, sizeof(buf), "Modbus RTU init failed: %d\r\n", modbus_err);
        app_serial_send(buf);
        printk("%s", buf);
        return 0;
    }

    app_serial_send("Modbus RTU server ready\r\n");
    app_control_start();
    app_telemetry_start();

    k_sleep(K_FOREVER);
    return 0;
}
