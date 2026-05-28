#include "app.h"

#include <zephyr/drivers/uart.h>
#include <zephyr/modbus/modbus.h>
#include <zephyr/sys/printk.h>

#include <errno.h>

static uint16_t u32_reg(uint32_t value, uint16_t word)
{
    return (uint16_t)(value >> (word * 16U));
}

static uint16_t i32_reg(int32_t value, uint16_t word)
{
    return u32_reg((uint32_t)value, word);
}

static uint16_t f_to_s16_reg(float value, float scale)
{
    return (uint16_t)(int16_t)(value * scale);
}

static bool lid_closed_from_mv(int32_t mv)
{
    return mv >= LID_CLOSED_THRESHOLD_MV;
}

static int discrete_input_rd(uint16_t addr, bool *state)
{
    struct status s = app_status_get();
    int32_t lid_mv = (int32_t)(s.temp.stm32_v * 1000.0f);

    switch (addr) {
    case 0:
        *state = lid_closed_from_mv(lid_mv);
        return 0;
    case 1:
        *state = s.power[0].ok;
        return 0;
    case 2:
        *state = s.power[1].ok;
        return 0;
    case 3:
        *state = app_lse_ready();
        return 0;
    default:
        return -ENOTSUP;
    }
}

static int input_reg_rd(uint16_t addr, uint16_t *reg)
{
    struct status s = app_status_get();
    int32_t stm32_mv = (int32_t)(s.temp.stm32_v * 1000.0f);

    switch (addr) {
    case 0:
        *reg = 1U;
        return 0;
    case 1:
        *reg = u32_reg(s.seq, 0);
        return 0;
    case 2:
        *reg = u32_reg(s.seq, 1);
        return 0;
    case 3:
        *reg = u32_reg((uint32_t)s.time_ms, 0);
        return 0;
    case 4:
        *reg = u32_reg((uint32_t)s.time_ms, 1);
        return 0;
    case 10:
        *reg = f_to_s16_reg(s.temp.pt_c, 100.0f);
        return 0;
    case 11:
        *reg = f_to_s16_reg(s.temp.lm_c, 100.0f);
        return 0;
    case 12:
        *reg = (uint16_t)(int16_t)stm32_mv;
        return 0;
    case 13:
        *reg = i32_reg(s.temp.pt_raw, 0);
        return 0;
    case 14:
        *reg = i32_reg(s.temp.pt_raw, 1);
        return 0;
    case 15:
        *reg = i32_reg(s.temp.lm_raw, 0);
        return 0;
    case 16:
        *reg = i32_reg(s.temp.lm_raw, 1);
        return 0;
    case 20:
        *reg = f_to_s16_reg(s.pwm_pt, 10.0f);
        return 0;
    case 21:
        *reg = f_to_s16_reg(s.pwm_lm, 10.0f);
        return 0;
    case 22:
        *reg = f_to_s16_reg(s.target_pt, 100.0f);
        return 0;
    case 23:
        *reg = f_to_s16_reg(s.target_lm, 100.0f);
        return 0;
    case 30:
    case 31:
        *reg = i32_reg(s.power[0].voltage_mv, addr - 30);
        return 0;
    case 32:
    case 33:
        *reg = i32_reg(s.power[0].current_ua, addr - 32);
        return 0;
    case 34:
    case 35:
        *reg = i32_reg(s.power[0].power_uw, addr - 34);
        return 0;
    case 40:
    case 41:
        *reg = i32_reg(s.power[1].voltage_mv, addr - 40);
        return 0;
    case 42:
    case 43:
        *reg = i32_reg(s.power[1].current_ua, addr - 42);
        return 0;
    case 44:
    case 45:
        *reg = i32_reg(s.power[1].power_uw, addr - 44);
        return 0;
    default:
        return -ENOTSUP;
    }
}

static int holding_reg_rd(uint16_t addr, uint16_t *reg)
{
    float pt;
    float lm;

    app_targets_get(&pt, &lm);

    switch (addr) {
    case 0:
        *reg = f_to_s16_reg(pt, 100.0f);
        return 0;
    case 1:
        *reg = f_to_s16_reg(lm, 100.0f);
        return 0;
    default:
        return -ENOTSUP;
    }
}

static int holding_reg_wr(uint16_t addr, uint16_t reg)
{
    float pt;
    float lm;
    float value = (float)(int16_t)reg / 100.0f;

    app_targets_get(&pt, &lm);

    switch (addr) {
    case 0:
        app_targets_set(value, lm);
        return 0;
    case 1:
        app_targets_set(pt, value);
        return 0;
    default:
        return -ENOTSUP;
    }
}

static struct modbus_user_callbacks mbs_cbs = {
    .discrete_input_rd = discrete_input_rd,
    .input_reg_rd = input_reg_rd,
    .holding_reg_rd = holding_reg_rd,
    .holding_reg_wr = holding_reg_wr,
};

static const struct modbus_iface_param server_param = {
    .mode = MODBUS_MODE_RTU,
    .server = {
        .user_cb = &mbs_cbs,
        .unit_id = MODBUS_UNIT_ID,
    },
    .serial = {
        .baud = MODBUS_BAUD,
        .parity = UART_CFG_PARITY_NONE,
    },
};

int app_modbus_server_init(void)
{
    const char iface_name[] = DEVICE_DT_NAME(MODBUS_NODE);
    const struct uart_config rs485_cfg = {
        .baudrate = MODBUS_BAUD,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_2,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_RS485,
    };
    int iface = modbus_iface_get_by_name(iface_name);
    int err;

    if (iface < 0) {
        printk("Failed to get Modbus iface %s: %d\n", iface_name, iface);
        return iface;
    }

    err = modbus_init_server(iface, server_param);
    if (err != 0) {
        return err;
    }

    /* Zephyr Modbus reconfigures UART flow control. Restore STM32 hardware DE
     * mode so PA12 drives the RS-485 transceiver direction automatically.
     */
    return uart_configure(app_uart_rs485, &rs485_cfg);
}
