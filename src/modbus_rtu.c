#include "modbus_rtu.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <stddef.h>

static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

static uint16_t crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }
    return crc;
}

bool modbus_read_register(uint8_t addr, uint16_t reg, int16_t *value)
{
    uint8_t req[8];
    uint8_t resp[7];

    if (!device_is_ready(uart)) {
        return false;
    }

    req[0] = addr;
    req[1] = 0x03;
    req[2] = reg >> 8;
    req[3] = reg & 0xFF;
    req[4] = 0x00;
    req[5] = 0x01;

    uint16_t crc = crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = crc >> 8;

    for (int i = 0; i < 8; i++) {
        uart_poll_out(uart, req[i]);
    }

    for (int i = 0; i < 7; i++) {
        int timeout = 100;
        while (uart_poll_in(uart, &resp[i]) != 0) {
            k_msleep(1);
            if (--timeout <= 0) {
                return false;
            }
        }
    }

    if (resp[1] & 0x80) {
        return false;
    }

    uint16_t crc_calc = crc16(resp, 5);
    uint16_t crc_recv = resp[5] | (resp[6] << 8);

    if (crc_calc != crc_recv) {
        return false;
    }

    *value = (resp[3] << 8) | resp[4];
    return true;
}
