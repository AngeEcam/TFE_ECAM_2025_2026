#include "modbus.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(modbus, LOG_LEVEL_DBG);

uint16_t modbus_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

static void modbus_flush_rx(const struct device *uart)
{
    unsigned char c;
    while (uart_poll_in(uart, &c) == 0) {}
}

void modbus_send(const struct device *uart,
                 const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++)
        uart_poll_out(uart, buf[i]);
}

int modbus_recv(const struct device *uart,
                uint8_t *buf, size_t max_len,
                uint32_t timeout_ms)
{
    size_t idx = 0;
    uint32_t start     = k_uptime_get_32();
    uint32_t last_byte = start;

    while (idx < max_len) {
        if (k_uptime_get_32() - start > timeout_ms) break;

        unsigned char c;
        if (uart_poll_in(uart, &c) == 0) {
            buf[idx++] = c;
            last_byte  = k_uptime_get_32();
        } else {
            /* silence de 5ms = fin de trame Modbus RTU */
            if (idx > 0 && k_uptime_get_32() - last_byte > 5) break;
            k_sleep(K_USEC(100));
        }
    }

    LOG_DBG("RX %d bytes", (int)idx);
    return (int)idx;
}

int modbus_read_input_registers(const struct device *uart,
                                uint8_t slave_id,
                                uint16_t reg_start,
                                uint16_t reg_count,
                                uint8_t *resp,
                                size_t resp_len)
{
    uint8_t req[8];
    req[0] = slave_id;
    req[1] = MODBUS_FC_READ_INPUT_REGISTERS;
    req[2] = (reg_start >> 8) & 0xFF;
    req[3] =  reg_start       & 0xFF;
    req[4] = (reg_count >> 8) & 0xFF;
    req[5] =  reg_count       & 0xFF;
    uint16_t crc = modbus_crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = crc >> 8;

    LOG_DBG("TX FC04: %02X %02X %02X %02X %02X %02X %02X %02X",
            req[0],req[1],req[2],req[3],
            req[4],req[5],req[6],req[7]);

    modbus_flush_rx(uart);
    modbus_send(uart, req, sizeof(req));
    return modbus_recv(uart, resp, resp_len, 1000);
}

int modbus_read_holding_registers(const struct device *uart,
                                  uint8_t slave_id,
                                  uint16_t reg_start,
                                  uint16_t reg_count,
                                  uint8_t *resp,
                                  size_t resp_len)
{
    uint8_t req[8];
    req[0] = slave_id;
    req[1] = MODBUS_FC_READ_HOLDING_REGISTERS;
    req[2] = (reg_start >> 8) & 0xFF;
    req[3] =  reg_start       & 0xFF;
    req[4] = (reg_count >> 8) & 0xFF;
    req[5] =  reg_count       & 0xFF;
    uint16_t crc = modbus_crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = crc >> 8;

    LOG_DBG("TX FC03: %02X %02X %02X %02X %02X %02X %02X %02X",
            req[0],req[1],req[2],req[3],
            req[4],req[5],req[6],req[7]);

    modbus_flush_rx(uart);
    modbus_send(uart, req, sizeof(req));
    return modbus_recv(uart, resp, resp_len, 1000);
}