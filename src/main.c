#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(modbus_test, LOG_LEVEL_DBG);

#define SLAVE_ID   1

static uint16_t crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
    }
    return crc;
}

int main(void)
{
    LOG_INF("=== Modbus minimal ===");

    const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
    if (!device_is_ready(uart)) {
        LOG_ERR("uart0 non pret");
        return -1;
    }
    LOG_INF("uart0 pret");

    while (1) {
        /* Forge trame FC04 */
        uint8_t req[8];
        req[0] = SLAVE_ID;
        req[1] = 0x04;
        req[2] = 0x00;
        req[3] = 0x00;
        req[4] = 0x00;
        req[5] = 0x04;
        uint16_t crc = crc16(req, 6);
        req[6] = crc & 0xFF;
        req[7] = crc >> 8;

        LOG_INF("TX: %02X %02X %02X %02X %02X %02X %02X %02X",
                req[0],req[1],req[2],req[3],
                req[4],req[5],req[6],req[7]);

        /* Envoie */
        for (int i = 0; i < 8; i++)
            uart_poll_out(uart, req[i]);

        /* Ecoute tout ce qui arrive pendant 1 seconde */
        uint32_t start = k_uptime_get_32();
        int n = 0;
        while (k_uptime_get_32() - start < 1000) {
            unsigned char c;
            if (uart_poll_in(uart, &c) == 0) {
                LOG_INF("RX[%d] = 0x%02X", n, c);
                n++;
            } else {
                k_sleep(K_USEC(100));
            }
        }

        if (n == 0)
            LOG_WRN("Aucune reponse");

        k_sleep(K_SECONDS(5));
    }
}