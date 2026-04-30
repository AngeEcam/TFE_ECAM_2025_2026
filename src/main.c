#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include "sensors/sthp01a.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define SLAVE_ID        1
#define READ_INTERVAL_S 5

int main(void)
{
    LOG_INF("=== nRF52 Monitoring – STHP01A ===");

    const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
    if (!device_is_ready(uart)) {
        LOG_ERR("uart0 non pret");
        return -1;
    }
    LOG_INF("uart0 pret");

    while (1) {
        sthp01a_data_t data;

        if (sthp01a_read(uart, SLAVE_ID, &data)) {
            LOG_INF("Temp  : %.2f C",    (double)data.temperature);
            LOG_INF("Hum   : %.2f %%RH", (double)data.humidity);
            LOG_INF("Press : %.1f hPa",  (double)data.pressure);
        } else {
            LOG_WRN("Lecture capteur echouee");
        }

        k_sleep(K_SECONDS(READ_INTERVAL_S));
    }
}