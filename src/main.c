#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include "sensors/sthp01a.h"
#include "sensors/hailege_sht20.h"
#include "sensors/pt100_pta8c04.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define SLAVE_ID_STHP01A   1
#define SLAVE_ID_HAILEGE   2
#define SLAVE_ID_PT100     3
#define READ_INTERVAL_S    30

int main(void)
{
    LOG_INF("=== nRF52 Monitoring ===");

    const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
    if (!device_is_ready(uart)) {
        LOG_ERR("uart0 non pret");
        return -1;
    }
    LOG_INF("uart0 pret");

    while (1) {
        sthp01a_data_t data1;
        if (sthp01a_read(uart, SLAVE_ID_STHP01A, &data1)) {
            LOG_INF("[STHP01A] Temp  : %.2f C",    (double)data1.temperature);
            LOG_INF("[STHP01A] Hum   : %.2f %%RH", (double)data1.humidity);
            LOG_INF("[STHP01A] Press : %.1f hPa",  (double)data1.pressure);
        } else {
            LOG_WRN("[STHP01A] Lecture echouee");
        }

        k_sleep(K_MSEC(500));

        hailege_sht20_data_t data2;
        if (hailege_sht20_read(uart, SLAVE_ID_HAILEGE, &data2)) {
            LOG_INF("[HAILEGE] Temp  : %.1f C",    (double)data2.temperature);
            LOG_INF("[HAILEGE] Hum   : %.1f %%RH", (double)data2.humidity);
        } else {
            LOG_WRN("[HAILEGE] Lecture echouee");
        }

        k_sleep(K_MSEC(500));

        pt100_pta8c04_data_t data3;
        if (pt100_pta8c04_read(uart, SLAVE_ID_PT100, &data3)) {
            LOG_INF("[PT100] CH1 : %.1f C", (double)data3.ch1);
            LOG_INF("[PT100] CH2 : %.1f C", (double)data3.ch2);
        } else {
            LOG_WRN("[PT100] Lecture echouee");
        }

        k_sleep(K_SECONDS(READ_INTERVAL_S));
    }
}