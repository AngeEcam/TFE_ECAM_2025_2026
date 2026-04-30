#include "sthp01a.h"
#include "../modbus/modbus.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sthp01a, LOG_LEVEL_DBG);

bool sthp01a_read(const struct device *uart,
                  uint8_t slave_id,
                  sthp01a_data_t *data)
{
    uint8_t resp[STHP01A_RESP_LEN] = {0};

    int n = modbus_read_input_registers(uart,
                                        slave_id,
                                        STHP01A_REG_START,
                                        STHP01A_REG_COUNT,
                                        resp,
                                        sizeof(resp));

    /* Validation */
    if (n != STHP01A_RESP_LEN) {
        LOG_WRN("Reponse incomplete : %d bytes recus", n);
        return false;
    }
    if (resp[0] != slave_id || resp[1] != 0x04) {
        LOG_WRN("Reponse invalide : ID=0x%02X FC=0x%02X",
                resp[0], resp[1]);
        return false;
    }

    /* Parsing */
    int16_t raw_temp  = (resp[3] << 8) | resp[4];
    int16_t raw_hum   = (resp[5] << 8) | resp[6];
    int16_t raw_press = (resp[9] << 8) | resp[10];

    data->temperature = raw_temp  / 100.0f;
    data->humidity    = raw_hum   / 100.0f;
    data->pressure    = raw_press / 10.0f;

    LOG_DBG("Temp: %.2f C | Hum: %.2f %%RH | Press: %.1f hPa",
            (double)data->temperature,
            (double)data->humidity,
            (double)data->pressure);

    return true;
}