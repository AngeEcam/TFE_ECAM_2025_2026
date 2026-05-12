#include "xymd04.h"
#include "../modbus/modbus.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(xymd04, LOG_LEVEL_DBG);

bool xymd04_read(const struct device *uart,
                 uint8_t slave_id,
                 xymd04_data_t *data)
{
    uint8_t resp[XYMD04_RESP_LEN] = {0};

    int n = modbus_read_input_registers(uart,
                                        slave_id,
                                        XYMD04_REG_START,
                                        XYMD04_REG_COUNT,
                                        resp,
                                        sizeof(resp));

    if (n != XYMD04_RESP_LEN) {
        LOG_WRN("Reponse incomplete : %d bytes recus", n);
        return false;
    }
    if (resp[0] != slave_id || resp[1] != 0x04) {
        LOG_WRN("Reponse invalide : ID=0x%02X FC=0x%02X",
                resp[0], resp[1]);
        return false;
    }

    uint16_t raw_temp = (resp[3] << 8) | resp[4];
    uint16_t raw_hum  = (resp[5] << 8) | resp[6];

    /* Températures négatives (signed 16-bit) */
    data->temperature = (raw_temp > 32767) ?
                        (raw_temp - 65536) / 10.0f :
                        raw_temp / 10.0f;
    data->humidity    = raw_hum / 10.0f;

    LOG_DBG("Temp: %.1f C | Hum: %.1f %%RH",
            (double)data->temperature,
            (double)data->humidity);

    return true;
}