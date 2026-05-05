#include "pt100_pta8c04.h"
#include "../modbus/modbus.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pt100_pta8c04, LOG_LEVEL_DBG);

bool pt100_pta8c04_read(const struct device *uart,
                        uint8_t slave_id,
                        pt100_pta8c04_data_t *data)
{
    uint8_t resp[PT100_RESP_LEN] = {0};

    int n = modbus_read_holding_registers(uart,
                                          slave_id,
                                          PT100_REG_START,
                                          PT100_REG_COUNT,
                                          resp,
                                          sizeof(resp));

    if (n != PT100_RESP_LEN) {
        LOG_WRN("Reponse incomplete : %d bytes recus", n);
        return false;
    }
    if (resp[0] != slave_id || resp[1] != 0x03) {
        LOG_WRN("Reponse invalide : ID=0x%02X FC=0x%02X",
                resp[0], resp[1]);
        return false;
    }

    uint16_t raw_ch1 = (resp[3] << 8) | resp[4];
    uint16_t raw_ch2 = (resp[5] << 8) | resp[6];

    /* Gestion températures négatives (signed 16-bit) */
    data->ch1 = (raw_ch1 > 32767) ? (raw_ch1 - 65536) / 10.0f
                                   : raw_ch1 / 10.0f;
    data->ch2 = (raw_ch2 > 32767) ? (raw_ch2 - 65536) / 10.0f
                                   : raw_ch2 / 10.0f;

    LOG_DBG("CH1: %.1f C | CH2: %.1f C",
            (double)data->ch1,
            (double)data->ch2);

    return true;
}