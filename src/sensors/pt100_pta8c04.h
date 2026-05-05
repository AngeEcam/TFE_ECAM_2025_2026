#ifndef PT100_PTA8C04_H
#define PT100_PTA8C04_H

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#define PT100_DEFAULT_SLAVE_ID  3

#define PT100_REG_START  0x0000
#define PT100_REG_COUNT  2
#define PT100_RESP_LEN   9   /* 1+1+1+4+2 */

typedef struct {
    float ch1;  /* °C */
    float ch2;  /* °C */
} pt100_pta8c04_data_t;

bool pt100_pta8c04_read(const struct device *uart,
                        uint8_t slave_id,
                        pt100_pta8c04_data_t *data);

#endif /* PT100_PTA8C04_H */