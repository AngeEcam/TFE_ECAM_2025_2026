#ifndef XYMD04_H
#define XYMD04_H

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#define XYMD04_DEFAULT_SLAVE_ID  2

#define XYMD04_REG_START  0x0001
#define XYMD04_REG_COUNT  2
#define XYMD04_RESP_LEN   9

typedef struct {
    float temperature;  /* °C  */
    float humidity;     /* %RH */
} xymd04_data_t;

bool xymd04_read(const struct device *uart,
                 uint8_t slave_id,
                 xymd04_data_t *data);

#endif /* XYMD04_H */