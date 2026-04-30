#ifndef HAILEGE_SHT20_H
#define HAILEGE_SHT20_H

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#define HAILEGE_DEFAULT_SLAVE_ID  2

#define HAILEGE_REG_START  0x0001
#define HAILEGE_REG_COUNT  2
#define HAILEGE_RESP_LEN   9   /* 1+1+1+4+2 */

typedef struct {
    float temperature;  /* °C  */
    float humidity;     /* %RH */
} hailege_sht20_data_t;

bool hailege_sht20_read(const struct device *uart,
                        uint8_t slave_id,
                        hailege_sht20_data_t *data);

#endif /* HAILEGE_SHT20_H */