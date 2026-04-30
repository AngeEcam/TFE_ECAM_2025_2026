#ifndef STHP01A_H
#define STHP01A_H

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

/* Slave ID par défaut */
#define STHP01A_DEFAULT_SLAVE_ID  1

/* Registres */
#define STHP01A_REG_START  0x0000
#define STHP01A_REG_COUNT  4

/* Taille réponse Modbus :
   1 (ID) + 1 (FC) + 1 (byte count) + 8 (4 regs x 2) + 2 (CRC) */
#define STHP01A_RESP_LEN   13

/* Structure de données du capteur */
typedef struct {
    float temperature;  /* °C  */
    float humidity;     /* %RH */
    float pressure;     /* hPa */
} sthp01a_data_t;

/* Lit les valeurs du capteur, retourne true si succès */
bool sthp01a_read(const struct device *uart,
                  uint8_t slave_id,
                  sthp01a_data_t *data);

#endif /* STHP01A_H */