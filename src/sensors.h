#pragma once
#include <stdint.h>

typedef enum {
    TYPE_S_THP_01A,
    TYPE_PT100,
    TYPE_SHT20,
    TYPE_XYMD04,
} capteur_type_t;

typedef struct {
    uint8_t         adresse;
    const char     *nom;
    capteur_type_t  type;

    uint16_t reg_temp;
    uint8_t  facteur_temp;

    uint16_t reg_hum;
    uint8_t  facteur_hum;

    uint16_t reg_pres;
    uint8_t  facteur_pres;
} capteur_t;

extern capteur_t capteurs[];
extern const int NB_CAPTEURS;