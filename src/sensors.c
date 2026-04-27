#include "sensors.h"
#include <stddef.h>


capteur_t capteurs[] = {
    /* --- Frigos 1, 2, 3 : S-THP-01A (temp + hum + pres) --- */
    { .adresse=0x01, .nom="Frigo 1",     .type=TYPE_S_THP_01A,
      .reg_temp=0x0000, .facteur_temp=100,
      .reg_hum=0x0001,  .facteur_hum=100,
      .reg_pres=0x0003, .facteur_pres=10 },

    { .adresse=0x02, .nom="Frigo 2",     .type=TYPE_S_THP_01A,
      .reg_temp=0x0000, .facteur_temp=100,
      .reg_hum=0x0001,  .facteur_hum=100,
      .reg_pres=0x0003, .facteur_pres=10 },

    { .adresse=0x03, .nom="Frigo 3",     .type=TYPE_S_THP_01A,
      .reg_temp=0x0000, .facteur_temp=100,
      .reg_hum=0x0001,  .facteur_hum=100,
      .reg_pres=0x0003, .facteur_pres=10 },

    /* --- Incubateur : S-THP-01A (temp + hum + pres) --- */
    { .adresse=0x04, .nom="Incubateur",  .type=TYPE_S_THP_01A,
      .reg_temp=0x0000, .facteur_temp=100,
      .reg_hum=0x0001,  .facteur_hum=100,
      .reg_pres=0x0003, .facteur_pres=10 },

    /* --- Congélateur : PT100 canal 0 (temp seulement) --- */
    { .adresse=0x05, .nom="Congelateur", .type=TYPE_PT100,
      .reg_temp=0x0000, .facteur_temp=10,
      .reg_hum=0,       .facteur_hum=0,
      .reg_pres=0,      .facteur_pres=0 },

    /* --- Four : PT100 canal 1 (temp seulement) --- */
    { .adresse=0x05, .nom="Four",        .type=TYPE_PT100,
      .reg_temp=0x0001, .facteur_temp=10,
      .reg_hum=0,       .facteur_hum=0,
      .reg_pres=0,      .facteur_pres=0 },

    /* --- Local : XY-MD04 (temp + hum, pas de pression) --- */
    { .adresse=0x06, .nom="Local",       .type=TYPE_XYMD04,
      .reg_temp=0x0001, .facteur_temp=10,
      .reg_hum=0x0002,  .facteur_hum=10,
      .reg_pres=0,      .facteur_pres=0 },
};

const size_t NB_CAPTEURS = sizeof(capteurs) / sizeof(capteurs[0]);