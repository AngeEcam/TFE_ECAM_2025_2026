#include "sensors.h"

capteur_t capteurs[] = {
    { 0x01, "Frigo 1", TYPE_S_THP_01A, 0x0000,100, 0x0001,100, 0x0003,10 },
    { 0x02, "Frigo 2", TYPE_S_THP_01A, 0x0000,100, 0x0001,100, 0x0003,10 },
    { 0x03, "Frigo 3", TYPE_S_THP_01A, 0x0000,100, 0x0001,100, 0x0003,10 },
    { 0x04, "Incubateur", TYPE_S_THP_01A, 0x0000,100, 0x0001,100, 0x0003,10 },
    { 0x05, "Congelateur", TYPE_PT100, 0x0000,10, 0,0, 0,0 },
    { 0x05, "Four", TYPE_PT100, 0x0001,10, 0,0, 0,0 },
    { 0x06, "Local", TYPE_XYMD04, 0x0001,10, 0x0002,10, 0,0 },
};

const int NB_CAPTEURS = sizeof(capteurs) / sizeof(capteurs[0]);
