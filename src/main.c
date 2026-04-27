#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

/*Dinstinction des types de capteurs*/

typedef enum {
    TYPE_S_THP_01A,   /* temp + hum + pres  Frigo/Incubateur */
    TYPE_PT100,   /* temp seulement    Four/Congélateur */
    TYPE_SHT20,    /*temp + hum Local */
    TYPE_XYMD04, /* temp + hum  Frigo  */
} capteur_type_t;

/* définition de la structure pour un capteur */
typedef struct {
    uint8_t         adresse;
    char            nom[20];
    capteur_type_t  type;

    uint16_t reg_temp;
    uint8_t  facteur_temp;

    /* Seulement pour TYPE_SHT20, TYPE_S_THP_01A, TYPE_XYMDD04 */
    uint16_t reg_hum;
    uint8_t  facteur_hum;

    /* Seulement pour TYPE_S_THP_01A */
    uint16_t reg_pres;
    uint8_t  facteur_pres;
} capteur_t;

/* Liste de tous les capteurs sur le bus */
static capteur_t capteurs[] = {

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
#define NB_CAPTEURS (sizeof(capteurs) / sizeof(capteurs[0]))


/* Buffer trame Modbus RTU qu'on envoie, 8 octets */
static uint8_t modbus_req[8];

/* Buffer de réception, 32 octets (suffisant pour 7 octets de réponse) */
static uint8_t rx_buf[32];

/* compteur qui suit combien d'octets ont été reçus dans rx_buf */
static volatile int rx_idx = 0; /* volatile car modifié dans le callback UART*/

/* Flag indiquant qu'une réponse est prête */
static volatile bool response_ready = false;

/* Pointeur vers le périphérique UART */
static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

/* Callback UART — réponse = toujours 7 octets (1 registre lu) */
static void uart_cb(const struct device *dev, void *user_data)
{
    uint8_t c;
    if (!uart_irq_update(dev)) return;
    while (uart_irq_rx_ready(dev)) {
        uart_fifo_read(dev, &c, 1);
        if (rx_idx < sizeof(rx_buf)) {
            rx_buf[rx_idx++] = c;
        }
        /* 1 registre = 7 octets : adresse + fonction + byte_count + 2 data + 2 CRC */
        if (rx_idx >= 7) {
            response_ready = true;
        }
    }
}

/* Calcul CRC16 Modbus */
static uint16_t crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else              crc >>= 1;
        }
    }
    return crc;
}

/* Envoie une requête Modbus et attend la réponse */
static void envoyer_requete(uint8_t adresse, uint16_t registre)
{
    modbus_req[0] =  adresse;
    modbus_req[1] =  0x03;
    modbus_req[2] = (registre >> 8) & 0xFF;
    modbus_req[3] =  registre & 0xFF;
    modbus_req[4] =  0x00;
    modbus_req[5] =  0x01;  /* toujours 1 registre à la fois */

    uint16_t crc  = crc16(modbus_req, 6);
    modbus_req[6] = crc & 0xFF;
    modbus_req[7] = (crc >> 8) & 0xFF;

    rx_idx = 0;
    response_ready = false;
    memset(rx_buf, 0, sizeof(rx_buf));

    for (int j = 0; j < sizeof(modbus_req); j++) {
        uart_poll_out(uart, modbus_req[j]);
    }

    int timeout = 500;
    while (!response_ready && timeout-- > 0) {
        k_msleep(1);
    }
}

/* Vérifie le CRC et retourne la valeur brute, -32768 si erreur */
static int16_t lire_registre(const char *nom, const char *mesure)
{
    uint16_t crc_calc = crc16(rx_buf, 5);
    uint16_t crc_recv = (rx_buf[6] << 8) | rx_buf[5];

    if (crc_calc != crc_recv) {
        printk("[%s] ERREUR CRC %s\n", nom, mesure);
        return -32768;  /* valeur sentinelle = erreur */
    }

    return (rx_buf[3] << 8) | rx_buf[4];
}

int main(void)
{
    if (!device_is_ready(uart)) {
        printk("UART non pret\n");
        return -1;
    }

    uart_irq_callback_set(uart, uart_cb); 
    uart_irq_rx_enable(uart);

    printk("Modbus master demarre — %d capteurs\n", NB_CAPTEURS);

    while (1) {
        for (int i = 0; i < NB_CAPTEURS; i++) {

            /* --- Température (tous les capteurs) --- */
            envoyer_requete(capteurs[i].adresse, capteurs[i].reg_temp);
            if (response_ready) {
                int16_t raw = lire_registre(capteurs[i].nom, "temp");
                if (raw != -32768) {
                    printk("[%s] Temp: %d.%02d C\n",
                        capteurs[i].nom,
                        raw / capteurs[i].facteur_temp,
                        raw % capteurs[i].facteur_temp);
                }
            } else {
                printk("[%s] Timeout temp\n", capteurs[i].nom);
            }
            k_msleep(50);

            /* --- Humidité (SHT20, S-THP-01A, XYMD04) --- */
            if (capteurs[i].type == TYPE_S_THP_01A ||
                capteurs[i].type == TYPE_XYMD04) {

                envoyer_requete(capteurs[i].adresse, capteurs[i].reg_hum);
                if (response_ready) {
                    int16_t raw = lire_registre(capteurs[i].nom, "hum");
                    if (raw != -32768) {
                        printk("[%s] Hum: %d.%02d %%\n",
                            capteurs[i].nom,
                            raw / capteurs[i].facteur_hum,
                            raw % capteurs[i].facteur_hum);
                    }
                } else {
                    printk("[%s] Timeout hum\n", capteurs[i].nom);
                }
                k_msleep(50);
            }

            /* --- Pression (S-THP-01A seulement) --- */
            if (capteurs[i].type == TYPE_S_THP_01A) {

                envoyer_requete(capteurs[i].adresse, capteurs[i].reg_pres);
                if (response_ready) {
                    int16_t raw = lire_registre(capteurs[i].nom, "pres");
                    if (raw != -32768) {
                        printk("[%s] Pres: %d.%01d hPa\n",
                            capteurs[i].nom,
                            raw / capteurs[i].facteur_pres,
                            raw % capteurs[i].facteur_pres);
                    }
                } else {
                    printk("[%s] Timeout pres\n", capteurs[i].nom);
                }
                k_msleep(50);
            }
        }

        k_sleep(K_SECONDS(60));
    }

    return 0;
}