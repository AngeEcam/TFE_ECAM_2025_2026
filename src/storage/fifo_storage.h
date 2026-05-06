/*
 * fifo_storage.h — Stockage FIFO circulaire pour mesures capteurs
 *
 * Capacité : FIFO_SIZE entrées × 16 bytes
 * RAM requise : 2160 × 16 = 34 560 bytes (~34 Ko)
 *
 * nRF52832 RAM totale : 64 Ko — adapter FIFO_SIZE si nécessaire.
 * Utiliser CONFIG_THREAD_MONITOR + LOG pour mesurer l'empreinte réelle.
 *
 * Thread-safety : accès protégé par mutex Zephyr (k_mutex).
 * ISR : ne pas appeler depuis un ISR (mutex non IRQ-safe).
 */

#ifndef FIFO_STORAGE_H
#define FIFO_STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ─── Dimensionnement ───────────────────────────────────────────────────────
 *  Période d'acquisition : 10 s  →  360 mesures/heure
 *  FIFO_SIZE = 2160 : couverture 6 h (≈ 34 Ko RAM)
 *  Pour 24 h il faudrait 8640 × 16 = 135 Ko > RAM disponible.
 *  Augmenter FIFO_SIZE uniquement après vérification de l'usage RAM réel.
 */
#define FIFO_SIZE       1200U   /* nombre maximum d'entrées                 */
#define FIFO_SIZE_BYTES (FIFO_SIZE * sizeof(sensor_record_t))  /* ≈ 34 Ko  */

/* ─── Identificateurs de capteurs ──────────────────────────────────────────*/
#define SENSOR_ID_STHP01A    1U  /* Temp / Hum / Press  — slave 1, FC04    */
#define SENSOR_ID_SHT20      2U  /* Temp / Hum          — slave 2, FC04    */
#define SENSOR_ID_PT100_CH1  3U  /* Temp canal 1        — slave 3, FC03    */
#define SENSOR_ID_PT100_CH2  4U  /* Temp canal 2        — slave 3, FC03    */

/* ─── Flags de validité ────────────────────────────────────────────────────*/
#define RECORD_FLAG_VALID    0x01U  /* mesure correcte                      */
#define RECORD_FLAG_ERROR    0x02U  /* erreur Modbus au moment de l'acq.    */
#define RECORD_FLAG_TIMEOUT  0x04U  /* timeout réception RS485              */

/* ─── Structure d'une mesure — 16 bytes, alignée sur 4 ─────────────────────
 *
 *  Unités (entiers pour éviter le FP) :
 *    temp     : dixièmes de °C  (ex. 235  → 23,5 °C ; -50 → -5,0 °C)
 *    humidity : dixièmes de %HR (ex. 482  → 48,2 %HR)
 *    pressure : dixièmes de hPa (ex. 10131 → 1013,1 hPa)
 *
 *  timestamp : uptime en secondes (k_uptime_get_32() / 1000).
 *              Débordement à 49 710 jours — suffisant pour ce projet.
 */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t timestamp;   /* uptime en secondes depuis boot                  */
    int16_t  temp;        /* température × 10  (signé : températures < 0)   */
    uint16_t humidity;    /* humidité    × 10                                */
    uint16_t pressure;    /* pression    × 10  (0 si capteur sans pression)  */
    uint8_t  sensor_id;   /* SENSOR_ID_*                                     */
    uint8_t  flags;       /* RECORD_FLAG_*                                   */
    uint8_t  _pad[2];     /* padding → sizeof == 16, aligné sur 4            */
} sensor_record_t;

/* Vérification statique de la taille à la compilation */
BUILD_ASSERT(sizeof(sensor_record_t) == 16,
             "sensor_record_t doit faire exactement 16 bytes");

/* ─── API publique ──────────────────────────────────────────────────────────*/

/**
 * @brief  Initialise le FIFO et le mutex associé.
 *         Doit être appelé une seule fois dans main() avant tout push/pop.
 */
void fifo_storage_init(void);

/**
 * @brief  Insère une mesure en queue du FIFO (politique circulaire).
 *
 *         Si le FIFO est plein, la mesure la plus ancienne (tête) est
 *         silencieusement écrasée et un compteur d'overflows est incrémenté.
 *
 * @param  record  Pointeur vers la mesure à copier dans le buffer.
 *                 Le pointeur peut être libéré après l'appel.
 * @return true  si insertion sans écrasement
 *         false si la mesure la plus ancienne a été écrasée (FIFO plein)
 */
bool fifo_push(const sensor_record_t *record);

/**
 * @brief  Extrait la mesure la plus ancienne (tête) du FIFO.
 *
 * @param  out  Destination où copier la mesure extraite.
 * @return true  si une mesure a été copiée dans *out
 *         false si le FIFO est vide (out non modifié)
 */
bool fifo_pop(sensor_record_t *out);

/**
 * @brief  Lit la mesure à l'index idx sans la retirer (lecture non-destructive).
 *
 *         idx=0 → mesure la plus ancienne (tête du FIFO).
 *         idx=(count-1) → mesure la plus récente.
 *
 * @param  idx  Indice relatif dans le FIFO [0, fifo_count()-1].
 * @param  out  Destination.
 * @return true si idx valide et mesure copiée, false sinon.
 */
bool fifo_peek(uint32_t idx, sensor_record_t *out);

/**
 * @brief  Nombre d'entrées actuellement stockées dans le FIFO.
 */
uint32_t fifo_count(void);

/**
 * @brief  Indique si le FIFO est vide.
 */
bool fifo_is_empty(void);

/**
 * @brief  Indique si le FIFO est plein.
 */
bool fifo_is_full(void);

/**
 * @brief  Vide complètement le FIFO (reset head, tail, count).
 *         À appeler après une transmission BLE réussie si désiré.
 */
void fifo_clear(void);

/**
 * @brief  Nombre de fois où un push a écrasé la mesure la plus ancienne.
 *         Utile pour le monitoring de la qualité de la couverture réseau.
 */
uint32_t fifo_overflow_count(void);

/**
 * @brief  Remplit *stats avec un snapshot des métriques du FIFO.
 *         Appel atomique (protégé par mutex).
 */
typedef struct {
    uint32_t count;          /* entrées actuellement stockées               */
    uint32_t capacity;       /* FIFO_SIZE                                   */
    uint32_t overflows;      /* écrasements depuis le dernier clear         */
    uint32_t oldest_ts;      /* timestamp de la mesure en tête (0 si vide) */
    uint32_t newest_ts;      /* timestamp de la mesure en queue (0 si vide)*/
} fifo_stats_t;

void fifo_get_stats(fifo_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* FIFO_STORAGE_H */