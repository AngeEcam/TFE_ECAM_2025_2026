/*
 * fifo_storage.c — Implémentation du ring buffer pour mesures capteurs
 *
 * Stratégie :
 *   - Buffer statique en .bss (zéro à l'init, pas de malloc).
 *   - Indices head et tail dans [0, FIFO_SIZE).
 *   - Accès protégé par k_mutex (compatible threads Zephyr, pas IRQ).
 *   - Politique overwrite : si plein, head avance avant d'écrire en tail.
 */

#include "fifo_storage.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(fifo_storage, LOG_LEVEL_DBG);

/* ─── Buffer statique ───────────────────────────────────────────────────────*/
static sensor_record_t s_buf[FIFO_SIZE];   /* ~34 Ko en .bss               */
static uint32_t        s_head;             /* index de la plus ancienne     */
static uint32_t        s_tail;             /* index où écrire la prochaine  */
static uint32_t        s_count;            /* nombre d'entrées valides      */
static uint32_t        s_overflows;        /* compteur d'écrasements        */
static bool            s_initialized;

K_MUTEX_DEFINE(s_fifo_mutex);

/* ─── Helpers internes (appelés mutex déjà tenu) ────────────────────────────*/

static inline uint32_t next_idx(uint32_t idx)
{
    return (idx + 1U >= FIFO_SIZE) ? 0U : idx + 1U;
}

/* ─── API publique ──────────────────────────────────────────────────────────*/

void fifo_storage_init(void)
{
    k_mutex_lock(&s_fifo_mutex, K_FOREVER);

    memset(s_buf, 0, sizeof(s_buf));
    s_head        = 0U;
    s_tail        = 0U;
    s_count       = 0U;
    s_overflows   = 0U;
    s_initialized = true;

    k_mutex_unlock(&s_fifo_mutex);

    LOG_INF("FIFO initialisé : %u slots × %u bytes = %u bytes (~%u Ko)",
            FIFO_SIZE,
            (uint32_t)sizeof(sensor_record_t),
            FIFO_SIZE_BYTES,
            FIFO_SIZE_BYTES / 1024U);
}

bool fifo_push(const sensor_record_t *record)
{
    bool overwritten = false;

    if (!record) {
        LOG_ERR("fifo_push: pointeur NULL");
        return false;
    }

    k_mutex_lock(&s_fifo_mutex, K_FOREVER);

    if (!s_initialized) {
        k_mutex_unlock(&s_fifo_mutex);
        LOG_ERR("fifo_push: FIFO non initialisé");
        return false;
    }

    if (s_count == FIFO_SIZE) {
        /* FIFO plein : on écrase la plus ancienne */
        s_head = next_idx(s_head);
        s_overflows++;
        overwritten = true;
        LOG_WRN("FIFO plein — écrasement (total overflows: %u)", s_overflows);
    } else {
        s_count++;
    }

    memcpy(&s_buf[s_tail], record, sizeof(sensor_record_t));
    s_tail = next_idx(s_tail);

    k_mutex_unlock(&s_fifo_mutex);

    LOG_DBG("fifo_push: sensor=%u ts=%u temp=%d — count=%u/%u",
            record->sensor_id, record->timestamp, record->temp,
            s_count, FIFO_SIZE);

    return !overwritten;
}

bool fifo_pop(sensor_record_t *out)
{
    if (!out) {
        LOG_ERR("fifo_pop: pointeur NULL");
        return false;
    }

    k_mutex_lock(&s_fifo_mutex, K_FOREVER);

    if (s_count == 0U) {
        k_mutex_unlock(&s_fifo_mutex);
        return false;
    }

    memcpy(out, &s_buf[s_head], sizeof(sensor_record_t));
    s_head = next_idx(s_head);
    s_count--;

    k_mutex_unlock(&s_fifo_mutex);

    LOG_DBG("fifo_pop: sensor=%u ts=%u — count restant=%u",
            out->sensor_id, out->timestamp, s_count);

    return true;
}

bool fifo_peek(uint32_t idx, sensor_record_t *out)
{
    if (!out) {
        return false;
    }

    k_mutex_lock(&s_fifo_mutex, K_FOREVER);

    if (idx >= s_count) {
        k_mutex_unlock(&s_fifo_mutex);
        return false;
    }

    uint32_t real_idx = (s_head + idx) % FIFO_SIZE;
    memcpy(out, &s_buf[real_idx], sizeof(sensor_record_t));

    k_mutex_unlock(&s_fifo_mutex);
    return true;
}

uint32_t fifo_count(void)
{
    k_mutex_lock(&s_fifo_mutex, K_FOREVER);
    uint32_t c = s_count;
    k_mutex_unlock(&s_fifo_mutex);
    return c;
}

bool fifo_is_empty(void)
{
    return fifo_count() == 0U;
}

bool fifo_is_full(void)
{
    return fifo_count() == FIFO_SIZE;
}

void fifo_clear(void)
{
    k_mutex_lock(&s_fifo_mutex, K_FOREVER);
    s_head      = 0U;
    s_tail      = 0U;
    s_count     = 0U;
    s_overflows = 0U;
    k_mutex_unlock(&s_fifo_mutex);
    LOG_INF("FIFO vidé");
}

uint32_t fifo_overflow_count(void)
{
    k_mutex_lock(&s_fifo_mutex, K_FOREVER);
    uint32_t ov = s_overflows;
    k_mutex_unlock(&s_fifo_mutex);
    return ov;
}

void fifo_get_stats(fifo_stats_t *stats)
{
    if (!stats) {
        return;
    }

    k_mutex_lock(&s_fifo_mutex, K_FOREVER);

    stats->count     = s_count;
    stats->capacity  = FIFO_SIZE;
    stats->overflows = s_overflows;

    if (s_count > 0U) {
        stats->oldest_ts = s_buf[s_head].timestamp;
        uint32_t tail_prev = (s_tail == 0U) ? FIFO_SIZE - 1U : s_tail - 1U;
        stats->newest_ts = s_buf[tail_prev].timestamp;
    } else {
        stats->oldest_ts = 0U;
        stats->newest_ts = 0U;
    }

    k_mutex_unlock(&s_fifo_mutex);
}