/*
 * HMRA Monitor — main.c  v1.3
 * Corrections appliquées :
 *  1. Stack du thread capteurs augmenté à 4096 bytes
 *  2. Structs capteurs déclarées static → hors du stack
 *  3. Gestion d'erreur bt_nus_send avec back-pressure propre
 *  4. json déclaré static dans flush_fifo_over_nus
 *  5. Timeout de connexion BLE watchdog
 *  6. Commentaires clairs sur chaque section
 *  7. memset sur r, r1, r2 avant chaque utilisation
 *  8. Sémaphore cycle_done_sem : flush déclenché uniquement après
 *     lecture complète de tous les capteurs du cycle
 *  9. JSON groupé : tous les records du cycle envoyés en UN SEUL
 *     paquet BLE → élimine le drop de notifications ESP32
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/services/nus.h>
#include <string.h>
#include <stdio.h>

#include "sensors/sthp01a.h"
#include "sensors/xymd04.h"
#include "sensors/pt100_pta8c04.h"
#include "storage/fifo_storage.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ─── IDs esclaves Modbus ─────────────────────────────────────────── */
#define SLAVE_ID_STHP01A     1
#define SLAVE_ID_XYMD04      2
#define SLAVE_ID_PT100       3
#define SLAVE_ID_XYMD04_2    4

/* ─── Timing ─────────────────────────────────────────────────────── */
#define SENSOR_POLL_MS       60000   /* Intervalle lecture capteurs    */
#define BLE_FLUSH_TIMEOUT_MS 5000    /* Timeout attente sémaphore      */
#define NUS_MAX_RECORDS      5       /* Max records par cycle flush    */
#define BLE_READY_TIMEOUT_MS 3000    /* Timeout attente MTU exchange   */

/* ─── Thread capteurs ────────────────────────────────────────────── */
#define SENSOR_THREAD_STACK  4096
#define SENSOR_THREAD_PRIO   5

/* ─── Sémaphore synchronisation cycle capteurs → flush BLE ──────── */
/*
 * CORRECTION #8 : le flush BLE attend ce sémaphore avant d'envoyer.
 * Le sensor_thread le donne UNE SEULE FOIS après avoir lu et pushé
 * TOUS les capteurs du cycle. Ainsi, quand flush_fifo_over_nus()
 * est appelé, le FIFO contient déjà tous les records du cycle.
 */
K_SEM_DEFINE(cycle_done_sem, 0, 1);

/* ─── État BLE global ────────────────────────────────────────────── */
static struct bt_conn               *current_conn = NULL;
static bool                          ble_ready    = false;
static struct bt_gatt_exchange_params mtu_params;

/* ─── Advertising data ───────────────────────────────────────────── */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS,
                  BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_NAME_COMPLETE,
            CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* ═══════════════════════════════════════════════════════════════════
 * Callbacks BLE
 * ═══════════════════════════════════════════════════════════════════ */

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                            struct bt_gatt_exchange_params *params)
{
    if (!err) {
        LOG_INF("MTU exchange OK (MTU=%u)", bt_gatt_get_mtu(conn));
    } else {
        LOG_WRN("MTU exchange err %u", err);
    }
    ble_ready = true;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Echec connexion BLE (err %u)", err);
        return;
    }

    current_conn    = bt_conn_ref(conn);
    ble_ready       = false;
    mtu_params.func = mtu_exchange_cb;

    LOG_INF("BLE connecte — MTU exchange...");

    int ret = bt_gatt_exchange_mtu(conn, &mtu_params);
    if (ret) {
        LOG_WRN("bt_gatt_exchange_mtu err %d — activation differee", ret);
        k_sleep(K_MSEC(BLE_READY_TIMEOUT_MS));
        ble_ready = true;
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("BLE deconnecte (reason %u)", reason);
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    ble_ready = false;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
};

static void nus_received(struct bt_conn *conn,
                         const uint8_t *data, uint16_t len)
{
    LOG_INF("NUS RX: %.*s", len, data);
}

static struct bt_nus_cb nus_cb = {
    .received = nus_received,
};

/* ═══════════════════════════════════════════════════════════════════
 * Thread capteurs
 * ═══════════════════════════════════════════════════════════════════ */

K_THREAD_STACK_DEFINE(sensor_stack, SENSOR_THREAD_STACK);
static struct k_thread sensor_thread_data;

static void sensor_thread(void *p1, void *p2, void *p3)
{
    const struct device *uart = (const struct device *)p1;

    static sthp01a_data_t       s1;
    static xymd04_data_t        s2;
    static xymd04_data_t        s4;
    static pt100_pta8c04_data_t s3;
    static sensor_record_t      r;
    static sensor_record_t      r1;
    static sensor_record_t      r2;

    LOG_INF("Thread capteurs demarre (stack=%u bytes)",
            SENSOR_THREAD_STACK);

    while (1) {
        uint32_t ts = (uint32_t)(k_uptime_get() / 1000U);

        /* ── Diagnostic stack ────────────────────────────────────── */
        size_t stack_unused = 0;
        k_thread_stack_space_get(&sensor_thread_data, &stack_unused);
        LOG_DBG("Stack libre: %zu bytes", stack_unused);

        /* ── STHP01A ─────────────────────────────────────────────── */
        memset(&s1, 0, sizeof(s1));
        memset(&r,  0, sizeof(r));
        if (sthp01a_read(uart, SLAVE_ID_STHP01A, &s1)) {
            r = (sensor_record_t){
                .timestamp = ts,
                .temp      = (int16_t)(s1.temperature * 10.0f),
                .humidity  = (uint16_t)(s1.humidity   * 10.0f),
                .pressure  = (uint16_t)(s1.pressure   * 10.0f),
                .sensor_id = SENSOR_ID_STHP01A,
                .flags     = RECORD_FLAG_VALID,
            };
            fifo_push(&r);
            LOG_INF("STHP01A  T=%.2f H=%.2f P=%.1f",
                    (double)s1.temperature,
                    (double)s1.humidity,
                    (double)s1.pressure);
        } else {
            LOG_WRN("STHP01A: echec lecture");
        }

        k_sleep(K_MSEC(500));

        /* ── XY-MD04 #1 ──────────────────────────────────────────── */
        memset(&s2, 0, sizeof(s2));
        memset(&r,  0, sizeof(r));
        if (xymd04_read(uart, SLAVE_ID_XYMD04, &s2)) {
            r = (sensor_record_t){
                .timestamp = ts,
                .temp      = (int16_t)(s2.temperature * 10.0f),
                .humidity  = (uint16_t)(s2.humidity   * 10.0f),
                .pressure  = 0,
                .sensor_id = SENSOR_ID_SHT20,
                .flags     = RECORD_FLAG_VALID,
            };
            fifo_push(&r);
            LOG_INF("XYMD04#1 T=%.1f H=%.1f",
                    (double)s2.temperature,
                    (double)s2.humidity);
        } else {
            LOG_WRN("XYMD04#1: echec lecture");
        }

        k_sleep(K_MSEC(500));

        /* ── XY-MD04 #2 ──────────────────────────────────────────── */
        memset(&s4, 0, sizeof(s4));
        memset(&r,  0, sizeof(r));
        if (xymd04_read(uart, SLAVE_ID_XYMD04_2, &s4)) {
            r = (sensor_record_t){
                .timestamp = ts,
                .temp      = (int16_t)(s4.temperature * 10.0f),
                .humidity  = (uint16_t)(s4.humidity   * 10.0f),
                .pressure  = 0,
                .sensor_id = 5,
                .flags     = RECORD_FLAG_VALID,
            };
            fifo_push(&r);
            LOG_INF("XYMD04#2 T=%.1f H=%.1f",
                    (double)s4.temperature,
                    (double)s4.humidity);
        } else {
            LOG_WRN("XYMD04#2: echec lecture");
        }

        k_sleep(K_MSEC(500));

        /* ── PT100 (2 canaux) ────────────────────────────────────── */
        memset(&s3, 0, sizeof(s3));
        memset(&r1, 0, sizeof(r1));
        memset(&r2, 0, sizeof(r2));
        if (pt100_pta8c04_read(uart, SLAVE_ID_PT100, &s3)) {
            r1 = (sensor_record_t){
                .timestamp = ts,
                .temp      = (int16_t)(s3.ch1 * 10.0f),
                .humidity  = 0,
                .pressure  = 0,
                .sensor_id = SENSOR_ID_PT100_CH1,
                .flags     = RECORD_FLAG_VALID,
            };
            fifo_push(&r1);

            r2 = (sensor_record_t){
                .timestamp = ts,
                .temp      = (int16_t)(s3.ch2 * 10.0f),
                .humidity  = 0,
                .pressure  = 0,
                .sensor_id = SENSOR_ID_PT100_CH2,
                .flags     = RECORD_FLAG_VALID,
            };
            fifo_push(&r2);

            LOG_INF("PT100    CH1=%.1f CH2=%.1f",
                    (double)s3.ch1,
                    (double)s3.ch2);
        } else {
            LOG_WRN("PT100: echec lecture");
        }

        /* ── Cycle complet — signal au flush BLE ─────────────────── */
        k_sem_give(&cycle_done_sem);

        /* ── Attente avant prochain cycle ────────────────────────── */
        k_sleep(K_MSEC(SENSOR_POLL_MS));
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * Flush FIFO → NUS (JSON groupé)
 * ═══════════════════════════════════════════════════════════════════
 *
 * CORRECTION #9 : au lieu d'envoyer N notifications BLE séparées
 * (une par capteur), on groupe tous les records du cycle dans un
 * seul paquet JSON et on fait UN SEUL bt_nus_send.
 *
 * Format : {"ts":2135,"d":[{"sid":1,"t":225,"h":411,"p":9948},
 *                          {"sid":2,"t":263,"h":338,"p":0},
 *                          {"sid":5,"t":246,"h":349,"p":0}]}
 *
 * Avantage : zéro risque de drop de notification BLE côté ESP32,
 * peu importe la vitesse du stack Espressif.
 * Le MTU est 247 bytes → largement suffisant pour 5 capteurs.
 * ═══════════════════════════════════════════════════════════════════ */

static void flush_fifo_over_nus(void)
{
    if (!current_conn || !ble_ready) {
        return;
    }

    static sensor_record_t records[NUS_MAX_RECORDS];
    static char            json[247];   /* MTU max */
    int count = 0;

    /* ── Collecter tous les records disponibles dans le FIFO ─────── */
    while (count < NUS_MAX_RECORDS && fifo_pop(&records[count])) {
        count++;
    }

    if (count == 0) {
        return;
    }

    /* ── Construire le JSON groupé ───────────────────────────────── */
    int pos = 0;
    pos += snprintf(json + pos, sizeof(json) - pos,
                    "{\"ts\":%u,\"d\":[",
                    records[0].timestamp);

    for (int i = 0; i < count; i++) {
        pos += snprintf(json + pos, sizeof(json) - pos,
                        "%s{\"sid\":%u,\"t\":%d,\"h\":%u,\"p\":%u}",
                        i > 0 ? "," : "",
                        (unsigned)records[i].sensor_id,
                        (int)records[i].temp,
                        (unsigned)records[i].humidity,
                        (unsigned)records[i].pressure);

        if (pos >= (int)sizeof(json) - 10) {
            LOG_ERR("JSON overflow a l'element %d", i);
            break;
        }
    }

    pos += snprintf(json + pos, sizeof(json) - pos, "]}\n");

    /* ── Envoyer en un seul paquet BLE ───────────────────────────── */
    int err = bt_nus_send(current_conn,
                          (const uint8_t *)json,
                          (uint16_t)pos);
    if (err) {
        LOG_WRN("NUS TX err %d — requeue %d records", err, count);
        for (int i = 0; i < count; i++) {
            fifo_push(&records[i]);
        }
    } else {
        LOG_INF("NUS TX OK — %d records ts=%u : %s",
                count, records[0].timestamp, json);
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * main
 * ═══════════════════════════════════════════════════════════════════ */

int main(void)
{
    LOG_INF("HMRA Monitor v1.3 — demarrage");

    /* ── UART ────────────────────────────────────────────────────── */
    const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
    if (!device_is_ready(uart)) {
        LOG_ERR("uart0 non pret — arret");
        return -1;
    }
    LOG_INF("uart0 pret");

    /* ── FIFO storage ────────────────────────────────────────────── */
    fifo_storage_init();
    LOG_INF("FIFO initialisee: %u slots", FIFO_SIZE);

    /* ── Bluetooth ───────────────────────────────────────────────── */
    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("bt_enable failed (err %d)", err);
        return -1;
    }

    err = bt_nus_init(&nus_cb);
    if (err) {
        LOG_ERR("bt_nus_init failed (err %d)", err);
        return -1;
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("bt_le_adv_start failed (err %d)", err);
        return -1;
    }
    LOG_INF("BLE advertising: \"%s\"", CONFIG_BT_DEVICE_NAME);

    /* ── Thread capteurs ─────────────────────────────────────────── */
    k_thread_create(&sensor_thread_data,
                    sensor_stack,
                    K_THREAD_STACK_SIZEOF(sensor_stack),
                    sensor_thread,
                    (void *)uart, NULL, NULL,
                    SENSOR_THREAD_PRIO,
                    0,
                    K_SECONDS(2));
    k_thread_name_set(&sensor_thread_data, "sensors");

    LOG_INF("Thread capteurs cree (stack=%u bytes)", SENSOR_THREAD_STACK);

    /* ── Boucle principale : flush FIFO → BLE ────────────────────── */
    while (1) {
        if (k_sem_take(&cycle_done_sem,
                       K_MSEC(BLE_FLUSH_TIMEOUT_MS)) == 0) {
            flush_fifo_over_nus();
        }
    }

    return 0;
}