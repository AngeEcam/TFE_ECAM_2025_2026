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
#include "sensors/hailege_sht20.h"
#include "sensors/pt100_pta8c04.h"
#include "storage/fifo_storage.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ═══════════════════════════════════════════════════
   CONFIGURATION
   ═══════════════════════════════════════════════════ */

#define SLAVE_ID_STHP01A     1
#define SLAVE_ID_HAILEGE     2
#define SLAVE_ID_PT100       3

#define SENSOR_POLL_MS       60000  /* intervalle lecture capteurs          */
#define BLE_FLUSH_MS         1000   /* intervalle flush FIFO → NUS          */
#define NUS_INTER_MSG_MS     50     /* délai entre deux notifications NUS   */
#define NUS_MAX_RECORDS      5      /* records max envoyés par flush        */

#define SENSOR_THREAD_STACK  2048
#define SENSOR_THREAD_PRIO   5

/* ═══════════════════════════════════════════════════
   ÉTAT BLE
   ═══════════════════════════════════════════════════ */

static struct bt_conn              *current_conn = NULL;
static bool                         ble_ready    = false;
static struct bt_gatt_exchange_params mtu_params;

/* ── Advertising data ── */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS,
                  BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_NAME_COMPLETE,
            CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* ── MTU exchange callback ──
 * Appelé quand le channel ATT est réellement prêt.
 * C'est seulement à ce moment qu'on autorise l'envoi NUS.
 */
static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                            struct bt_gatt_exchange_params *params)
{
    if (!err) {
        LOG_INF("MTU exchange OK (MTU=%u) — envoi NUS active",
                bt_gatt_get_mtu(conn));
    } else {
        LOG_WRN("MTU exchange err %u — on tente quand meme", err);
    }
    ble_ready = true;
}

/* ── Callbacks connexion ── */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) return;

    current_conn     = bt_conn_ref(conn);
    ble_ready        = false;
    mtu_params.func  = mtu_exchange_cb;

    LOG_INF("BLE connecte — MTU exchange...");

    int ret = bt_gatt_exchange_mtu(conn, &mtu_params);
    if (ret) {
        /* Fallback si le MTU exchange échoue immédiatement */
        LOG_WRN("MTU exchange request err %d — activation differee", ret);
        k_sleep(K_MSEC(500));
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

/* ── NUS callbacks ── */
static void nus_received(struct bt_conn *conn,
                         const uint8_t *data, uint16_t len)
{
    LOG_INF("NUS RX: %.*s", len, data);
}

static struct bt_nus_cb nus_cb = {
    .received = nus_received,
};

/* ═══════════════════════════════════════════════════
   THREAD CAPTEURS
   Séparé du main thread pour ne jamais bloquer le BLE.
   Le polling Modbus (timeout 1s/capteur) tourne ici.
   ═══════════════════════════════════════════════════ */

K_THREAD_STACK_DEFINE(sensor_stack, SENSOR_THREAD_STACK);
static struct k_thread sensor_thread_data;

static void sensor_thread(void *p1, void *p2, void *p3)
{
    const struct device *uart = (const struct device *)p1;

    LOG_INF("Thread capteurs demarre");

    while (1) {
        uint32_t ts = (uint32_t)(k_uptime_get() / 1000U);

        /* ── STHP01A — temp / hum / pression ── */
        sthp01a_data_t s1;
        if (sthp01a_read(uart, SLAVE_ID_STHP01A, &s1)) {
            sensor_record_t r = {
                .timestamp = ts,
                .temp      = (int16_t)(s1.temperature * 10.0f),
                .humidity  = (uint16_t)(s1.humidity   * 10.0f),
                .pressure  = (uint16_t)(s1.pressure   * 10.0f),
                .sensor_id = SENSOR_ID_STHP01A,
                .flags     = RECORD_FLAG_VALID,
            };
            fifo_push(&r);
            LOG_INF("STHP01A T=%.2f H=%.2f P=%.1f",
                    (double)s1.temperature,
                    (double)s1.humidity,
                    (double)s1.pressure);
        } else {
            LOG_WRN("STHP01A: echec lecture");
        }

        k_sleep(K_MSEC(500));

        /* ── Hailege SHT20 — temp / hum ── */
        hailege_sht20_data_t s2;
        if (hailege_sht20_read(uart, SLAVE_ID_HAILEGE, &s2)) {
            sensor_record_t r = {
                .timestamp = ts,
                .temp      = (int16_t)(s2.temperature * 10.0f),
                .humidity  = (uint16_t)(s2.humidity   * 10.0f),
                .pressure  = 0,
                .sensor_id = SENSOR_ID_SHT20,
                .flags     = RECORD_FLAG_VALID,
            };
            fifo_push(&r);
            LOG_INF("SHT20   T=%.1f H=%.1f",
                    (double)s2.temperature,
                    (double)s2.humidity);
        } else {
            LOG_WRN("SHT20: echec lecture");
        }

        k_sleep(K_MSEC(500));

        /* ── PT100 PTA8C04 — 2 canaux ── */
        pt100_pta8c04_data_t s3;
        if (pt100_pta8c04_read(uart, SLAVE_ID_PT100, &s3)) {
            sensor_record_t r1 = {
                .timestamp = ts,
                .temp      = (int16_t)(s3.ch1 * 10.0f),
                .humidity  = 0,
                .pressure  = 0,
                .sensor_id = SENSOR_ID_PT100_CH1,
                .flags     = RECORD_FLAG_VALID,
            };
            fifo_push(&r1);

            sensor_record_t r2 = {
                .timestamp = ts,
                .temp      = (int16_t)(s3.ch2 * 10.0f),
                .humidity  = 0,
                .pressure  = 0,
                .sensor_id = SENSOR_ID_PT100_CH2,
                .flags     = RECORD_FLAG_VALID,
            };
            fifo_push(&r2);

            LOG_INF("PT100   CH1=%.1f CH2=%.1f",
                    (double)s3.ch1,
                    (double)s3.ch2);
        } else {
            LOG_WRN("PT100: echec lecture");
        }

        /* Attente jusqu'au prochain cycle */
        k_sleep(K_MSEC(SENSOR_POLL_MS));
    }
}

/* ═══════════════════════════════════════════════════
   FLUSH FIFO → NUS
   Appelé depuis le main thread toutes les BLE_FLUSH_MS.
   N'envoie que si ble_ready (MTU exchange terminé).
   ═══════════════════════════════════════════════════ */

static void flush_fifo_over_nus(void)
{
    if (!current_conn || !ble_ready) {
        return;
    }

    sensor_record_t r;
    int sent = 0;

    while (sent < NUS_MAX_RECORDS && fifo_pop(&r)) {
        char json[128];
        snprintf(json, sizeof(json),
                 "{\"ts\":%u,\"sid\":%u,"
                 "\"t\":%d,\"h\":%u,\"p\":%u,\"f\":%u}\n",
                 r.timestamp,
                 (unsigned)r.sensor_id,
                 (int)r.temp,
                 (unsigned)r.humidity,
                 (unsigned)r.pressure,
                 (unsigned)r.flags);

        int err = bt_nus_send(current_conn,
                              (const uint8_t *)json,
                              strlen(json));
        if (err) {
            /* Remettre dans le FIFO et arrêter ce cycle */
            fifo_push(&r);
            LOG_WRN("NUS TX err %d — flush stoppe (%d en attente)",
                    err, fifo_count());
            break;
        }

        sent++;
        k_sleep(K_MSEC(NUS_INTER_MSG_MS));
    }

    if (sent > 0) {
        LOG_INF("NUS TX: %d records envoyes, %d en attente",
                sent, fifo_count());
    }
}

/* ═══════════════════════════════════════════════════
   MAIN
   ═══════════════════════════════════════════════════ */

int main(void)
{
    LOG_INF("HMRA Monitor v1.0 — demarrage");

    /* UART Modbus */
    const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
    if (!device_is_ready(uart)) {
        LOG_ERR("uart0 non pret");
        return -1;
    }
    LOG_INF("uart0 pret");

    /* FIFO */
    fifo_storage_init();
    LOG_INF("FIFO capacite: %u slots", FIFO_SIZE);

    /* BLE */
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
    LOG_INF("BLE advertising: %s", CONFIG_BT_DEVICE_NAME);

    /* Thread capteurs — démarre 2s après pour laisser le BLE s'initialiser */
    k_thread_create(&sensor_thread_data,
                    sensor_stack,
                    K_THREAD_STACK_SIZEOF(sensor_stack),
                    sensor_thread,
                    (void *)uart, NULL, NULL,
                    SENSOR_THREAD_PRIO,
                    0,
                    K_SECONDS(2));
    k_thread_name_set(&sensor_thread_data, "sensors");

    /* Boucle principale : flush FIFO → NUS */
    while (1) {
        flush_fifo_over_nus();
        k_sleep(K_MSEC(BLE_FLUSH_MS));
    }
}