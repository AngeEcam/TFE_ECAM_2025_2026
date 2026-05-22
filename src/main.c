/*
 * HMRA Monitor — main.c  v2.1
 *
 * Nouveauté v2.1 : intégration du capteur XYMD04 (SHT40)
 * ─────────────────────────────────────────────────────────────────
 * bus_scan() parcourt les adresses 1→SCAN_SLAVE_MAX et identifie
 * chaque capteur par sa réponse caractéristique :
 *
 *   FC04 (input 0x0000, 4 regs) répond OK → 13 bytes  →  STHP01A   (testé EN PREMIER)
 *   FC04 (input 0x0001, 2 regs) répond OK →  9 bytes  →  XYMD04    (testé EN SECOND)
 *   FC03 (holding 0x0000, 2 regs) répond OK → 9 bytes →  PT100     (testé EN DERNIER)
 *   aucune réponse                                     →  absent
 *
 * Ordre critique :
 *   - STHP01A en 1er : répond aussi au FC03 → serait classé PT100 sinon.
 *   - XYMD04 en 2e   : FC04 sur 0x0001 (signature unique, jamais 0x0000).
 *   - PT100 en 3e    : FC03 uniquement, ne répond pas au FC04.
 *
 * probe() accepte maintenant un reg_start variable pour discriminer
 * XYMD04 (0x0001) de STHP01A (0x0000).
 *
 * Aucune configuration préalable des capteurs n'est nécessaire.
 * Un capteur débranché disparaît au prochain scan.
 * Un capteur rebranché est redécouvert automatiquement.
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
#include "modbus/modbus.h"
#include "storage/fifo_storage.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ─── Scan bus ────────────────────────────────────────────────────── */
#define SCAN_SLAVE_MIN       1
#define SCAN_SLAVE_MAX       5    /* Équivalent range(1,6) du Python  */
#define SCAN_MAX_SLAVES      (SCAN_SLAVE_MAX - SCAN_SLAVE_MIN + 1)
#define SCAN_TIMEOUT_MS      300  /* Timeout réduit pour les absents  */
#define SCAN_INTER_MSG_MS    50   /* Délai entre sondes (half-duplex) */

/* ─── Timing ─────────────────────────────────────────────────────── */
#define SENSOR_POLL_MS       60000
#define BLE_FLUSH_TIMEOUT_MS 5000
#define NUS_MAX_RECORDS      10   /* Augmenté : jusqu'à 5 PT100x2 + STHP */
#define BLE_READY_TIMEOUT_MS 3000

/* ─── Thread capteurs ────────────────────────────────────────────── */
#define SENSOR_THREAD_STACK  4096
#define SENSOR_THREAD_PRIO   5

/* ─── Types de capteurs ──────────────────────────────────────────── */
typedef enum {
    SENSOR_UNKNOWN = 0,
    SENSOR_PT100,
    SENSOR_STHP01A,
    SENSOR_XYMD04,
} sensor_type_t;

/* ─── Table des capteurs découverts ──────────────────────────────── */
typedef struct {
    uint8_t       slave_id;
    sensor_type_t type;
    bool          present;
} bus_entry_t;

static bus_entry_t bus_map[SCAN_MAX_SLAVES];
static uint8_t     bus_count = 0;

/* ─── Sémaphore cycle capteurs → flush BLE ───────────────────────── */
K_SEM_DEFINE(cycle_done_sem, 0, 1);

/* ─── État BLE ───────────────────────────────────────────────────── */
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
 * Callbacks BLE (inchangés)
 * ═══════════════════════════════════════════════════════════════════ */

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
                            struct bt_gatt_exchange_params *params)
{
    if (!err)
        LOG_INF("MTU exchange OK (MTU=%u)", bt_gatt_get_mtu(conn));
    else
        LOG_WRN("MTU exchange err %u", err);
    ble_ready = true;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) { LOG_ERR("Echec connexion BLE (err %u)", err); return; }
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
    if (current_conn) { bt_conn_unref(current_conn); current_conn = NULL; }
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

static struct bt_nus_cb nus_cb = { .received = nus_received };

/* ═══════════════════════════════════════════════════════════════════
 * bus_scan()
 *
 * Ordre de détection pour chaque adresse :
 *
 *   1. FC04 sur 0x0000, count=4 → 13 bytes → STHP01A  (en premier car
 *      STHP01A répond aussi au FC03, ce qui le classerait faussement PT100)
 *   2. FC04 sur 0x0001, count=2 →  9 bytes → XYMD04   (reg_start unique)
 *   3. FC03 sur 0x0000, count=2 →  9 bytes → PT100    (ne répond pas FC04)
 *   4. aucune réponse           →  absent
 *
 * probe() utilise SCAN_TIMEOUT_MS (300 ms) au lieu du timeout normal
 * de 1000 ms — essentiel pour ne pas bloquer sur chaque adresse vide.
 * ═══════════════════════════════════════════════════════════════════ */

static bool probe(const struct device *uart,
                  uint8_t  slave_id,
                  uint8_t  fc,
                  uint16_t reg_start,
                  uint16_t reg_count,
                  uint8_t  expected_len)
{
    uint8_t req[8];
    req[0] = slave_id;
    req[1] = fc;
    req[2] = (reg_start >> 8) & 0xFF;
    req[3] =  reg_start       & 0xFF;
    req[4] = (reg_count >> 8) & 0xFF;
    req[5] =  reg_count       & 0xFF;
    uint16_t crc = modbus_crc16(req, 6);
    req[6] = crc & 0xFF;
    req[7] = crc >> 8;

    uint8_t resp[32] = {0};
    modbus_send(uart, req, sizeof(req));
    int n = modbus_recv(uart, resp, sizeof(resp), SCAN_TIMEOUT_MS);

    /* Longueur correcte + slave_id + function code cohérents */
    return (n == expected_len)
        && (resp[0] == slave_id)
        && (resp[1] == fc);
}

static void bus_scan(const struct device *uart)
{
    memset(bus_map, 0, sizeof(bus_map));
    bus_count = 0;

    LOG_INF("=== Scan bus RS-485 (adresses %u→%u) ===",
            SCAN_SLAVE_MIN, SCAN_SLAVE_MAX);

    for (uint8_t id = SCAN_SLAVE_MIN; id <= SCAN_SLAVE_MAX; id++) {

        uint8_t idx = id - SCAN_SLAVE_MIN;
        bus_map[idx].slave_id = id;
        bus_map[idx].present  = false;
        bus_map[idx].type     = SENSOR_UNKNOWN;

        /* ── 1. Essai STHP01A : FC04, reg 0x0000, 4 regs → 13 bytes ───────
         *   En premier : STHP01A répond aussi au FC03 (9 bytes), ce qui le
         *   classerait faussement PT100. Le tester via FC04/0x0000 d'abord
         *   le discrimine sans ambiguïté. */
        if (probe(uart, id,
                  MODBUS_FC_READ_INPUT_REGISTERS,
                  0x0000,
                  STHP01A_REG_COUNT,   /* 4 */
                  STHP01A_RESP_LEN)) { /* 13 */

            bus_map[idx].present = true;
            bus_map[idx].type    = SENSOR_STHP01A;
            bus_count++;
            LOG_INF("  ID %u → STHP01A detecte", id);

        /* ── 2. Essai XYMD04 : FC04, reg 0x0001, 2 regs → 9 bytes ─────────
         *   reg_start=0x0001 est la signature propre au XYMD04 (SHT40).
         *   Le STHP01A a déjà été écarté ; le PT100 ne répond pas au FC04. */
        } else if (probe(uart, id,
                         MODBUS_FC_READ_INPUT_REGISTERS,
                         0x0001,
                         XYMD04_REG_COUNT,   /* 2 */
                         XYMD04_RESP_LEN)) { /* 9 */

            bus_map[idx].present = true;
            bus_map[idx].type    = SENSOR_XYMD04;
            bus_count++;
            LOG_INF("  ID %u → XYMD04 detecte", id);

        /* ── 3. Essai PT100 : FC03, reg 0x0000, 2 regs → 9 bytes ───────── */
        } else if (probe(uart, id,
                         MODBUS_FC_READ_HOLDING_REGISTERS,
                         0x0000,
                         PT100_REG_COUNT,      /* 2 */
                         PT100_RESP_LEN)) {    /* 9 */

            bus_map[idx].present = true;
            bus_map[idx].type    = SENSOR_PT100;
            bus_count++;
            LOG_INF("  ID %u → PT100 detecte", id);

        } else {
            LOG_DBG("  ID %u → aucun capteur", id);
        }

        k_sleep(K_MSEC(SCAN_INTER_MSG_MS));
    }

    LOG_INF("=== Scan termine : %u capteur(s) ===", bus_count);
}

/* ═══════════════════════════════════════════════════════════════════
 * Thread capteurs
 * ═══════════════════════════════════════════════════════════════════ */

K_THREAD_STACK_DEFINE(sensor_stack, SENSOR_THREAD_STACK);
static struct k_thread sensor_thread_data;

static void sensor_thread(void *p1, void *p2, void *p3)
{
    const struct device *uart = (const struct device *)p1;

    static sthp01a_data_t       s_sthp;
    static xymd04_data_t        s_xymd;
    static pt100_pta8c04_data_t s_pt100;
    static sensor_record_t      r, r1, r2;

    LOG_INF("Thread capteurs demarre (stack=%u bytes)", SENSOR_THREAD_STACK);

    /* Scan initial avant le premier cycle */
    bus_scan(uart);

    while (1) {
        uint32_t ts = (uint32_t)(k_uptime_get() / 1000U);

        /* ── Diagnostic stack ────────────────────────────────────── */
        size_t stack_unused = 0;
        k_thread_stack_space_get(&sensor_thread_data, &stack_unused);
        LOG_DBG("Stack libre: %zu bytes", stack_unused);

        /* ── Re-scan : redécouvre les capteurs branchés/débranchés ─ */
        bus_scan(uart);

        /* ── Acquisition sur chaque capteur détecté ─────────────── */
        for (uint8_t i = 0; i < SCAN_MAX_SLAVES; i++) {

            if (!bus_map[i].present) continue;

            uint8_t id = bus_map[i].slave_id;

            if (bus_map[i].type == SENSOR_PT100) {

                /* ── PT100 : 2 canaux → 2 records ────────────────── */
                memset(&s_pt100, 0, sizeof(s_pt100));
                memset(&r1, 0, sizeof(r1));
                memset(&r2, 0, sizeof(r2));

                if (pt100_pta8c04_read(uart, id, &s_pt100)) {
                    r1 = (sensor_record_t){
                        .timestamp = ts,
                        .temp      = (int16_t)(s_pt100.ch1 * 10.0f),
                        .humidity  = 0,
                        .pressure  = 0,
                        .sensor_id = (uint8_t)((id << 4) | 0x1),
                        .flags     = RECORD_FLAG_VALID,
                    };
                    fifo_push(&r1);
                    r2 = (sensor_record_t){
                        .timestamp = ts,
                        .temp      = (int16_t)(s_pt100.ch2 * 10.0f),
                        .humidity  = 0,
                        .pressure  = 0,
                        .sensor_id = (uint8_t)((id << 4) | 0x2),
                        .flags     = RECORD_FLAG_VALID,
                    };
                    fifo_push(&r2);
                    LOG_INF("PT100[%u]  CH1=%.1f CH2=%.1f",
                            id, (double)s_pt100.ch1, (double)s_pt100.ch2);
                } else {
                    LOG_WRN("PT100[%u]: echec lecture", id);
                }

            } else if (bus_map[i].type == SENSOR_XYMD04) {

                /* ── XYMD04 : T + HR → 1 record ──────────────────── */
                memset(&s_xymd, 0, sizeof(s_xymd));
                memset(&r, 0, sizeof(r));

                if (xymd04_read(uart, id, &s_xymd)) {
                    r = (sensor_record_t){
                        .timestamp = ts,
                        .temp      = (int16_t)(s_xymd.temperature * 10.0f),
                        .humidity  = (uint16_t)(s_xymd.humidity   * 10.0f),
                        .pressure  = 0,
                        .sensor_id = id,
                        .flags     = RECORD_FLAG_VALID,
                    };
                    fifo_push(&r);
                    LOG_INF("XYMD04[%u]  T=%.1f H=%.1f",
                            id,
                            (double)s_xymd.temperature,
                            (double)s_xymd.humidity);
                } else {
                    LOG_WRN("XYMD04[%u]: echec lecture", id);
                }

            } else if (bus_map[i].type == SENSOR_STHP01A) {

                /* ── STHP01A : T + HR + P → 1 record ─────────────── */
                memset(&s_sthp, 0, sizeof(s_sthp));
                memset(&r, 0, sizeof(r));

                if (sthp01a_read(uart, id, &s_sthp)) {
                    r = (sensor_record_t){
                        .timestamp = ts,
                        .temp      = (int16_t)(s_sthp.temperature * 10.0f),
                        .humidity  = (uint16_t)(s_sthp.humidity   * 10.0f),
                        .pressure  = (uint16_t)(s_sthp.pressure   * 10.0f),
                        .sensor_id = id,
                        .flags     = RECORD_FLAG_VALID,
                    };
                    fifo_push(&r);
                    LOG_INF("STHP01A[%u] T=%.2f H=%.2f P=%.1f",
                            id,
                            (double)s_sthp.temperature,
                            (double)s_sthp.humidity,
                            (double)s_sthp.pressure);
                } else {
                    LOG_WRN("STHP01A[%u]: echec lecture", id);
                }
            }

            k_sleep(K_MSEC(500));
        }

        /* ── Cycle complet → signal flush BLE ───────────────────── */
        k_sem_give(&cycle_done_sem);

        /* ── Attente avant prochain cycle ────────────────────────── */
        k_sleep(K_MSEC(SENSOR_POLL_MS));
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * Flush FIFO → NUS (JSON groupé)
 *
 * Format : {"ts":2135,"d":[{"sid":1,"t":225,"h":411,"p":9948},
 *                          {"sid":2,"t":221,"h":677,"p":0},
 *                          {"sid":49,"t":185,"h":0,"p":0},
 *                          {"sid":50,"t":201,"h":0,"p":0}]}
 *
 * Décodage sensor_id côté ESP32 :
 *   STHP01A   → sid = slave_id             (ex: 1)
 *   XYMD04    → sid = slave_id             (ex: 2)
 *   PT100 CH1 → sid = (slave_id << 4) | 1  (ex: slave 3 → 49)
 *   PT100 CH2 → sid = (slave_id << 4) | 2  (ex: slave 3 → 50)
 * ═══════════════════════════════════════════════════════════════════ */

static void flush_fifo_over_nus(void)
{
    if (!current_conn || !ble_ready) return;

    static sensor_record_t records[NUS_MAX_RECORDS];
    static char            json[247];
    int count = 0;

    while (count < NUS_MAX_RECORDS && fifo_pop(&records[count]))
        count++;

    if (count == 0) return;

    int pos = 0;
    pos += snprintf(json + pos, sizeof(json) - pos,
                    "{\"ts\":%u,\"d\":[", records[0].timestamp);

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

    int err = bt_nus_send(current_conn, (const uint8_t *)json, (uint16_t)pos);
    if (err) {
        LOG_WRN("NUS TX err %d — requeue %d records", err, count);
        for (int i = 0; i < count; i++) fifo_push(&records[i]);
    } else {
        LOG_INF("NUS TX OK — %d records : %s", count, json);
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * main (inchangé)
 * ═══════════════════════════════════════════════════════════════════ */

int main(void)
{
    LOG_INF("HMRA Monitor v2.1 — demarrage");

    const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
    if (!device_is_ready(uart)) {
        LOG_ERR("uart0 non pret — arret");
        return -1;
    }
    LOG_INF("uart0 pret");

    fifo_storage_init();
    LOG_INF("FIFO initialisee: %u slots", FIFO_SIZE);

    int err = bt_enable(NULL);
    if (err) { LOG_ERR("bt_enable failed (err %d)", err); return -1; }

    err = bt_nus_init(&nus_cb);
    if (err) { LOG_ERR("bt_nus_init failed (err %d)", err); return -1; }

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) { LOG_ERR("bt_le_adv_start failed (err %d)", err); return -1; }
    LOG_INF("BLE advertising: \"%s\"", CONFIG_BT_DEVICE_NAME);

    k_thread_create(&sensor_thread_data,
                    sensor_stack,
                    K_THREAD_STACK_SIZEOF(sensor_stack),
                    sensor_thread,
                    (void *)uart, NULL, NULL,
                    SENSOR_THREAD_PRIO, 0, K_SECONDS(2));
    k_thread_name_set(&sensor_thread_data, "sensors");
    LOG_INF("Thread capteurs cree (stack=%u bytes)", SENSOR_THREAD_STACK);

    while (1) {
        if (k_sem_take(&cycle_done_sem, K_MSEC(BLE_FLUSH_TIMEOUT_MS)) == 0)
            flush_fifo_over_nus();
    }

    return 0;
}