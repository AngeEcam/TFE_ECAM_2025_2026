# hmra-nrf52-firmware

Firmware d'**acquisition** du système de monitoring environnemental HMRA, pour **nRF52832** (carte nRF52 DK / PCA10040) sous **Zephyr**.

Brique « acquisition » du projet **[TFE_2025_2026](https://github.com/AngeEcam/TFE_2025_2026)** (architecture d'ensemble et câblage dans ce dépôt).

---

## Rôle

Le nRF52 est le **maître Modbus RTU** du bus RS-485. À chaque cycle, il interroge les capteurs, stocke les mesures dans un **tampon FIFO** anti-coupure, puis les transmet à la passerelle ESP32 en **BLE (Nordic UART Service)**.

```
Capteurs ──Modbus RTU (RS-485)──▶ nRF52832 (FIFO) ──BLE/NUS──▶ ESP32
```

Points clés :
- **Découverte automatique** des capteurs : un scan du bus identifie chaque capteur par sa signature de réponse Modbus — aucune configuration manuelle côté firmware.
- **Tampon FIFO circulaire** : 1440 mesures × 16 octets ≈ 23 Ko RAM → ≈ **4 h 48** de couverture réseau sans perte.
- **Trame BLE groupée** : toutes les mesures d'un cycle dans un seul paquet, pour éviter les pertes de notifications.

---

## Contenu du dépôt

| Élément | Rôle |
|---|---|
| `src/` | code source (acquisition, scan bus, FIFO, BLE) |
| `boards/` | overlays de carte (devicetree) |
| `prj.conf` | configuration Zephyr (BLE, NUS, UART…) |
| `CMakeLists.txt` | build Zephyr |

> Le `prj.conf` doit notamment activer Bluetooth, le **Nordic UART Service** (`CONFIG_BT_NUS`), l'UART du bus RS-485, les logs, et définir `CONFIG_BT_DEVICE_NAME="HMRA_Monitor"`.

---

## Détection des capteurs (ordre critique)

| Ordre | Capteur | Signature Modbus |
|---|---|---|
| 1 | **S-THP-01A** | FC04, registre `0x0000` (testé **en premier** : répond aussi au FC03) |
| 2 | **XY-MD04** | FC04, registre `0x0001` (signature unique) |
| 3 | **PT100 / PTA8C04** | FC03, registre `0x0000`, 2 canaux |

Les capteurs doivent d'abord recevoir une **adresse Slave ID unique** (1–247) — voir le dépôt **[modbus-rs485-sensor-utils](https://github.com/AngeEcam/modbus-rs485-sensor-utils)**. Adressage du projet : S-THP-01A → 1, XY-MD04 → 2, module PT100 → 3.

---

## Compilation & flashage

Environnement nRF Connect SDK / Zephyr (`west`) installé et activé :

```bash
west build -b nrf52dk_nrf52832 .
west flash
```

Suivre les logs sur la sortie série : `BLE advertising: "HMRA_Monitor"`, puis `Thread capteurs cree`, puis `NUS TX OK` à chaque cycle.

---

## Paramètres ajustables

| Fichier | Constante | Effet |
|---|---|---|
| `src/main.c` | `SENSOR_POLL_MS` | intervalle d'acquisition (défaut 60 000 ms) |
| `src/main.c` | `SCAN_SLAVE_MAX` | nombre d'adresses Modbus scannées |
| `src/storage/fifo_storage.h` | `FIFO_SIZE` | profondeur du tampon (défaut 1440) |

> RAM du nRF52832 : 64 Ko. À 16 octets/record, 1440 cases ≈ 23 Ko. Viser 24 h (≈ 8640 cases ≈ 135 Ko) dépasse la RAM → nécessiterait un buffer en flash non-volatile. **Vérifier l'usage RAM réel avant d'augmenter `FIFO_SIZE`.**

---

## Trame BLE émise

```json
{"ts":2135,"d":[{"sid":1,"t":225,"h":411,"p":9948},{"sid":2,"t":263,"h":338,"p":0}]}
```

`t` / `h` / `p` sont en dixièmes d'unité (×10) ; `0` signifie « non mesuré ». Détails dans [docs/ARCHITECTURE.md du hub](https://github.com/AngeEcam/TFE_2025_2026/blob/master/docs/ARCHITECTURE.md).
