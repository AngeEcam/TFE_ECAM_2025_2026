#ifndef MODBUS_H
#define MODBUS_H

#include <zephyr/device.h>
#include <stdint.h>
#include <stddef.h>

/* Fonction codes */
#define MODBUS_FC_READ_INPUT_REGISTERS  0x04

/* Calcul CRC16 Modbus */
uint16_t modbus_crc16(const uint8_t *buf, uint16_t len);

/* Envoi d'une trame sur l'UART */
void modbus_send(const struct device *uart,
                 const uint8_t *buf, size_t len);

/* Réception avec timeout (ms), retourne le nombre de bytes reçus */
int modbus_recv(const struct device *uart,
                uint8_t *buf, size_t max_len,
                uint32_t timeout_ms);

/* Forge et envoie une requête FC04, retourne le nb de bytes reçus */
int modbus_read_input_registers(const struct device *uart,
                                uint8_t slave_id,
                                uint16_t reg_start,
                                uint16_t reg_count,
                                uint8_t *resp,
                                size_t resp_len);

#endif /* MODBUS_H */