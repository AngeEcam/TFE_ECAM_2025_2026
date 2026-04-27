#pragma once
#include <stdint.h>
#include <stdbool.h>

bool modbus_read_register(uint8_t addr, uint16_t reg, int16_t *value);
