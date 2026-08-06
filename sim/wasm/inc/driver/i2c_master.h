#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>
typedef int   i2c_port_t;
#define I2C_NUM_0 0
typedef void* i2c_master_bus_handle_t;
typedef void* i2c_master_dev_handle_t;
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t dev, const uint8_t* buf, size_t len, int timeout_ms);
esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus, uint16_t addr, int timeout_ms);
