/**
 * ESP-Drone — BMP581 via Bosch BMP5 API on ESP-IDF I2C.
 */
#ifndef BMP581_ESP_H
#define BMP581_ESP_H

#include <stdbool.h>
#include <stdint.h>
#include "i2cdev.h"
#include "bmp5.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BMP581_I2C_ADDR_DEFAULT  UINT8_C(0x46)

bool bmp581_esp_init(I2C_Dev *i2c, uint8_t i2c_addr_7bit);

/** Pressure [Pa], temperature [°C]. Returns false if read failed. */
bool bmp581_esp_read(float *pressure_pa, float *temp_c);

/** Hypsometric ASL [m], same convention as MS5611 helper (pressure [mbar]). */
float bmp581_pressure_pa_to_asl_m(float pressure_pa);

#ifdef __cplusplus
}
#endif
#endif
