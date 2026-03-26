/**
 * ESP-Drone — Bosch BMI270 on ESP-IDF I2C (wraps Bosch BMI270_SensorAPI).
 */
#ifndef BMI270_ESP_H
#define BMI270_ESP_H

#include <stdbool.h>
#include <stdint.h>
#include "i2cdev.h"
#include "bmi2.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 7-bit I2C address (0x68 or 0x69). */
bool bmi270_esp_init(I2C_Dev *i2c, uint8_t i2c_addr_7bit, struct bmi2_dev *dev_out);

/** After init + enable: read raw accel/gyro (sensor frame, 16-bit). */
int8_t bmi270_esp_read_raw(struct bmi2_dev *dev, int16_t *ax, int16_t *ay, int16_t *az,
                           int16_t *gx, int16_t *gy, int16_t *gz);

#ifdef __cplusplus
}
#endif
#endif
