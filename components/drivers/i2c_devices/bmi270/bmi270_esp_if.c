/**
 * I2C glue for Bosch BMI270_SensorAPI (BSD-3-Clause).
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

#include "i2cdev.h"
#include "bmi270.h"
#include "bmi270_esp.h"

typedef struct {
    I2C_Dev *i2c;
    uint8_t addr_7bit;
} bmi270_esp_ctx_t;

static void bmi270_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    if (period >= 1000) {
        vTaskDelay(pdMS_TO_TICKS((period + 999) / 1000));
    } else {
        esp_rom_delay_us(period);
    }
}

static BMI2_INTF_RETURN_TYPE bmi270_esp_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    bmi270_esp_ctx_t *ctx = (bmi270_esp_ctx_t *)intf_ptr;
    if (!ctx || !ctx->i2c || !reg_data) {
        return BMI2_E_NULL_PTR;
    }
    bool ok = i2cdevReadReg8(ctx->i2c, ctx->addr_7bit, reg_addr, (uint16_t)len, reg_data);
    return ok ? BMI2_INTF_RET_SUCCESS : BMI2_E_COM_FAIL;
}

static BMI2_INTF_RETURN_TYPE bmi270_esp_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    bmi270_esp_ctx_t *ctx = (bmi270_esp_ctx_t *)intf_ptr;
    if (!ctx || !ctx->i2c || !reg_data) {
        return BMI2_E_NULL_PTR;
    }
    bool ok = i2cdevWriteReg8(ctx->i2c, ctx->addr_7bit, reg_addr, (uint16_t)len, (uint8_t *)reg_data);
    return ok ? BMI2_INTF_RET_SUCCESS : BMI2_E_COM_FAIL;
}

bool bmi270_esp_init(I2C_Dev *i2c, uint8_t i2c_addr_7bit, struct bmi2_dev *dev_out)
{
    static bmi270_esp_ctx_t ctx;
    struct bmi2_int_pin_config pin_config = { 0 };
    uint8_t sens_list[] = { BMI2_ACCEL, BMI2_GYRO };
    struct bmi2_sens_config cfg[2];

    if (!i2c || !dev_out) {
        return false;
    }

    memset(dev_out, 0, sizeof(*dev_out));
    ctx.i2c = i2c;
    ctx.addr_7bit = i2c_addr_7bit;
    dev_out->intf = BMI2_I2C_INTF;
    dev_out->intf_ptr = &ctx;
    dev_out->read = bmi270_esp_read;
    dev_out->write = bmi270_esp_write;
    dev_out->delay_us = bmi270_delay_us;
    dev_out->read_write_len = 46;
    dev_out->config_file_ptr = NULL;

    if (bmi270_init(dev_out) != BMI2_OK) {
        return false;
    }

    if (bmi2_get_int_pin_config(&pin_config, dev_out) != BMI2_OK) {
        return false;
    }

    cfg[0].type = BMI2_ACCEL;
    cfg[1].type = BMI2_GYRO;
    if (bmi2_get_sensor_config(cfg, 2, dev_out) != BMI2_OK) {
        return false;
    }

    if (bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, dev_out) != BMI2_OK) {
        return false;
    }

    /* Accel: 800 Hz ODR, 16g, OSR2 averaging. */
    cfg[0].cfg.acc.odr = BMI2_ACC_ODR_800HZ;
    cfg[0].cfg.acc.range = BMI2_ACC_RANGE_16G;
    cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
    cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    /* Gyro: 800 Hz, 2000 dps, OSR4 (narrower HW BW than NORMAL). */
    cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_800HZ;
    cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR4_MODE;
    cfg[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
    cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

    /* Push-pull DRDY on INT1, active high → ESP32 POSEDGE (matches MPU6050 path). */
    pin_config.pin_type = BMI2_INT1;
    pin_config.pin_cfg[0].input_en = BMI2_INT_INPUT_DISABLE;
    pin_config.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    pin_config.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    pin_config.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    pin_config.int_latch = BMI2_INT_NON_LATCH;

    if (bmi2_set_int_pin_config(&pin_config, dev_out) != BMI2_OK) {
        return false;
    }

    if (bmi2_set_sensor_config(cfg, 2, dev_out) != BMI2_OK) {
        return false;
    }

    if (bmi2_sensor_enable(sens_list, 2, dev_out) != BMI2_OK) {
        return false;
    }

    return true;
}

int8_t bmi270_esp_read_raw(struct bmi2_dev *dev, int16_t *ax, int16_t *ay, int16_t *az,
                          int16_t *gx, int16_t *gy, int16_t *gz)
{
    struct bmi2_sens_data d;
    int8_t r = bmi2_get_sensor_data(&d, dev);
    if (r != BMI2_OK) {
        return r;
    }
    *ax = d.acc.x;
    *ay = d.acc.y;
    *az = d.acc.z;
    *gx = d.gyr.x;
    *gy = d.gyr.y;
    *gz = d.gyr.z;
    return BMI2_OK;
}
