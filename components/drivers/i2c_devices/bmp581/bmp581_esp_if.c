/**
 * I2C glue for Bosch BMP5 API (BMP581), BSD-3-Clause.
 */
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

#include "i2cdev.h"
#include "bmp5.h"
#include "bmp581_esp.h"

typedef struct {
    I2C_Dev *i2c;
    uint8_t addr_7bit;
} bmp581_esp_ctx_t;

static bmp581_esp_ctx_t s_ctx;
static struct bmp5_dev s_dev;
static struct bmp5_osr_odr_press_config s_osr_odr;
static bool s_init;

#ifndef FIX_TEMP_ASL
#define FIX_TEMP_ASL 25.0f
#endif
#ifndef CONST_PF_ASL
#define CONST_PF_ASL 0.1902630958f
#endif

static void bmp581_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    if (period >= 1000) {
        vTaskDelay(pdMS_TO_TICKS((period + 999) / 1000));
    } else {
        esp_rom_delay_us(period);
    }
}

/** Bosch bus read (register); not the public bmp581_esp_read(pressure, temp). */
static BMP5_INTF_RET_TYPE bmp581_bus_read_reg(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr)
{
    bmp581_esp_ctx_t *ctx = (bmp581_esp_ctx_t *)intf_ptr;
    if (!ctx || !ctx->i2c || !read_data) {
        return BMP5_E_NULL_PTR;
    }
    bool ok = i2cdevReadReg8(ctx->i2c, ctx->addr_7bit, reg_addr, (uint16_t)len, read_data);
    return ok ? BMP5_INTF_RET_SUCCESS : BMP5_E_COM_FAIL;
}

static BMP5_INTF_RET_TYPE bmp581_bus_write_reg(uint8_t reg_addr, const uint8_t *write_data, uint32_t len, void *intf_ptr)
{
    bmp581_esp_ctx_t *ctx = (bmp581_esp_ctx_t *)intf_ptr;
    if (!ctx || !ctx->i2c || !write_data) {
        return BMP5_E_NULL_PTR;
    }
    bool ok = i2cdevWriteReg8(ctx->i2c, ctx->addr_7bit, reg_addr, (uint16_t)len, (uint8_t *)write_data);
    return ok ? BMP5_INTF_RET_SUCCESS : BMP5_E_COM_FAIL;
}

float bmp581_pressure_pa_to_asl_m(float pressure_pa)
{
    float p_mbar = pressure_pa / 100.0f;
    if (p_mbar > 0.0f) {
        return ((powf((1015.7f / p_mbar), CONST_PF_ASL) - 1.0f) * (FIX_TEMP_ASL + 273.15f)) / 0.0065f;
    }
    return 0.0f;
}

bool bmp581_esp_init(I2C_Dev *i2c, uint8_t i2c_addr_7bit)
{
    struct bmp5_iir_config iir = { 0 };

    if (!i2c) {
        return false;
    }

    memset(&s_dev, 0, sizeof(s_dev));
    memset(&s_osr_odr, 0, sizeof(s_osr_odr));
    s_ctx.i2c = i2c;
    s_ctx.addr_7bit = i2c_addr_7bit;
    s_dev.intf = BMP5_I2C_INTF;
    s_dev.intf_ptr = &s_ctx;
    s_dev.read = bmp581_bus_read_reg;
    s_dev.write = bmp581_bus_write_reg;
    s_dev.delay_us = bmp581_delay_us;

    if (bmp5_init(&s_dev) != BMP5_OK) {
        return false;
    }

    if (bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, &s_dev) != BMP5_OK) {
        return false;
    }

    if (bmp5_get_osr_odr_press_config(&s_osr_odr, &s_dev) != BMP5_OK) {
        return false;
    }

    s_osr_odr.odr = BMP5_ODR_50_HZ;
    s_osr_odr.press_en = BMP5_ENABLE;
    s_osr_odr.osr_t = BMP5_OVERSAMPLING_64X;
    s_osr_odr.osr_p = BMP5_OVERSAMPLING_4X;

    if (bmp5_set_osr_odr_press_config(&s_osr_odr, &s_dev) != BMP5_OK) {
        return false;
    }

    iir.set_iir_t = BMP5_IIR_FILTER_COEFF_1;
    iir.set_iir_p = BMP5_IIR_FILTER_COEFF_1;
    iir.shdw_set_iir_t = BMP5_ENABLE;
    iir.shdw_set_iir_p = BMP5_ENABLE;
    if (bmp5_set_iir_config(&iir, &s_dev) != BMP5_OK) {
        return false;
    }

    if (bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &s_dev) != BMP5_OK) {
        return false;
    }

    s_init = true;
    return true;
}

bool bmp581_esp_read(float *pressure_pa, float *temp_c)
{
    struct bmp5_sensor_data d;

    if (!s_init || !pressure_pa || !temp_c) {
        return false;
    }

    if (bmp5_get_sensor_data(&d, &s_osr_odr, &s_dev) != BMP5_OK) {
        return false;
    }

    *pressure_pa = d.pressure;
    *temp_c = d.temperature;
    return true;
}
