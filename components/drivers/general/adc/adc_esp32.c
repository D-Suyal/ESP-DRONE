/**
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * adc.c - Analog Digital Conversion
 *
 *
 */

#include "esp_idf_version.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "sdkconfig.h"
#include "adc_esp32.h"
#include "config.h"
#include "pm_esplane.h"
#include "stm32_legacy.h"
#define DEBUG_MODULE "ADC"
#include "debug_cf.h"

static bool isInit;

static esp_adc_cal_characteristics_t *adc_chars;
#ifdef CONFIG_IDF_TARGET_ESP32
static const adc_channel_t channel = ADC_CHANNEL_7; // GPIO35 if ADC1
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
static const adc_channel_t channel = ADC_CHANNEL_1; // GPIO2 if ADC1
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
/* ESP32-S3 ADC1: channel N uses GPIO (N+1) for GPIO1..GPIO10. */
static adc_channel_t channel = ADC_CHANNEL_1;
#endif

static const adc_bits_width_t width = ADC_WIDTH_MAX-1;
static const adc_atten_t atten = 3; // we directly set the attenuation to 3(11dB/12dB) to avoid the build warning
static const adc_unit_t unit = ADC_UNIT_1;
#define DEFAULT_VREF 1100 //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   30          //Multisampling

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

float analogReadVoltage(uint32_t pin)
{
    uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        if (unit == ADC_UNIT_1) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        } else {
            int raw;
            adc2_get_raw((adc2_channel_t)channel, width, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES;
    //Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    return voltage / 1000.0;
}

#if defined(CONFIG_IDF_TARGET_ESP32S3)
static void adcS3SelectChannelFromConfigPin(void)
{
    switch (CONFIG_ADC1_PIN) {
    case 1: channel = ADC_CHANNEL_0; break;
    case 2: channel = ADC_CHANNEL_1; break;
    case 3: channel = ADC_CHANNEL_2; break;
    case 4: channel = ADC_CHANNEL_3; break;
    case 5: channel = ADC_CHANNEL_4; break;
    case 6: channel = ADC_CHANNEL_5; break;
    case 7: channel = ADC_CHANNEL_6; break;
    case 8: channel = ADC_CHANNEL_7; break;
    case 9: channel = ADC_CHANNEL_8; break;
    case 10: channel = ADC_CHANNEL_9; break;
    default:
        DEBUG_PRINTW("ADC: CONFIG_ADC1_PIN=%d invalid for ADC1; using CH1 (GPIO2)\n", CONFIG_ADC1_PIN);
        channel = ADC_CHANNEL_1;
        break;
    }
}
#endif

void adcInit(void)
{
    if (isInit) {
        return;
    }

#if defined(CONFIG_IDF_TARGET_ESP32S3)
    adcS3SelectChannelFromConfigPin();
#endif

    if (unit == ADC_UNIT_1) {
        adc1_config_width(width);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    isInit = true;
}

bool adcTest(void)
{
    return isInit;
}
