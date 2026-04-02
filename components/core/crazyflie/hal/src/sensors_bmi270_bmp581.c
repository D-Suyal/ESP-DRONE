/**
 * HAL: Bosch BMI270 (I2C) + BMP581 (I2C), ESP32-S3 custom PCB.
 * Axis remap matches TARGET_ESP32_S2_DRONE_V1_2 legacy MPU path (not ESPLANE V1).
 * BMI270 chip axes use opposite horizontal sign vs that MPU path after remap; gyro/accel
 * x,y signs below restore the body frame expected by sensfusion6 + controller_pid.
 */
#include <math.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "projdefs.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "sensors_bmi270_bmp581.h"
#include "system.h"
#include "param.h"
#include "log.h"
#include "ledseq.h"
#include "sound.h"
#include "filter.h"
#include "config.h"
#include "stm32_legacy.h"
#include "i2cdev.h"
#include "bmi270_esp.h"
#include "bmp581_esp.h"
#include "bmi2.h"

#define DEBUG_MODULE "SENSORS"
#include "debug_cf.h"
#include "static_mem.h"

static const char *TAG_SENS = "ESPDrone";

/* Same 16-bit full-scale mapping as MPU6050 driver (65536 LSB span). */
#define SENSORS_DEG_PER_LSB_CFG    ((float)((2 * 2000.0f) / 65536.0f))
#define SENSORS_G_PER_LSB_CFG      ((float)((2 * 16) / 65536.0f))

#define SENSORS_VARIANCE_MAN_TEST_TIMEOUT M2T(2000)
#define SENSORS_MAN_TEST_LEVEL_MAX        5.0f
#define SENSORS_BIAS_SAMPLES              1000
#define SENSORS_ACC_SCALE_SAMPLES         200
#define SENSORS_GYRO_BIAS_CALCULATE_STDDEV

#define GPIO_INT_BMI270_IO     CONFIG_MPU_PIN_INT
#define GYRO_NBR_OF_AXES       3
#define GYRO_MIN_BIAS_TIMEOUT_MS M2T(1000)
#define SENSORS_NBR_OF_BIAS_SAMPLES 1024
#define GYRO_VARIANCE_BASE     5000
#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)
#define ESP_INTR_FLAG_DEFAULT  0

#define GYRO_LPF_CUTOFF_FREQ   80
#define ACCEL_LPF_CUTOFF_FREQ  30
#define SENSORS_IMU_ODR_HZ     800
#define SENSORS_BARO_EVERY_N_SAMPLES 10

#define PITCH_CALIB (CONFIG_PITCH_CALIB * 1.0f / 100.0f)
#define ROLL_CALIB  (CONFIG_ROLL_CALIB * 1.0f / 100.0f)

#if defined(CONFIG_ESPDRONE_BMI270_I2C_ADDR)
#define BMI270_ADDR_7BIT CONFIG_ESPDRONE_BMI270_I2C_ADDR
#else
#define BMI270_ADDR_7BIT 0x68
#endif
#if defined(CONFIG_ESPDRONE_BMP581_I2C_ADDR)
#define BMP581_ADDR_7BIT CONFIG_ESPDRONE_BMP581_I2C_ADDR
#else
#define BMP581_ADDR_7BIT 0x46
#endif

typedef struct {
    Axis3f bias;
    Axis3f variance;
    Axis3f mean;
    bool isBiasValueFound;
    bool isBufferFilled;
    Axis3i16 *bufHead;
    Axis3i16 buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

static xQueueHandle accelerometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(accelerometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle gyroDataQueue;
STATIC_MEM_QUEUE_ALLOC(gyroDataQueue, 1, sizeof(Axis3f));
static xQueueHandle magnetometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(magnetometerDataQueue, 1, sizeof(Axis3f));
static xQueueHandle barometerDataQueue;
STATIC_MEM_QUEUE_ALLOC(barometerDataQueue, 1, sizeof(baro_t));

static xSemaphoreHandle sensorsDataReady;
static xSemaphoreHandle dataReady;

static bool isInit;
static sensorData_t sensorData;
static volatile uint64_t imuIntTimestamp;

static Axis3i16 gyroRaw;
static Axis3i16 accelRaw;
static BiasObj gyroBiasRunning;
static Axis3f gyroBias;
static bool gyroBiasFound;
static float accScaleSum;
static float accScale = 1.0f;

static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];

static struct bmi2_dev bmi2Dev;
static bool isBarometerPresent;
static bool isImuOk;
static uint32_t baroSampleCounter;
static bool isMagnetometerPresent;

static float cosPitch, sinPitch, cosRoll, sinRoll;

static void applyAxis3fLpf(lpf2pData *data, Axis3f *in);
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut);
static bool processAccScale(int16_t ax, int16_t ay, int16_t az);
static void sensorsBiasObjInit(BiasObj *bias);
static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut);
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z);
static bool sensorsFindBiasValue(BiasObj *bias);
static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out);
static void processAccGyroMeasurements(int16_t rax, int16_t ray, int16_t raz, int16_t rgx, int16_t rgy, int16_t rgz);
static void sensorsTask(void *param);
static void sensors_inta_isr_handler(void *arg);

STATIC_MEM_TASK_ALLOC(sensorsTask, SENSORS_TASK_STACKSIZE);

bool sensorsBmi270Bmp581ReadGyro(Axis3f *gyro)
{
    return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

bool sensorsBmi270Bmp581ReadAcc(Axis3f *acc)
{
    return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

bool sensorsBmi270Bmp581ReadMag(Axis3f *mag)
{
    return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}

bool sensorsBmi270Bmp581ReadBaro(baro_t *baro)
{
    return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

void sensorsBmi270Bmp581Acquire(sensorData_t *sensors, const uint32_t tick)
{
    (void)tick;
    sensorsReadGyro(&sensors->gyro);
    sensorsReadAcc(&sensors->acc);
    sensorsReadMag(&sensors->mag);
    sensorsReadBaro(&sensors->baro);
    sensors->interruptTimestamp = sensorData.interruptTimestamp;
}

bool sensorsBmi270Bmp581AreCalibrated(void)
{
    return gyroBiasFound;
}

void sensorsBmi270Bmp581WaitDataReady(void)
{
    xSemaphoreTake(dataReady, portMAX_DELAY);
}

void sensorsBmi270Bmp581SetAccMode(accModes accMode)
{
    (void)accMode;
}

void sensorsBmi270Bmp581DataAvailableCallback(void) {}

static void sensorsTask(void *param)
{
    (void)param;
    systemWaitStart();
    vTaskDelay(M2T(200));

    while (1) {
        if (pdTRUE != xSemaphoreTake(sensorsDataReady, portMAX_DELAY)) {
            continue;
        }

        sensorData.interruptTimestamp = imuIntTimestamp;

        int16_t rax, ray, raz, rgx, rgy, rgz;
        if (bmi270_esp_read_raw(&bmi2Dev, &rax, &ray, &raz, &rgx, &rgy, &rgz) != BMI2_OK) {
            xSemaphoreGive(dataReady);
            continue;
        }

        processAccGyroMeasurements(rax, ray, raz, rgx, rgy, rgz);

        if (isBarometerPresent) {
            if (++baroSampleCounter >= SENSORS_BARO_EVERY_N_SAMPLES) {
                baroSampleCounter = 0;
                float pa, tc;
                if (bmp581_esp_read(&pa, &tc)) {
                    sensorData.baro.pressure = pa / 100.0f;
                    sensorData.baro.temperature = tc;
                    sensorData.baro.asl = bmp581_pressure_pa_to_asl_m(pa);
                }
            }
        }

        static const Axis3f zeroMag = { 0 };
        xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
        xQueueOverwrite(gyroDataQueue, &sensorData.gyro);
        xQueueOverwrite(magnetometerDataQueue, &zeroMag);
        if (isBarometerPresent) {
            xQueueOverwrite(barometerDataQueue, &sensorData.baro);
        }

        xSemaphoreGive(dataReady);
    }
}

static void IRAM_ATTR sensors_inta_isr_handler(void *arg)
{
    (void)arg;
    portBASE_TYPE hpw = pdFALSE;
    imuIntTimestamp = usecTimestamp();
    xSemaphoreGiveFromISR(sensorsDataReady, &hpw);
    if (hpw) {
        portYIELD_FROM_ISR();
    }
}

static void processAccGyroMeasurements(int16_t rax, int16_t ray, int16_t raz, int16_t rgx, int16_t rgy, int16_t rgz)
{
    Axis3f accScaled;

#ifdef CONFIG_TARGET_ESPLANE_V1
    accelRaw.x = rax;
    accelRaw.y = ray;
    accelRaw.z = raz;
    gyroRaw.x = rgx;
    gyroRaw.y = rgy;
    gyroRaw.z = rgz;
#else
    accelRaw.y = rax;
    accelRaw.x = ray;
    accelRaw.z = raz;
    gyroRaw.y = rgx;
    gyroRaw.x = rgy;
    gyroRaw.z = rgz;
#endif

    gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);

    if (gyroBiasFound) {
        processAccScale(accelRaw.x, accelRaw.y, accelRaw.z);
    }

#ifdef CONFIG_TARGET_ESPLANE_V1
    sensorData.gyro.x = (gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
    sensorData.gyro.y = (gyroRaw.y - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
#else
    /* +x, -y here = negate legacy (-x, +y) MPU convention for Bosch BMI270 raw. */
    sensorData.gyro.x = (gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
    sensorData.gyro.y = -(gyroRaw.y - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
#endif
    sensorData.gyro.z = (gyroRaw.z - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
    applyAxis3fLpf((lpf2pData *)(&gyroLpf), &sensorData.gyro);

#ifdef CONFIG_TARGET_ESPLANE_V1
    accScaled.x = (accelRaw.x) * SENSORS_G_PER_LSB_CFG / accScale;
    accScaled.y = (accelRaw.y) * SENSORS_G_PER_LSB_CFG / accScale;
#else
    accScaled.x = (accelRaw.x) * SENSORS_G_PER_LSB_CFG / accScale;
    accScaled.y = -(accelRaw.y) * SENSORS_G_PER_LSB_CFG / accScale;
#endif
    accScaled.z = (accelRaw.z) * SENSORS_G_PER_LSB_CFG / accScale;

    sensorsAccAlignToGravity(&accScaled, &sensorData.acc);
    applyAxis3fLpf((lpf2pData *)(&accLpf), &sensorData.acc);
}

static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
    static bool accBiasFound;
    static uint32_t accScaleSumCount;

    if (!accBiasFound) {
        accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) +
                             powf(az * SENSORS_G_PER_LSB_CFG, 2));
        accScaleSumCount++;
        if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES) {
            accScale = accScaleSum / (float)SENSORS_ACC_SCALE_SAMPLES;
            accBiasFound = true;
        }
    }
    return accBiasFound;
}

static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
    sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

    if (!gyroBiasRunning.isBiasValueFound) {
        if (sensorsFindBiasValue(&gyroBiasRunning)) {
            soundSetEffect(SND_CALIB);
            ledseqRun(&seq_calibrated);
            DEBUG_PRINTI("Gyro bias found (BMI270)");
        }
    }

    gyroBiasOut->x = gyroBiasRunning.bias.x;
    gyroBiasOut->y = gyroBiasRunning.bias.y;
    gyroBiasOut->z = gyroBiasRunning.bias.z;

    return gyroBiasRunning.isBiasValueFound;
}

static void sensorsBiasObjInit(BiasObj *bias)
{
    bias->isBufferFilled = false;
    bias->isBiasValueFound = false;
    bias->bufHead = bias->buffer;
}

static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut)
{
    int64_t sum[GYRO_NBR_OF_AXES] = { 0 };
    int64_t sumSq[GYRO_NBR_OF_AXES] = { 0 };

    for (uint32_t i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++) {
        sum[0] += bias->buffer[i].x;
        sum[1] += bias->buffer[i].y;
        sum[2] += bias->buffer[i].z;
        sumSq[0] += (int64_t)bias->buffer[i].x * bias->buffer[i].x;
        sumSq[1] += (int64_t)bias->buffer[i].y * bias->buffer[i].y;
        sumSq[2] += (int64_t)bias->buffer[i].z * bias->buffer[i].z;
    }

    varOut->x = (float)(sumSq[0] - (sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
    varOut->y = (float)(sumSq[1] - (sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
    varOut->z = (float)(sumSq[2] - (sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

    meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}

static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z)
{
    bias->bufHead->x = x;
    bias->bufHead->y = y;
    bias->bufHead->z = z;
    bias->bufHead++;
    if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES]) {
        bias->bufHead = bias->buffer;
        bias->isBufferFilled = true;
    }
}

static bool sensorsFindBiasValue(BiasObj *bias)
{
    static int32_t varianceSampleTime;
    bool foundBias = false;

    if (bias->isBufferFilled) {
        sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);

        if (bias->variance.x < GYRO_VARIANCE_THRESHOLD_X && bias->variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
            bias->variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
            (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < (int32_t)xTaskGetTickCount())) {
            varianceSampleTime = (int32_t)xTaskGetTickCount();
            bias->bias.x = bias->mean.x;
            bias->bias.y = bias->mean.y;
            bias->bias.z = bias->mean.z;
            foundBias = true;
            bias->isBiasValueFound = true;
        }
    }
    return foundBias;
}

static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out)
{
    Axis3f rx;
    Axis3f ry;

    rx.x = in->x;
    rx.y = in->y * cosRoll - in->z * sinRoll;
    rx.z = in->y * sinRoll + in->z * cosRoll;

    ry.x = rx.x * cosPitch - rx.z * sinPitch;
    ry.y = rx.y;
    ry.z = -rx.x * sinPitch + rx.z * cosPitch;

    out->x = ry.x;
    out->y = ry.y;
    out->z = ry.z;
}

static void applyAxis3fLpf(lpf2pData *data, Axis3f *in)
{
    for (uint8_t i = 0; i < 3; i++) {
        in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
    }
}

static void sensorsInterruptInit(void)
{
    gpio_config_t io_conf = {
#if ESP_IDF_VERSION_MAJOR > 4
        .intr_type = GPIO_INTR_POSEDGE,
#else
        .intr_type = GPIO_PIN_INTR_POSEDGE,
#endif
        .pin_bit_mask = (1ULL << GPIO_INT_BMI270_IO),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    sensorsDataReady = xSemaphoreCreateBinary();
    dataReady = xSemaphoreCreateBinary();
    gpio_config(&io_conf);
    gpio_set_intr_type(GPIO_INT_BMI270_IO, GPIO_INTR_POSEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INT_BMI270_IO, sensors_inta_isr_handler, (void *)GPIO_INT_BMI270_IO);
}

static void sensorsDeviceInit(void)
{
    isBarometerPresent = false;
    isMagnetometerPresent = false;
    isImuOk = false;
    baroSampleCounter = 0;

    printf("\r\n[ESPDrone] sensors: ~2s delay then I2C0 + BMI270/BMP581...\r\n");
    fflush(stdout);
    ESP_LOGI(TAG_SENS, "sensors: boot delay + I2C0 init (BMI270/BMP581)...");
    while (xTaskGetTickCount() < 2000) {
        vTaskDelay(M2T(50));
    }

    i2cdevInit(I2C0_DEV);

    if (!bmi270_esp_init(I2C0_DEV, BMI270_ADDR_7BIT, &bmi2Dev)) {
        ESP_LOGE(TAG_SENS, "BMI270 init FAILED (I2C addr 0x%02X, SDA/SCL/power?)", (unsigned)BMI270_ADDR_7BIT);
        DEBUG_PRINTE("BMI270 init [FAIL]\n");
        return;
    }

    ESP_LOGI(TAG_SENS, "BMI270 OK (gyro OSR4 @ 800 Hz)");
    DEBUG_PRINTI("BMI270 I2C [OK], gyro OSR4 @ 800Hz\n");
    isImuOk = true;

    if (bmp581_esp_init(I2C0_DEV, BMP581_ADDR_7BIT)) {
        isBarometerPresent = true;
        ESP_LOGI(TAG_SENS, "BMP581 OK");
        DEBUG_PRINTI("BMP581 I2C [OK]\n");
    } else {
        ESP_LOGW(TAG_SENS, "BMP581 init failed — no baro / alt-hold");
        DEBUG_PRINTW("BMP581 I2C [FAIL] — altitude hold unavailable\n");
    }

    cosPitch = cosf(PITCH_CALIB * (float)M_PI / 180.0f);
    sinPitch = sinf(PITCH_CALIB * (float)M_PI / 180.0f);
    cosRoll = cosf(ROLL_CALIB * (float)M_PI / 180.0f);
    sinRoll = sinf(ROLL_CALIB * (float)M_PI / 180.0f);

    for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&gyroLpf[i], SENSORS_IMU_ODR_HZ, GYRO_LPF_CUTOFF_FREQ);
        lpf2pInit(&accLpf[i], SENSORS_IMU_ODR_HZ, ACCEL_LPF_CUTOFF_FREQ);
    }

    sensorData.baro.pressure = 0;
    sensorData.baro.temperature = 0;
    sensorData.baro.asl = 0;
}

static void sensorsTaskInit(void)
{
    accelerometerDataQueue = STATIC_MEM_QUEUE_CREATE(accelerometerDataQueue);
    gyroDataQueue = STATIC_MEM_QUEUE_CREATE(gyroDataQueue);
    magnetometerDataQueue = STATIC_MEM_QUEUE_CREATE(magnetometerDataQueue);
    barometerDataQueue = STATIC_MEM_QUEUE_CREATE(barometerDataQueue);
    STATIC_MEM_TASK_CREATE(sensorsTask, sensorsTask, SENSORS_TASK_NAME, NULL, SENSORS_TASK_PRI);
}

void sensorsBmi270Bmp581Init(void)
{
    if (isInit) {
        return;
    }

    printf("\r\n[ESPDrone] sensorsBmi270Bmp581Init: start\r\n");
    fflush(stdout);
    ESP_LOGI(TAG_SENS, "sensorsBmi270Bmp581Init: start");
    sensorsBiasObjInit(&gyroBiasRunning);
    sensorsDeviceInit();
    if (!isImuOk) {
        ESP_LOGE(TAG_SENS, "HALT: BMI270 required — blocking until I2C works");
        DEBUG_PRINTE("BMI270 required: fix I2C address / wiring; blocking (no flight)\n");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            ESP_LOGE(TAG_SENS, "still waiting: BMI270 not responding on I2C");
            DEBUG_PRINTE("BMI270 init still failed — check BMI270 power and I2C\n");
        }
    }

    sensorsInterruptInit();
    sensorsTaskInit();
    isInit = true;
}

bool sensorsBmi270Bmp581Test(void)
{
    if (!isInit || !isImuOk) {
        DEBUG_PRINTE("Sensor init failed\n");
        return false;
    }
    if (!isBarometerPresent) {
        DEBUG_PRINTW("BMP581 missing — baro tests skipped\n");
    }
    return true;
}

bool sensorsBmi270Bmp581ManufacturingTest(void)
{
    bool testStatus = isImuOk;
    int16_t rax, ray, raz, rgx, rgy, rgz;
    uint32_t startTick = xTaskGetTickCount();

    if (!testStatus) {
        return false;
    }

    sensorsBiasObjInit(&gyroBiasRunning);
    gyroBiasFound = false;

    while (xTaskGetTickCount() - startTick < SENSORS_VARIANCE_MAN_TEST_TIMEOUT) {
        if (bmi270_esp_read_raw(&bmi2Dev, &rax, &ray, &raz, &rgx, &rgy, &rgz) != BMI2_OK) {
            vTaskDelay(M2T(5));
            continue;
        }
#ifndef CONFIG_TARGET_ESPLANE_V1
        gyroRaw.y = rgx;
        gyroRaw.x = rgy;
        gyroRaw.z = rgz;
#else
        gyroRaw.x = rgx;
        gyroRaw.y = rgy;
        gyroRaw.z = rgz;
#endif
        if (processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias)) {
            gyroBiasFound = true;
            DEBUG_PRINTI("Gyro variance test [OK] (BMI270)\n");
            break;
        }
        vTaskDelay(M2T(2));
    }

    if (!gyroBiasFound) {
        DEBUG_PRINTE("Gyro variance test [FAIL]\n");
        return false;
    }

#ifndef CONFIG_TARGET_ESPLANE_V1
    accelRaw.y = rax;
    accelRaw.x = ray;
    accelRaw.z = raz;
#else
    accelRaw.x = rax;
    accelRaw.y = ray;
    accelRaw.z = raz;
#endif

    Axis3f acc;
#ifdef CONFIG_TARGET_ESPLANE_V1
    acc.x = (accelRaw.x) * SENSORS_G_PER_LSB_CFG;
    acc.y = (accelRaw.y) * SENSORS_G_PER_LSB_CFG;
#else
    acc.x = (accelRaw.x) * SENSORS_G_PER_LSB_CFG;
    acc.y = -(accelRaw.y) * SENSORS_G_PER_LSB_CFG;
#endif
    acc.z = (accelRaw.z) * SENSORS_G_PER_LSB_CFG;

    float pitch = tanf(-acc.x / (sqrtf(acc.y * acc.y + acc.z * acc.z))) * 180.0f / (float)M_PI;
    float roll = tanf(acc.y / acc.z) * 180.0f / (float)M_PI;

    if ((fabsf(roll) < SENSORS_MAN_TEST_LEVEL_MAX) && (fabsf(pitch) < SENSORS_MAN_TEST_LEVEL_MAX)) {
        DEBUG_PRINTI("Acc level test [OK]\n");
        testStatus = true;
    } else {
        DEBUG_PRINTE("Acc level test Roll:%0.2f, Pitch:%0.2f [FAIL]\n", (double)roll, (double)pitch);
        testStatus = false;
    }

    return testStatus;
}

PARAM_GROUP_START(imu_sensors)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, BMP581, &isBarometerPresent)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, magPresent, &isMagnetometerPresent)
PARAM_GROUP_STOP(imu_sensors)

PARAM_GROUP_START(imu_tests)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bmi270, &isImuOk)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, BMP581, &isBarometerPresent)
PARAM_GROUP_STOP(imu_tests)
