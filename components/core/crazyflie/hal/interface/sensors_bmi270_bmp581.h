/**
 * ESP-Drone — BMI270 + BMP581 sensor HAL.
 */
#ifndef SENSORS_BMI270_BMP581_H
#define SENSORS_BMI270_BMP581_H

#include "sensors.h"

void sensorsBmi270Bmp581Init(void);
bool sensorsBmi270Bmp581Test(void);
bool sensorsBmi270Bmp581AreCalibrated(void);
bool sensorsBmi270Bmp581ManufacturingTest(void);
void sensorsBmi270Bmp581Acquire(sensorData_t *sensors, const uint32_t tick);
void sensorsBmi270Bmp581WaitDataReady(void);
bool sensorsBmi270Bmp581ReadGyro(Axis3f *gyro);
bool sensorsBmi270Bmp581ReadAcc(Axis3f *acc);
bool sensorsBmi270Bmp581ReadMag(Axis3f *mag);
bool sensorsBmi270Bmp581ReadBaro(baro_t *baro);
void sensorsBmi270Bmp581SetAccMode(accModes accMode);
void sensorsBmi270Bmp581DataAvailableCallback(void);

#endif
