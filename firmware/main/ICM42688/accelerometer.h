#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include "ICM42688.h"

float accelerometer_get_fs(ICM42688 *imuhdl);
float accelerometer_set_fs(ICM42688 *imuhdl, uint8_t value);

float accelerometer_get_odr(ICM42688 *imuhdl);
float accelerometer_set_odr(ICM42688 *imuhdl, uint8_t value);

int16_t accelerometer_x(ICM42688 *imuhdl);
int16_t accelerometer_y(ICM42688 *imuhdl);
int16_t accelerometer_z(ICM42688 *imuhdl);

#endif