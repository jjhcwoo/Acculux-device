#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include "ICM42688.h"

float gyroscope_get_fs(ICM42688 *imuhdl);
float gyroscope_set_fs(ICM42688 *imuhdl, uint8_t value);

float gyroscope_get_odr(ICM42688 *imuhdl);
float gyroscope_set_odr(ICM42688 *imuhdl, uint8_t value);

float gyroscope_x(ICM42688 *imuhdl);
float gyroscope_y(ICM42688 *imuhdl);
float gyroscope_z(ICM42688 *imuhdl);

#endif