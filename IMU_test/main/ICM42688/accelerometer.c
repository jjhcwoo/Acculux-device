#include <stdio.h>

#include "ICM42688.h"
#include "register.h"

#include "esp_err.h"
#include "driver/spi_master.h"

#include "accelerometer.h"

static float convert_fs(uint8_t fs)
{
    switch (fs)
    {
        case 0:
            return 16;
        case 1:
            return 8;
        case 2:
            return 4;
        case 3:
            return 2;
        default:
            return 0;
    }
}

static float convert_odr(uint8_t odr)
{
    switch (odr)
    {
        case 1:
            return 32000;
        case 2:
            return 16000;
        case 3:
            return 8000;
        case 4:
            return 4000;
        case 5:
            return 2000;
        case 6:
            return 1000;
        case 7:
            return 200;
        case 8:
            return 100;
        case 9:
            return 50;
        case 10:
            return 25;
        case 11:
            return 12.5;
        case 12:
            return 6.25;
        case 13:
            return 3.125;
        case 14:
            return 1.5625;
        case 15:
            return 500;
        default:
            return 0;
    }
}

// converts the IMU reading from the accelerometer into mm/s^2
static float convert_accel(ICM42688 *imuhdl, uint8_t upper_byte, uint8_t lower_byte)
{
    int16_t accel_bit = ((uint16_t)upper_byte << 8) | lower_byte;
    float accel = (float)((int32_t)accel_bit * (int32_t)convert_fs(imuhdl->accel_fs)) / 32768;
    return accel;
}

float accelerometer_get_fs(ICM42688 *imuhdl)
{
    uint8_t buffer[1];

    esp_err_t ret;
    ret = icm42688_read_register(imuhdl->devhdl, ACCEL_CONFIG0, buffer, 1);
    ESP_ERROR_CHECK(ret);

    // fs is stored from bit 7:5
    imuhdl->accel_fs = (buffer[0] & 0xE0) >> 5;
    return convert_fs(imuhdl->accel_fs);
}

float accelerometer_set_fs(ICM42688 *imuhdl, uint8_t value)
{
    uint8_t buffer[1];

    // check if input is valid
    if (!(value <= 3))
    {
        return 0;
    }

    esp_err_t ret;

    ret = icm42688_read_register(imuhdl->devhdl, ACCEL_CONFIG0, buffer, 1);
    ESP_ERROR_CHECK(ret);

    buffer[0] = (buffer[0] & 0x0F) | value << 5;

    ret = icm42688_write_register(imuhdl->devhdl, ACCEL_CONFIG0, buffer, 1);
    ESP_ERROR_CHECK(ret);

    imuhdl->accel_fs = value;
    return convert_fs(imuhdl->accel_fs);
}

float accelerometer_get_odr(ICM42688 *imuhdl)
{
    uint8_t buffer[1];

    esp_err_t ret;
    ret = icm42688_read_register(imuhdl->devhdl, ACCEL_CONFIG0, buffer, 1);
    ESP_ERROR_CHECK(ret);

    // fs is stored from bit 3:0
    imuhdl->accel_odr = (buffer[0] & 0x0F);
    return convert_odr(imuhdl->accel_odr);
}

float accelerometer_set_odr(ICM42688 *imuhdl, uint8_t value)
{
    uint8_t buffer[1];

    // check if input is valid
    if (!(value >= 1 && value <= 15))
    {
        return 0;
    }

    esp_err_t ret;

    ret = icm42688_read_register(imuhdl->devhdl, ACCEL_CONFIG0, buffer, 1);
    ESP_ERROR_CHECK(ret);
    
    buffer[0] = (buffer[0] & 0xE0) | value;

    ret = icm42688_write_register(imuhdl->devhdl, ACCEL_CONFIG0, buffer, 1);
    ESP_ERROR_CHECK(ret);

    imuhdl->accel_odr = value;
    return convert_odr(imuhdl->accel_odr);
}

float accelerometer_x(ICM42688 *imuhdl)
{
    uint8_t buffer[2];

    esp_err_t ret;
    ret = icm42688_read_register(imuhdl->devhdl, ACCEL_DATA_X, buffer, 2);
    ESP_ERROR_CHECK(ret);

    return convert_accel(imuhdl, buffer[0], buffer[1]);
}

float accelerometer_y(ICM42688 *imuhdl)
{
    uint8_t buffer[2];

    esp_err_t ret;
    ret = icm42688_read_register(imuhdl->devhdl, ACCEL_DATA_Y, buffer, 2);
    ESP_ERROR_CHECK(ret);

    return convert_accel(imuhdl, buffer[0], buffer[1]);
}

float accelerometer_z(ICM42688 *imuhdl)
{
    uint8_t buffer[2];

    esp_err_t ret;
    ret = icm42688_read_register(imuhdl->devhdl, ACCEL_DATA_Z, buffer, 2);
    ESP_ERROR_CHECK(ret);

    return convert_accel(imuhdl, buffer[0], buffer[1]);
}