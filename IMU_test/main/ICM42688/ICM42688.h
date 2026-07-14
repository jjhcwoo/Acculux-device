#ifndef ICM42688_H
#define ICM42688_H

#include "ICM42688.h"
#include "register.h"

#include "driver/spi_master.h"

typedef struct
{
    spi_device_handle_t devhdl;
    int accel_fs;
    int accel_odr;
    int gyro_fs;
    int gyro_odr;
} ICM42688;

esp_err_t icm42688_add_device(int cs_io_num, ICM42688 *imuhdl);
esp_err_t icm42688_write_register(spi_device_handle_t devhdl, uint8_t address, uint8_t* buffer, int length);
esp_err_t icm42688_read_register(spi_device_handle_t devhdl, uint8_t address, uint8_t* buffer, int length);

#endif