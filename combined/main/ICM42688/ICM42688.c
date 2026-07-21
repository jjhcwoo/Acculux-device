#include "esp_err.h"
#include "driver/spi_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ICM42688/accelerometer.h"
#include "ICM42688/gyroscope.h"

#include "ICM42688.h"

#define ADDRESS_BITS 8
#define SPI_MODE 3
#define CLOCK_SPEED 500000

esp_err_t icm42688_add_device(spi_host_device_t host_id, int cs_io_num, ICM42688 *imuhdl)
{
    spi_device_interface_config_t devcfg = {
        .address_bits = ADDRESS_BITS,
        .mode = SPI_MODE,
        .clock_speed_hz = CLOCK_SPEED,
        .queue_size = 1,
        .spics_io_num = cs_io_num,
    };

    esp_err_t ret = spi_bus_add_device(host_id, &devcfg, &imuhdl->devhdl);

    // is delay needed before reading from device?
    vTaskDelay(100 / portTICK_PERIOD_MS);

    accelerometer_get_fs(imuhdl);
    accelerometer_get_odr(imuhdl);
    gyroscope_get_fs(imuhdl);
    gyroscope_get_odr(imuhdl);

    return ret;
}

// User is responsible for allocating and deallocating data memory
// length in byte
esp_err_t icm42688_write_register(spi_device_handle_t devhdl, uint8_t address, uint8_t* buffer, int length)
{
    spi_transaction_t transdesc = {
        .addr = 0 << 7 | address,
        .length = length * 8,
        .tx_buffer = buffer,
    };

    return (spi_device_polling_transmit(devhdl, &transdesc));
}

// User is responsible for allocating and deallocating data memory
// length in bits
esp_err_t icm42688_read_register(spi_device_handle_t devhdl, uint8_t address, uint8_t* buffer, int length)
{
    spi_transaction_t transdesc = {
        .addr = 1 << 7 | address,
        .length = length * 8,
        .rx_buffer = buffer,
    };

    return (spi_device_polling_transmit(devhdl, &transdesc));
}