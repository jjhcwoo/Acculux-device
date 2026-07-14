#include "esp_err.h"
#include "driver/spi_master.h"

#include "bus.h"

esp_err_t bus_init(spi_host_device_t host_id, int mosi_io_num, int miso_io_num, int sclk_io_num)
{
    // Max transfer size could be changed. Probably set to 16 for FIFO on ICM-42688-P
    spi_bus_config_t buscfg = {
        .mosi_io_num = mosi_io_num,
        .miso_io_num = miso_io_num,
        .sclk_io_num = sclk_io_num,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    return spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
}