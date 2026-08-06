#ifndef BUS_H
#define BUS_H

// bus_init
esp_err_t bus_init(spi_host_device_t host_id, int mosi_io_num, int miso_io_num, int sclk_io_num);
#endif