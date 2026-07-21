#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c_master.h"

// These variables are periodically updated when the ADC is read.
static int16_t force_sensor_top_left = 0;
static int16_t force_sensor_bottom_left = 0;
static int16_t force_sensor_top_right = 0;
static int16_t force_sensor_bottom_right = 0;

// These variables track what is the next force sensor being read.
// false = AIN0;
// true = AIN1;
// F0 = (AIN0 48) (top right)
// F1 = (AIN1 48) (bottom right)
// F2 = (AIN0 49) (bottom left)
// F3 = (AIN1 49) (top left)
static bool adc_status = false;

void update_ADC(i2c_master_dev_handle_t dev_handle_left, i2c_master_dev_handle_t dev_handle_right)
{
    // Perform the read and update
    uint8_t buffer[3];

    buffer[0] = 0;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle_left, buffer, 1, -1));
    ESP_ERROR_CHECK(i2c_master_receive(dev_handle_left, buffer, 2, -1));

    if (adc_status)
    {
        force_sensor_top_left = (int16_t) ((uint16_t)((buffer[0]) << 8) | buffer[1]);
    }
    else
    {
        force_sensor_bottom_left = (int16_t) ((uint16_t)((buffer[0]) << 8) | buffer[1]);
    }

    buffer[0] = 0;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle_right, buffer, 1, -1));
    ESP_ERROR_CHECK(i2c_master_receive(dev_handle_right, buffer, 2, -1));

    if (adc_status)
    {
        force_sensor_bottom_right = (int16_t) ((uint16_t)((buffer[0]) << 8) | buffer[1]);
    }
    else
    {
        force_sensor_top_right = (int16_t) ((uint16_t)((buffer[0]) << 8) | buffer[1]);
    }

    adc_status = !adc_status;

    // Setup the ADC for the next read;
    buffer[0] = 0b01;
    buffer[1] = 0b11000100 | ((uint8_t)adc_status << 4);
    buffer[2] = 0b10000011;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle_left, buffer, 3, -1));
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle_right, buffer, 3, -1));
}

void app_main(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = 47,
        .sda_io_num = 48,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_master_dev_handle_t dev_handle_right;
    i2c_master_dev_handle_t dev_handle_left;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x48,
        .scl_speed_hz = 100000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle_right));

    dev_cfg.device_address = 0x49;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle_left));

    while(1)
    {
        update_ADC(dev_handle_left, dev_handle_right);
        printf("%d %d %d %d\n", force_sensor_top_right, force_sensor_bottom_right, force_sensor_bottom_left, force_sensor_top_left);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
