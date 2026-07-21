#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/i2c_master.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_err.h"
#include "esp_log.h"

#include "ICM42688/register.h"
#include "ICM42688/ICM42688.h"
#include "ICM42688/accelerometer.h"
#include "ICM42688/gyroscope.h"
#include "bus.h"

#include "gpio_connections.h"

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

static QueueHandle_t gpio_evt_queue = NULL;

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

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    ICM42688 *imuhdl = (ICM42688*) arg;
    xQueueSendFromISR(gpio_evt_queue, &imuhdl, NULL);
}

static void gpio_task(void* arg)
{
    ICM42688 *imu0hdl;
    ICM42688 *imu1hdl;

    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &imu0hdl, portMAX_DELAY) && xQueueReceive(gpio_evt_queue, &imu1hdl, portMAX_DELAY)) {
            //printf("%d\n", uxQueueMessagesWaiting(gpio_evt_queue));
            printf("%0.8f,%0.8f,%0.8f,", accelerometer_x(imu0hdl), accelerometer_y(imu0hdl), accelerometer_z(imu0hdl));
            printf("%0.8f,%0.8f,%0.8f,", gyroscope_x(imu0hdl), gyroscope_y(imu0hdl), gyroscope_z(imu0hdl));
            printf("%0.8f,%0.8f,%0.8f,", accelerometer_x(imu1hdl), accelerometer_y(imu1hdl), accelerometer_z(imu1hdl));
            printf("%0.8f,%0.8f,%0.8f,", gyroscope_x(imu1hdl), gyroscope_y(imu1hdl), gyroscope_z(imu1hdl));
            printf("%d,%d,%d,%d", force_sensor_top_right, force_sensor_bottom_right, force_sensor_bottom_left, force_sensor_top_left);
            printf("\n");
        }
    }
}

void app_main(void)
{
    uart_set_baudrate(UART_NUM_0, 921600);

    esp_err_t ret;
    uint8_t buffer[4];

    ESP_LOGI("IMU0", "Setting up");
    ret = bus_init(SPI2_HOST, GPIO_IMU0_SDO, GPIO_IMU0_SDI, GPIO_IMU0_CLK);
    ESP_ERROR_CHECK(ret);

    ICM42688 imu0hdl;

    ret = icm42688_add_device(SPI2_HOST, GPIO_IMU0_CS, &imu0hdl);
    ESP_ERROR_CHECK(ret);

    // reset device
    buffer[0] = 1;
    icm42688_write_register(imu0hdl.devhdl, DEVICE_CONFIG, buffer, 1);
    vTaskDelay(1000/ portTICK_PERIOD_MS);

    // this changes the ODR to 100Hz.
    accelerometer_set_odr(&imu0hdl, 8);
    gyroscope_set_odr(&imu0hdl, 8);

    // enable and configure interrupt
    // set interrupt to push pull
    buffer[0] = 0x02;
    icm42688_write_register(imu0hdl.devhdl, INT_CONFIG, buffer, 1);
    buffer[0] = 0x08;
    icm42688_write_register(imu0hdl.devhdl, INT_SOURCE0, buffer, 1);
    ESP_LOGI("IMU0", "Set up done");

    ESP_LOGI("IMU1", "Setting up");
    ret = bus_init(SPI3_HOST, GPIO_IMU1_SDO, GPIO_IMU1_SDI, GPIO_IMU1_CLK);
    ESP_ERROR_CHECK(ret);

    ICM42688 imu1hdl;

    ret = icm42688_add_device(SPI3_HOST, GPIO_IMU1_CS, &imu1hdl);
    ESP_ERROR_CHECK(ret);

    // reset device
    buffer[0] = 1;
    icm42688_write_register(imu1hdl.devhdl, DEVICE_CONFIG, buffer, 1);
    vTaskDelay(1000/ portTICK_PERIOD_MS);

    // this changes the ODR to 100Hz.
    accelerometer_set_odr(&imu1hdl, 8);
    gyroscope_set_odr(&imu1hdl, 8);

    // enable and configure interrupt
    // set interrupt to push pull
    buffer[0] = 0x02;
    icm42688_write_register(imu1hdl.devhdl, INT_CONFIG, buffer, 1);
    buffer[0] = 0x08;
    icm42688_write_register(imu1hdl.devhdl, INT_SOURCE0, buffer, 1);
    ESP_LOGI("IMU1", "Set up done");

    // turn on accelerometer and gyroscope
    buffer[0] = 0x1F;
    icm42688_write_register(imu0hdl.devhdl, PWR_MGMT0, buffer, 1);

    // turn on accelerometer and gyroscope
    buffer[0] = 0x1F;
    icm42688_write_register(imu1hdl.devhdl, PWR_MGMT0, buffer, 1);

    // see https://github.com/espressif/esp-idf/blob/v6.0.1/examples/peripherals/gpio/generic_gpio/main/gpio_example_main.c
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = ((1 << GPIO_IMU0_INT)|(1<<GPIO_IMU1_INT)),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };

    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(ICM42688*));

    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(GPIO_IMU0_INT, gpio_isr_handler, (void*) &imu0hdl);
    ESP_LOGI("IMU0", "Interrupt registered");

    gpio_isr_handler_add(GPIO_IMU1_INT, gpio_isr_handler, (void*) &imu1hdl);
    ESP_LOGI("IMU1", "Interrupt registered");

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = GPIO_SCL,
        .sda_io_num = GPIO_SDA,
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
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}