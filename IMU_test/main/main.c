#include <stdio.h>

#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

#include "esp_log.h"

#include "freertos/freeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "ICM42688/register.h"
#include "ICM42688/ICM42688.h"
#include "ICM42688/accelerometer.h"
#include "ICM42688/gyroscope.h"
#include "bus.h"

#define PIN_NUM_MISO 32
#define PIN_NUM_MOSI 33
#define PIN_NUM_CLK 25
#define PIN_NUM_CS 26

#define GPIO_INTR_PIN 27

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    ICM42688 *imuhdl = (ICM42688*) arg;
    xQueueSendFromISR(gpio_evt_queue, &imuhdl, NULL);
}

static void gpio_task(void* arg)
{
    ICM42688 *imuhdl;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &imuhdl, portMAX_DELAY)){
            //printf("%d\n", uxQueueMessagesWaiting(gpio_evt_queue));
            printf("%0.8f,%0.8f,%0.8f,%0.8f,%0.8f,%0.8f,%d,%d\n", accelerometer_x(imuhdl), accelerometer_y(imuhdl), accelerometer_z(imuhdl), gyroscope_x(imuhdl), gyroscope_y(imuhdl), gyroscope_z(imuhdl),imuhdl->);
        }
    }
}

void app_main(void)
{
    esp_err_t ret;
    
    ret = bus_init(SPI2_HOST, PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_CLK);
    ESP_ERROR_CHECK(ret);

    ICM42688 imuhdl;

    ret = icm42688_add_device(PIN_NUM_CS, &imuhdl);
    ESP_ERROR_CHECK(ret);

    uint8_t buffer[4];

    // reset device
    buffer[0] = 1;
    icm42688_write_register(imuhdl.devhdl, DEVICE_CONFIG, buffer, 1);
    vTaskDelay(1000/ portTICK_PERIOD_MS);

    // this changes the ODR to 100Hz.
    accelerometer_set_odr(&imuhdl, 8);
    gyroscope_set_odr(&imuhdl, 8);

    // enable and configure interrupt
    // set interrupt to push pull
    buffer[0] = 0x02;
    icm42688_write_register(imuhdl.devhdl, INT_CONFIG, buffer, 1);
    buffer[0] = 0x08;
    icm42688_write_register(imuhdl.devhdl, INT_SOURCE0, buffer, 1);

    // turn on accelerometer and gyroscope
    buffer[0] = 0x1F;
    icm42688_write_register(imuhdl.devhdl, PWR_MGMT0, buffer, 1);

    // see https://github.com/espressif/esp-idf/blob/v6.0.1/examples/peripherals/gpio/generic_gpio/main/gpio_example_main.c
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1 << GPIO_INTR_PIN),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };

    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(ICM42688*));

    xTaskCreate(gpio_task, "gpio_task", 2048, NULL, 10, NULL);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(GPIO_INTR_PIN, gpio_isr_handler, (void*) &imuhdl);

    while(1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
