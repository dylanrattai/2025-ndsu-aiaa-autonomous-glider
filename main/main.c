#include <stdio.h>
#include <stdbool.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

/*
// PINOUT MAP
// 
// GPIO 10 - LED
// GPIO 11 - LED 2
//
// GPIO 2 - Left servo
// GPIO 4 - Right servo
//
// I2C
// GPIO 8 - SDA IMU
// GPIO 9 - SCL IMU
//
// UART
// TX - GPS TX
// RX - GPS RX
//
// ---------------------
//
// CORE JOBS
// Core 0 - Strobe
// Core 1 - Flightpath algorithm, servo manipuation
*/

// Var setup
bool stop = false;
TaskHandle_t strobeTask;
#define LED_1_GPIO GPIO_NUM_10
#define LED_2_GPIO GPIO_NUM_11
#define SERVO_LEFT_GPIO GPIO_NUM_2
#define SERVO_RIGHT_GPIO GPIO_NUM_4
#define IMU_SDA GPIO_NUM_8
#define IMU_SCL GPIO_NUM_9

/*
// Asynchronous LED strobe function
*/
void StrobeTask()
{
    // specify pins are for GPIO && set to be output pins
    esp_rom_gpio_pad_select_gpio(LED_1_GPIO);
    gpio_set_direction(LED_1_GPIO, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(LED_2_GPIO);
    gpio_set_direction(LED_2_GPIO, GPIO_MODE_OUTPUT);

    // Strobe loop
    while (!stop)
    {
        // flash
        gpio_set_level(LED_1_GPIO, 1);
        gpio_set_level(LED_2_GPIO, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(LED_1_GPIO, 0);
        gpio_set_level(LED_2_GPIO, 0);

        //wait 1.5 seconds
        vTaskDelay(1500 / portTICK_PERIOD_MS);
    }
}

/*
// Set servo position function
// position can be from -90 to 90
*/
void SetServoPositionTask(int position)
{
    
}

void app_main(void)
{
    // Strobe lights, max priority because if they are not on, no bonus points
    // Pinned to core 1 alone so it doesnt interrupt any other functions
    xTaskCreatePinnedToCore(StrobeTask, "StrobeTask", 1000, NULL, 10, &strobeTask, 1);

    // Other tasks, pinned to core 0
}
