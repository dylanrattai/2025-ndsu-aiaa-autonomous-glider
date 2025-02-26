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
#include <esp_err.h>
#include "bno08x_wrapper.h"

/*
// PINOUT MAP
// 
// GPIO 10 - LED - Gray
// GPIO 11 - LED 2 - Gray
//
// GPIO 2 - Left servo - White
// GPIO 4 - Right servo - Purple
//
// IMU, using SPI
// GPIO 16 - IMU INT - Purple
// GPIO 15 - IMU reset - Brown
// GPIO 7 - IMU chip select - Gray
// GPIO 18 - IMU SCL - Blue
// GPIO 17 - IMU SDA - Green
// GPIO 8 - IMU DI - Yellow
//
// UART
// TX - GPS TX - White
// RX - GPS RX - Yellow
//
// ---------------------
//
// CORE JOBS
// Core 0 - Strobe
// Core 1 - Flightpath algorithm, servo manipuation
*/

/*
// BNO085 Euler Info
//
// Gyro (degrees)
// type - bno08x_euler_t
// x - Roll
// y - Pitch
// z - Yaw
//
// Gyro Accelerometer (rad/s)
// type - bno08x_gyro_t
// x - Roll
// y - Pitch
// z - Yaw
//
// Linear Accelerometer (m/s^2)
// type - bno08x_accel_t
// x - x acceleration
// y - y acceleration
// z - z acceleration
*/

/*
// TODO:
// Test IMU connection & output
*/

// Var setup
bool stop = false;
TaskHandle_t strobeTask;
TaskHandle_t imuTask;
static const char *TAG = "Main";
#define LED_1_GPIO GPIO_NUM_10
#define LED_2_GPIO GPIO_NUM_11
#define SERVO_LEFT_GPIO GPIO_NUM_2
#define SERVO_RIGHT_GPIO GPIO_NUM_4

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

void ImuDataReady() {
    ESP_LOGI("IMU", "New Data Available!");

    // Fetch data from the IMU
    bno08x_euler_t euler = bno08x_get_euler_angles();
    bno08x_gyro_t gyro = bno08x_get_gyro_data();
    bno08x_accel_t accel = bno08x_get_accel_data();

    // Log the data
    ESP_LOGI("IMU", "Roll: %.2f, Pitch: %.2f, Yaw: %.2f", euler.x, euler.y, euler.z);
    ESP_LOGI("IMU", "Gyro X: %.2f rad/s, Y: %.2f rad/s, Z: %.2f rad/s", gyro.x, gyro.y, gyro.z);
    ESP_LOGI("IMU", "Accel X: %.2f m/s², Y: %.2f m/s², Z: %.2f m/s²", accel.x, accel.y, accel.z);
}

// Init IMU
void ImuTask()
{
    ESP_LOGI(TAG, "Initializing IMU.");

    if (!bno08x_initialize()) {
        ESP_LOGE(TAG, "IMU Initialization Failed!");
        vTaskDelete(NULL);
    }
}

// Call in the start of any function that uses imu information
void UpdateImu()
{
    bno08x_register_callback(ImuDataReady);
}

/*
// Getters for Roll, Pitch, Yaw, Gyro Euler; Xacc, Yacc, Zacc, Acceleration Euler.
*/
float GetRoll()
{
    bno08x_euler_t euler = bno08x_get_euler_angles();
    return euler.x;
}
float GetPitch()
{
    bno08x_euler_t euler = bno08x_get_euler_angles();
    return euler.y;
}
float GetYaw()
{
    bno08x_euler_t euler = bno08x_get_euler_angles();
    return euler.z;
}
bno08x_euler_t GetGyroEuler()
{
    return bno08x_get_euler_angles();
}
// Gyro Accelerometer
float GetGyroRollAccel()
{
    bno08x_gyro_t gyro = bno08x_get_gyro_data();
    return gyro.x;
}
float GetGyroPitchAccel()
{
    bno08x_gyro_t gyro = bno08x_get_gyro_data();
    return gyro.y;
}
float GetGyroYawAccel()
{
    bno08x_gyro_t gyro = bno08x_get_gyro_data();
    return gyro.z;
}
bno08x_gyro_t GetGyroAccelEuler()
{
    return bno08x_get_gyro_data();
}
// Linear Accelerometer
float GetXAccel()
{
    bno08x_accel_t accel = bno08x_get_accel_data();
    return accel.x;
}
float GetYAccel()
{
    bno08x_accel_t accel = bno08x_get_accel_data();
    return accel.y;
}
float GetZAccel()
{
    bno08x_accel_t accel = bno08x_get_accel_data();
    return accel.x;
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
    stop = true; // stop program until limit switch detects it is disconnected from the plane

    // Strobe lights, max priority because if they are not on, no bonus points
    // Pinned to core 1 alone so it doesnt interrupt any other functions
    xTaskCreatePinnedToCore(StrobeTask, "StrobeTask", 1000, NULL, 10, &strobeTask, 1);

    // Other tasks, pinned to core 0
    xTaskCreatePinnedToCore(ImuTask, "IMU Task", 4096, NULL, 8, &imuTask, 0);
}
