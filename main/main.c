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
#include "bno08x_wrapper.h"    // Updated wrapper header
#include <string.h>
#include <stdlib.h>
#include "gps_wrapper.h"

// PINOUT MAP, etc. (unchanged)
#define LED_1_GPIO GPIO_NUM_10
#define LED_2_GPIO GPIO_NUM_11
#define SERVO_LEFT_GPIO GPIO_NUM_2
#define SERVO_RIGHT_GPIO GPIO_NUM_4
#define GPS_RX_PIN 3  // Change to actual pin
#define GPS_TX_PIN UART_PIN_NO_CHANGE

// Global variables
bool stop = false;
TaskHandle_t strobeTask;
TaskHandle_t imuTask;
static const char *TAG = "Main";

// Strobe task function
void StrobeTask(void *pvParameters)
{
    esp_rom_gpio_pad_select_gpio(LED_1_GPIO);
    gpio_set_direction(LED_1_GPIO, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(LED_2_GPIO);
    gpio_set_direction(LED_2_GPIO, GPIO_MODE_OUTPUT);

    while (!stop) {
        gpio_set_level(LED_1_GPIO, 1);
        gpio_set_level(LED_2_GPIO, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(LED_1_GPIO, 0);
        gpio_set_level(LED_2_GPIO, 0);
        vTaskDelay(1500 / portTICK_PERIOD_MS);
    }
}

// GPS task function
void gps_task(void *pvParameters) {
    gps_data_t gps_data;
    while (1) {
        if (gps_read_data(&gps_data)) {
            ESP_LOGI(TAG, "Lat: %.6f, Lon: %.6f, Speed: %.2f m/s, Altitude: %.2f m",
                     gps_data.latitude, gps_data.longitude, gps_data.speed, gps_data.altitude);
        } else {
            ESP_LOGW(TAG, "GPS data not available.");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// IMU data callback (registered via the wrapper)
void ImuDataReady(void) {
    ESP_LOGI("IMU", "New Data Available!");

    // Use the C interface types from the wrapper.
    bno08x_euler_c_t euler = bno08x_get_euler_angles();
    bno08x_gyro_c_t gyro = bno08x_get_gyro_data();
    bno08x_accel_c_t accel = bno08x_get_accel_data();

    ESP_LOGI("IMU", "Roll: %.2f, Pitch: %.2f, Yaw: %.2f", euler.x, euler.y, euler.z);
    ESP_LOGI("IMU", "Gyro X: %.2f rad/s, Y: %.2f rad/s, Z: %.2f rad/s", gyro.x, gyro.y, gyro.z);
    ESP_LOGI("IMU", "Accel X: %.2f m/s², Y: %.2f m/s², Z: %.2f m/s²", accel.x, accel.y, accel.z);
}

// IMU task
void ImuTask(void *pvParameters) {
    ESP_LOGI(TAG, "Initializing IMU.");
    if (!bno08x_initialize()) {
        ESP_LOGE(TAG, "IMU Initialization Failed!");
        vTaskDelete(NULL);
    }
    // Register callback for IMU data
    bno08x_register_callback(ImuDataReady);
    // Optionally, add a loop to poll or wait if needed.
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// Getters using the wrapper types
float GetRoll(void) {
    bno08x_euler_c_t euler = bno08x_get_euler_angles();
    return euler.x;
}
float GetPitch(void) {
    bno08x_euler_c_t euler = bno08x_get_euler_angles();
    return euler.y;
}
float GetYaw(void) {
    bno08x_euler_c_t euler = bno08x_get_euler_angles();
    return euler.z;
}
bno08x_euler_c_t GetGyroEuler(void) {
    return bno08x_get_euler_angles();
}
float GetGyroRollAccel(void) {
    bno08x_gyro_c_t gyro = bno08x_get_gyro_data();
    return gyro.x;
}
float GetGyroPitchAccel(void) {
    bno08x_gyro_c_t gyro = bno08x_get_gyro_data();
    return gyro.y;
}
float GetGyroYawAccel(void) {
    bno08x_gyro_c_t gyro = bno08x_get_gyro_data();
    return gyro.z;
}
bno08x_gyro_c_t GetGyroAccelEuler(void) {
    return bno08x_get_gyro_data();
}
float GetXAccel(void) {
    bno08x_accel_c_t accel = bno08x_get_accel_data();
    return accel.x;
}
float GetYAccel(void) {
    bno08x_accel_c_t accel = bno08x_get_accel_data();
    return accel.y;
}
float GetZAccel(void) {
    bno08x_accel_c_t accel = bno08x_get_accel_data();
    return accel.z;  // Corrected: return z, not x.
}

// Dummy function for servo (to be implemented)
void SetServoPositionTask(int position) {
    // Implement servo control here.
}

void app_main(void)
{
    stop = false;
    
    // Create the strobe task on core 1.
    xTaskCreatePinnedToCore(StrobeTask, "StrobeTask", 1000, NULL, 10, &strobeTask, 1);

    // Create the IMU task on core 0.
    xTaskCreatePinnedToCore(ImuTask, "IMU Task", 4096, NULL, 8, &imuTask, 0);

    // Initialize and start GPS if available.
    if (gps_init(UART_NUM_1, GPS_TX_PIN, GPS_RX_PIN) == ESP_OK) {
        ESP_LOGI(TAG, "GPS initialized successfully!");
        xTaskCreatePinnedToCore(gps_task, "gps_task", 4096, NULL, 5, NULL, 0);
    } else {
        ESP_LOGE(TAG, "Failed to initialize GPS.");
    }
}
