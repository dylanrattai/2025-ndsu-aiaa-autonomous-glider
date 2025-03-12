#include <stdio.h>
#include <stdbool.h>
#include <esp_err.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "bno08x_wrapper.h"

// PINOUT MAP, etc. (unchanged)
#define LED_1_GPIO GPIO_NUM_10
#define LED_2_GPIO GPIO_NUM_11
#define SERVO_LEFT_GPIO GPIO_NUM_2
#define SERVO_RIGHT_GPIO GPIO_NUM_4
#define TXD_PIN GPIO_NUM_1
#define RXD_PIN GPIO_NUM_3
/*#define TXD_PIN2 GPIO_NUM_17
#define RXD_PIN2 GPIO_NUM_16*/

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
// Test GPS
*/

// Global variables
bool stop = false;
TaskHandle_t strobeTaskHandle;
TaskHandle_t imuTaskHandle;
TaskHandle_t gpsTaskHandle;
static const char *TAG = "Main";
QueueHandle_t uart_queue;
static const int RX_BUFFER_SIZE = 2048;
SemaphoreHandle_t imuSemaphore = NULL;

typedef struct {
    char latitude[20];
    char lat_direction[4];
    char longitude[20];
    char lon_direction[4];
    char altitude[10];
} gps_data_t;

gps_data_t gpsData;

// UART 0
void gpsInit(void)
{
    const uart_config_t uart_config =
    {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(UART_NUM_0, RX_BUFFER_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void gpsTask(void *arg)
{
    const char *str = "$GPGGA";
    char *p = NULL;
    char str1[100];
    int j = 0;

    char *str2 = (char*)malloc(sizeof(str1));
    uint8_t* data = (uint8_t*)malloc(RX_BUFFER_SIZE + 1);

    while (1)
    {
        const int rxbytes = uart_read_bytes(UART_NUM_0, data, RX_BUFFER_SIZE, 1000 / portTICK_PERIOD_MS);

        if(rxbytes > 0)
        {
            p = strstr((const char*)data, str);
            if(p)
            {
                for(int i = 0; p[i] != '\n'; i++)
                {
                    str2[j] = p[i];
                    j++;
                }
                str2[j + 1] = '\n';
                const int len = j + 2;
                data[rxbytes] = 0;

                // 2 = Lat value
                // 3 = Lat direction
                // 4 = Long value
                // 5 = Long direction
                // 9 = Altitude
                // fill out gpsData
                char *token;
                int tokenIndex = 0;

                token = strtok(str2, ",");
                while (token != NULL) {
                    switch (tokenIndex) {
                        case 2: // Latitude
                            strncpy(gpsData.latitude, token, sizeof(gpsData.latitude) - 1);
                            gpsData.latitude[sizeof(gpsData.latitude) - 1] = '\0';
                            break;
                        case 3: // Latitude Direction
                            strncpy(gpsData.lat_direction, token, sizeof(gpsData.lat_direction) - 1);
                            gpsData.lat_direction[sizeof(gpsData.lat_direction) - 1] = '\0';
                            break;
                        case 4: // Longitude
                            strncpy(gpsData.longitude, token, sizeof(gpsData.longitude) - 1);
                            gpsData.longitude[sizeof(gpsData.longitude) - 1] = '\0';
                            break;
                        case 5: // Longitude Direction
                            strncpy(gpsData.lon_direction, token, sizeof(gpsData.lon_direction) - 1);
                            gpsData.lon_direction[sizeof(gpsData.lon_direction) - 1] = '\0';
                            break;
                        case 9: // Altitude
                            strncpy(gpsData.altitude, token, sizeof(gpsData.altitude) - 1);
                            gpsData.altitude[sizeof(gpsData.altitude) - 1] = '\0';
                            break;
                        default:
                            break;
                    }
                    token = strtok(NULL, ",");
                    tokenIndex++;
                }
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    free(str2);
    free(data);
}

// Strobe task function
void strobeTask(void *pvParameters)
{
    esp_rom_gpio_pad_select_gpio(LED_1_GPIO);
    gpio_set_direction(LED_1_GPIO, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(LED_2_GPIO);
    gpio_set_direction(LED_2_GPIO, GPIO_MODE_OUTPUT);

    // only strobe while running drone
    while (1) 
    {
        if(!stop)
        {
            gpio_set_level(LED_1_GPIO, 1);
            gpio_set_level(LED_2_GPIO, 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_1_GPIO, 0);
            gpio_set_level(LED_2_GPIO, 0);
            vTaskDelay(1500 / portTICK_PERIOD_MS);
        }
    }
}

// IMU data callback (registered via the wrapper)
void imuDataReady(void) {
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
void imuTask(void *pvParameters) {
    ESP_LOGI(TAG, "Initializing IMU.");
    if (!bno08x_initialize()) {
        ESP_LOGE(TAG, "IMU Initialization Failed!");
        vTaskDelete(NULL);
    }

    bno08x_register_callback(imuDataReady);

    while (1) {
        if (xSemaphoreTake(imuSemaphore, portMAX_DELAY) == pdTRUE) {
            // Read IMU data and process it on demand
            bno08x_euler_c_t euler = bno08x_get_euler_angles();
            bno08x_gyro_c_t  gyro  = bno08x_get_gyro_data();
            bno08x_accel_c_t accel = bno08x_get_accel_data();

            ESP_LOGI("IMU", "Roll: %.2f, Pitch: %.2f, Yaw: %.2f", euler.x, euler.y, euler.z);
            ESP_LOGI("IMU", "Gyro X: %.2f, Y: %.2f, Z: %.2f", gyro.x, gyro.y, gyro.z);
            ESP_LOGI("IMU", "Accel X: %.2f, Y: %.2f, Z: %.2f", accel.x, accel.y, accel.z);

            // Flightpath calculation and servo control could be done here.
        }
    }
}

// Getters using the wrapper types
float getRoll(void) {
    bno08x_euler_c_t euler = bno08x_get_euler_angles();
    return euler.x;
}
float getPitch(void) {
    bno08x_euler_c_t euler = bno08x_get_euler_angles();
    return euler.y;
}
float getYaw(void) {
    bno08x_euler_c_t euler = bno08x_get_euler_angles();
    return euler.z;
}
bno08x_euler_c_t GetGyroEuler(void) {
    return bno08x_get_euler_angles();
}
float getGyroRollAccel(void) {
    bno08x_gyro_c_t gyro = bno08x_get_gyro_data();
    return gyro.x;
}
float getGyroPitchAccel(void) {
    bno08x_gyro_c_t gyro = bno08x_get_gyro_data();
    return gyro.y;
}
float getGyroYawAccel(void) {
    bno08x_gyro_c_t gyro = bno08x_get_gyro_data();
    return gyro.z;
}
bno08x_gyro_c_t GetGyroAccelEuler(void) {
    return bno08x_get_gyro_data();
}
float getXAccel(void) {
    bno08x_accel_c_t accel = bno08x_get_accel_data();
    return accel.x;
}
float getYAccel(void) {
    bno08x_accel_c_t accel = bno08x_get_accel_data();
    return accel.y;
}
float getZAccel(void) {
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

    imuSemaphore = xSemaphoreCreateBinary();
    if (imuSemaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU semaphore");
        return;
    }

    gpsInit();
    
    // Core 1 - Strobe, update GPS
    xTaskCreatePinnedToCore(strobeTask, "StrobeTask", 1000, NULL, 10, &strobeTaskHandle, 1);
    xTaskCreatePinnedToCore(gpsTask, "GPS Task", 4096, NULL, 8, &gpsTaskHandle, 1);

    // Core 0 - Update IMU, generate flightpath segment, control flight surfaces
    xTaskCreatePinnedToCore(imuTask, "IMU Task", 4096, NULL, 8, &imuTaskHandle, 0);

    // flightpath, control surfaces, and imu update loop
    while (1) {
        vTaskDelay(50 / portTICK_PERIOD_MS); // every 50ms
        xSemaphoreGive(imuSemaphore); // update imu data
    }
}