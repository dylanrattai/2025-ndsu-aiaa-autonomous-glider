#include <stdio.h>
#include <stdbool.h>
#include <esp_err.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
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
#define START_PIN_OUT 0 //tbd
#define START_PIN_IN 0 //tbd
/*#define TXD_PIN2 GPIO_NUM_17
#define RXD_PIN2 GPIO_NUM_16*/

// Global variables
typedef struct {
    char latitude[20];
    char lat_direction[4];
    char longitude[20];
    char lon_direction[4];
    char altitude[10];
} gps_data_t;

typedef struct {
    double x;
    double y;
    double altitude;
} waypoint;

bool stop = false;
TaskHandle_t strobe_task_handle;
TaskHandle_t imu_task_handle;
TaskHandle_t gps_task_handle;
TaskHandle_t autonomous_flight_Task_Handle;
QueueHandle_t uart_queue;
SemaphoreHandle_t imu_semaphore = NULL;
gps_data_t gps_data;
static const char *TAG = "Main";
static const int RX_BUFFER_SIZE = 2048;

/**
 * return latitutde as + for N and - for S
*/
double getLatitude(gps_data_t* data)
{
    if (strcmp(data->lat_direction, "N") == 0)
    {
        return atof(data->latitude);
    }
    else
    {
        return -atof(data->latitude);
    }
}

/**
 * return longitude as + for E and - for W
*/
double getLongitude(gps_data_t* data)
{
    if(strcmp(data->lon_direction, "E") == 0)
    {
        return atof(data->longitude);
    }
    else
    {
        return -atof(data->longitude);
    }
}

/**
 * returns the altitude in MSL estimated by the GPS
*/
double getMSLAltitude(gps_data_t* data)
{
    return atof(data->altitude);
}

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

/**
 * check and update gps_data every 10ms
*/
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
                // fill out gps_data
                char *token;
                int tokenIndex = 0;

                token = strtok(str2, ",");
                while (token != NULL) {
                    switch (tokenIndex) {
                        case 2: // Latitude
                            strncpy(gps_data.latitude, token, sizeof(gps_data.latitude) - 1);
                            gps_data.latitude[sizeof(gps_data.latitude) - 1] = '\0';
                            break;
                        case 3: // Latitude Direction
                            strncpy(gps_data.lat_direction, token, sizeof(gps_data.lat_direction) - 1);
                            gps_data.lat_direction[sizeof(gps_data.lat_direction) - 1] = '\0';
                            break;
                        case 4: // Longitude
                            strncpy(gps_data.longitude, token, sizeof(gps_data.longitude) - 1);
                            gps_data.longitude[sizeof(gps_data.longitude) - 1] = '\0';
                            break;
                        case 5: // Longitude Direction
                            strncpy(gps_data.lon_direction, token, sizeof(gps_data.lon_direction) - 1);
                            gps_data.lon_direction[sizeof(gps_data.lon_direction) - 1] = '\0';
                            break;
                        case 9: // Altitude
                            strncpy(gps_data.altitude, token, sizeof(gps_data.altitude) - 1);
                            gps_data.altitude[sizeof(gps_data.altitude) - 1] = '\0';
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

/**
 * Strobe 2 leds in the same pattern as the ones on commercial jets
 * (Flash, 1.5 sec wait, repeat)
 * Only run when not connected to mothership
*/
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

void setControlSurfacePosition(double pitch, double roll, double yaw)
{
    // TODO
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
        if (xSemaphoreTake(imu_semaphore, portMAX_DELAY) == pdTRUE) {
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

waypoint generateWaypoint(gps_data_t* gps_data)
{
    // TODO
    waypoint temp = {0,0,0};
    return temp;
}

/**
 * checks if the starter GPIO pin is giving power
 * if not, start the autonomous flight
 * 
 * start pin is a wire that leaves and comes back to the drone, attached to a hook, 
 * will get pulled out when the drone is dropped, so the in pin will not be receiving a signal
*/
void checkToStartFlight()
{
    if(gpio_get_level(START_PIN_IN) == 0)
    {
        stop = !stop;
    }
}

/**
 * set the out pin to send a constant signal
 * wont be received when wire is pulled
*/
void sendStartSignal()
{
    gpio_set_level(START_PIN_OUT, 1);
}

/**
 * run 
 * - refresh imu
 * - flightpath generation
 * - set control surfaces pose
*/
void autonomousFlightTask()
{
    //Update IMU data
    xSemaphoreGive(imu_semaphore);

    // TODO
}

void app_main(void)
{
    stop = false;

    imu_semaphore = xSemaphoreCreateBinary();
    if (imu_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU semaphore");
        return;
    }

    gpsInit();
    sendStartSignal();
    
    // Core 1 - Strobe, update GPS
    xTaskCreatePinnedToCore(strobeTask, "StrobeTask", 1000, NULL, 10, &strobe_task_handle, 1);
    xTaskCreatePinnedToCore(gpsTask, "GPS Task", 4096, NULL, 8, &gps_task_handle, 1);

    // Core 0 - Update IMU, generate flightpath segment, control flight surfaces
    xTaskCreatePinnedToCore(imuTask, "IMU Task", 4096, NULL, 8, &imu_task_handle, 0);
    xTaskCreatePinnedToCore(autonomousFlightTask, "Auto Flight Task", 8192, NULL, 7, &autonomous_flight_Task_Handle, 0);

    // every 10ms check to start the plane, stops once started
    while (stop)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        checkToStartFlight();
    }
}