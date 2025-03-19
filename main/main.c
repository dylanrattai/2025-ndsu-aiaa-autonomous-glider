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
#include "esp_check.h"
#include "driver/uart.h"
#include "bno08x_wrapper.h"

// PINOUT MAP, etc. (unchanged)
#define LED_1_GPIO GPIO_NUM_10
#define LED_2_GPIO GPIO_NUM_11
#define SERVO_LEFT_GPIO GPIO_NUM_2
#define SERVO_RIGHT_GPIO GPIO_NUM_4
#define TXD_PIN GPIO_NUM_43
#define RXD_PIN GPIO_NUM_44
#define UART_NUM UART_NUM_2
#define START_PIN_OUT GPIO_NUM_39
#define START_PIN_IN GPIO_NUM_45
/*#define TXD_PIN2 GPIO_NUM_17
#define RXD_PIN2 GPIO_NUM_16*/

// Global variables
bool stop = true;
bool led_stop = true;
bool imu_init = false;
bool gps_init = false;
double start_drop_distance = 250;
double pitch;
double roll;
double yaw;
double pitch_accel;
double roll_accel;
double yaw_accel;
double y_accel;
double x_accel;
double z_accel;
double latitutde;
double longitude;
double altitude;
TaskHandle_t strobe_task_handle;
TaskHandle_t imu_task_handle;
TaskHandle_t gps_task_handle;
TaskHandle_t autonomous_flight_task_gandle;
TaskHandle_t backup_flight_task_handle;
QueueHandle_t gps_queue;
const char *TAG = "Main";
const int UART_BUFFER_SIZE = 1024;
const double PI = 3.1415926535;
const double AUTONOMOUS_START_DELAY = 1500; // 1.5 seconds before auto flight starts. time to seperate from mothership
const double ORBIT_RADIUS = 50;
const double COMP_ORBIT_PT_LAT = 32.26558491693584; // copied from google maps (100ft northeast of runway)
const double COMP_ORBIT_PT_LONG = -111.27351705189541; // copied from google maps
const double COMP_MSL_GROUND = 2188.3;
const double ORBIT_PT_LAT = 0;
const double ORBIT_PT_LONG = 0;
const double ORBIT_MSL_GROUND = 0;
const double LANDING_PHASE_OFFSET_FT = 30;

double degrees_to_radians(double degrees) {
    return degrees * PI / 180.0;
}

esp_err_t gpsInit(void)
{
    uart_config_t gps_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
    };

    // configure UART
    ESP_RETURN_ON_ERROR(uart_param_config(UART_NUM, &gps_config), TAG, "Failed to configure GPS UART.");

    // set pins
    ESP_RETURN_ON_ERROR(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "Failed to set GPS UART pins.");

    // install driver
    ESP_RETURN_ON_ERROR(uart_driver_install(UART_NUM, UART_BUFFER_SIZE * 2, 0, 10, &gps_queue, 0), TAG, "Failed to install GPS UART driver.");

    char buffer[100];
    int retry_count = 5;
    bool gps_working = false;

    while (retry_count > 0 && !gps_working) {
        int length = uart_read_bytes(UART_NUM, buffer, sizeof(buffer), 1000 / portTICK_PERIOD_MS);
        
        if (length > 0) {
            // Check for a valid NMEA sentence start
            if (strstr(buffer, "$GP") != NULL || strstr(buffer, "$GN") != NULL) {
                ESP_LOGI(TAG, "Valid GPS data received");
                gps_working = true;
            } else {
                ESP_LOGW(TAG, "Received data, but not valid GPS sentence");
            }
        } else {
            ESP_LOGW(TAG, "No data received from GPS, retrying...");
        }
        
        retry_count--;
        vTaskDelay(500 / portTICK_PERIOD_MS);  // Wait a half second before next attempt
    }

    if (!gps_working) {
        ESP_LOGE(TAG, "GPS initialization failed: No valid data received");
        return ESP_FAIL;
    }

    gps_init = true;
    return ESP_OK;
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
        if(!led_stop)
        {
            gpio_set_level(LED_1_GPIO, 1);
            gpio_set_level(LED_2_GPIO, 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);

            gpio_set_level(LED_1_GPIO, 0);
            gpio_set_level(LED_2_GPIO, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        else
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
}

/**
 * calc distance using haverisne formula
 * 
 * 1 is current
 * 2 is orbit pt
*/
double distanceFromOrbitPoint()
{
    // TODO: update lat long here 
    double earth_radius_meters = 6378137;
    double meters_to_ft = 3.28084;
    double current_lat = latitutde;
    double current_long = longitude;
    double lat_1_radians = current_lat * PI / 180;
    double lat_2_radians = ORBIT_PT_LAT * PI / 180;
    double long_1_radians = current_long * PI / 180;
    double long_2_radians = ORBIT_PT_LONG * PI / 180;
    double delta_lat = (lat_2_radians - lat_1_radians);
    double delta_long = (long_2_radians - long_1_radians);

    double a = sin(delta_lat / 2) * sin(delta_lat / 2) + 
               cos(lat_1_radians) * cos(lat_2_radians) *
               sin(delta_long / 2) * sin(delta_long / 2);
    
    double b = 2 * atan2(sqrt(a), sqrt(1 - a));

    return b * earth_radius_meters * meters_to_ft;
}

/**
 * Setup GPIO pins, then,
 * Set the out pin to send a constant signal.
 * Wont be received when wire is pulled.
*/
void sendStartSignal(void)
{
    esp_rom_gpio_pad_select_gpio(START_PIN_IN);
    gpio_set_direction(START_PIN_IN, GPIO_MODE_INPUT);
    esp_rom_gpio_pad_select_gpio(START_PIN_OUT);
    gpio_set_direction(START_PIN_OUT, GPIO_MODE_OUTPUT);

    gpio_set_level(START_PIN_OUT, 1);
}

/**
 * checks if the starter GPIO pin is giving power
 * if not, start the autonomous flight
 * 
 * start pin is a wire that leaves and comes back to the drone, attached to a hook, 
 * will get pulled out when the drone is dropped, so the in pin will not be receiving a signal
*/
void checkToStartFlight(void)
{
    /**
     * start LEDs right away, delay to auto flight for seperation from mothership.
    */
    if(gpio_get_level(START_PIN_IN) != 1)
    {
        ESP_LOGI(TAG, "---------- Starting autonomous flight. ----------");

        led_stop = !led_stop;

        vTaskDelay(AUTONOMOUS_START_DELAY / portTICK_PERIOD_MS);

        // start autonomous flight
        stop = !stop;
        // set start drop distance var for flightpath spiral formula
        start_drop_distance = distanceFromOrbitPoint(); 
    }
}

/**
 * run 
 * - refresh imu
 * - flightpath generation
 * - set control surfaces pose
 * 
 * 
 * 
 * stage 1 - enter orbit
 * Archimedean spiral
 * formula: r(theta) = distanceFromOrbitCenter * e ^ {-theta / (2 * pi)}
 * 
 * when near orbit radius, go to stage 2
 * 
 * stage 2 - orbit
 * circle with a radius of 50'
 * r(theta) = 50
 * 
 * when near ground, go to stage 3, even if in stage 1 rn
 * 
 * stage 3 - landing
 * straight line going east or west only, slow descent rate
*/
void autonomousFlightTask(void *pvParameters)
{
    while (1)
    {
        if (!stop)
        {
            


            // TODO: flightpath logic

            ESP_LOGE(TAG, "!stop");
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);

        ESP_LOGE(TAG, "Test");
    }
}

/**
 * backup incase GPS or IMU doesnt init
 * 
 * one program for imu based only, and one for neither
 * 
 * ----- IMU only -----
 * run an orbit just based off imu
 * 
 * ----- None -----
 * hold servos in a certain pose
*/
void backupFlightTask(void *pvParameters)
{

}

void app_main(void)
{
    /**
     * ----- INITILIZE THINGS -----
    */
    if(bno08xInit() != 0)
    {
        imu_init = false;
        ESP_LOGE(TAG, "Failed to init IMU.");
    }
    else imu_init = true;

    if(gpsInit() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init GPS.");
    }
    else ESP_LOGI(TAG, "GPS init successful.");

    sendStartSignal();

    /**
     * ----- CREATE TASKS -----
    */
    //xTaskCreate(strobeTask, "StrobeTask", 1000, NULL, 10, &strobe_task_handle);
    //xTaskCreate(gpsTask, "GPS Task", 4096, NULL, 8, &gps_task_handle);
    //xTaskCreate(autonomousFlightTask, "Auto Flight Task", 8192, NULL, 9, &autonomous_flight_task_handle);
    //xTaskCreate(backupFlightTask, "Backup Flight Task", 4096, NULL, 5, &backup_flight_task_handle);

    // every 10ms check to start the plane, stops once started
    while (stop)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        checkToStartFlight();
    }
}