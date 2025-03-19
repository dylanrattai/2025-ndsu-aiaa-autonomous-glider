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
#include "test.c"

// PINOUT MAP, etc. (unchanged)
#define LED_1_GPIO GPIO_NUM_10
#define LED_2_GPIO GPIO_NUM_11
#define SERVO_LEFT_GPIO GPIO_NUM_2
#define SERVO_RIGHT_GPIO GPIO_NUM_4
#define TXD_PIN GPIO_NUM_1
#define RXD_PIN GPIO_NUM_3
#define START_PIN_OUT GPIO_NUM_39
#define START_PIN_IN GPIO_NUM_45
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

bool stop = true;
bool led_stop = true;
bool imu_init = false;
bool gps_init = false;
double roll;
double pitch;
double yaw;
double roll_accel;
double pitch_accel;
double yaw_accel;
double x_accel;
double y_accel;
double z_accel;
double start_drop_distance = 250;
TaskHandle_t strobe_task_handle;
TaskHandle_t gps_task_handle;
TaskHandle_t autonomous_flight_task_handle;
TaskHandle_t backup_flight_task_handle;
SemaphoreHandle_t gps_semaphore = NULL;
gps_data_t gps_data;
gps_data_t* p_gps_data = &gps_data;
const char *TAG = "MAIN";
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

/**
 * return latitutde as + for N and - for S
*/
double getLatitude()
{
    if (strcmp(p_gps_data->lat_direction, "N") == 0)
    {
        return atof(p_gps_data->latitude);
    }
    else
    {
        return -atof(p_gps_data->latitude);
    }
}

/**
 * return longitude as + for E and - for W
*/
double getLongitude()
{
    if(strcmp(p_gps_data->lon_direction, "E") == 0)
    {
        return atof(p_gps_data->longitude);
    }
    else
    {
        return -atof(p_gps_data->longitude);
    }
}

/**
 * returns the altitude in MSL estimated by the GPS
*/
double getMSLAltitude()
{
    return atof(p_gps_data->altitude);
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
            vTaskDelay(1500 / portTICK_PERIOD_MS);
        }
        else
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
}

void setControlSurfacePosition(double pitch, double roll, double yaw)
{
    // TODO
}

// Dummy function for servo (to be implemented)
void SetServoPositionTask(int position) {
    // Implement servo control here.
}

/**
 * calc distance using haverisne formula
 * 
 * 1 is current
 * 2 is orbit pt
*/
double distanceFromOrbitPoint()
{
    double earth_radius_meters = 6378137;
    double meters_to_ft = 3.28084;
    double current_lat = getLatitude();
    double current_long = getLongitude();
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
            //Update GPS data
            xSemaphoreGive(gps_semaphore);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // update imu data
            roll = getRoll();
            pitch = getPitch();
            yaw = getYaw();
            roll_accel = getRollAccel();
            pitch_accel = getPitchAccel();
            yaw_accel = getYawAccel();
            y_accel = getYAccel();
            x_accel = getXAccel();
            z_accel = getZAccel();

            // TODO: flightpath logic
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
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
     * ----- CREATE SEMAPHORES -----
    */
    gps_semaphore = xSemaphoreCreateBinary();
    if (gps_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create GPS semaphore");
        return;
    }

    /**
     * ----- INITILIZE THINGS -----
    */
    if(bno08xInit() != 0)
    {
        ESP_LOGE(TAG, "Failed to init IMU.");
    }
    // 
    sendStartSignal();

    /**
     * ----- CREATE TASKS -----
    */
    //xTaskCreate(strobeTask, "StrobeTask", 1000, NULL, 10, &strobe_task_handle);
    xTaskCreate(strobeTask, "StrobeTask", 4096, NULL, 10, &strobe_task_handle);
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