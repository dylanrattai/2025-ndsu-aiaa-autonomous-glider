#include <stdio.h>
#include <stdbool.h>
#include <esp_err.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <sys/time.h>
#include "sdkconfig.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "bno08x_wrapper.h"

// PINOUT MAP, etc. (unchanged)
#define LED_1_GPIO GPIO_NUM_10 //10
#define LED_2_GPIO GPIO_NUM_11 //11
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
typedef struct {
    uint32_t min_us;
    uint32_t max_us;
} servo_params_t;
struct timeval tv;
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
double latitude;
double longitude;
double altitude;
double gps_ground_speed;
double start_time;
double start_altitude;
TaskHandle_t strobe_task_handle;
TaskHandle_t imu_task_handle;
TaskHandle_t gps_task_handle;
TaskHandle_t autonomous_flight_task_handle;
TaskHandle_t backup_flight_task_handle;
QueueHandle_t gps_queue;
ledc_channel_config_t left_servo_channel;
ledc_channel_config_t right_servo_channel;
const char *TAG = "Main";
const int UART_BUFFER_SIZE = 1024;
const double PI = 3.1415926535;
const double E = 2.718281828459045;
const double AUTONOMOUS_START_DELAY = 1500; // 1.5 seconds before auto flight starts. time to seperate from mothership
const double ORBIT_RADIUS = 15; // comp 50'
const double COMP_ORBIT_PT_LAT = 32.26558491693584; // copied from google maps (100ft northeast of runway)
const double COMP_ORBIT_PT_LONG = -111.27351705189541; // copied from google maps
const double COMP_MSL_GROUND = 2188.3;
const double ORBIT_PT_LAT = COMP_ORBIT_PT_LAT; // REAL
const double ORBIT_PT_LONG = COMP_ORBIT_PT_LONG; // REAL
const double ORBIT_MSL_GROUND = COMP_MSL_GROUND; // REAL
const double LANDING_PHASE_OFFSET_FT = 30;
const double MAX_BANK_ANGLE = 25;
const servo_params_t left_servo_params = { .min_us = 500, .max_us = 1500 };
const servo_params_t right_servo_params = { .min_us = 0, .max_us = 2100 };
const double SERVO_MAX_DEGREE = 180;
const double DESCENT_RATE = -3; // control surface degrees
const double CONTROL_ALTITUDE = 50;

double degrees_to_radians(double degrees) {
    return degrees * PI / 180.0;
}

static void send_ubx_message(const uint8_t *msg, size_t len)
{
    int tx_bytes = uart_write_bytes(UART_NUM, (const char *)msg, len);
    ESP_LOGI(TAG, "Sent %d bytes", tx_bytes);
}

void configure_nmea_output(void)
{
    // UBX-CFG-MSGOUT message for NMEA GGA on UART2:
    // Payload: [0xF0, 0x00, rate_UART1, rate_UART2, rate_USB, rate_SPI]
    // Enable only UART2 (rate=1) and disable all others (rate=0)
    uint8_t ubx_cfg_nmea_gga[] = {
        0xB5, 0x62,         // UBX header
        0x06, 0x01,         // CFG-MSGOUT message
        0x08, 0x00,         // Payload length = 8 bytes
        0xF0, 0x00,         // NMEA GGA (MsgClass=F0, MsgID=00)
        0x00,               // rate on UART1 = 0
        0x01,               // rate on UART2 = 1
        0x00,               // rate on USB   = 0
        0x00,               // rate on SPI   = 0
        0x00, 0x28          // Precalculated checksum (CK_A=0x00, CK_B=0x28)
    };

    // UBX-CFG-MSGOUT message for NMEA RMC on UART2:
    // Payload: [0xF0, 0x04, rate_UART1, rate_UART2, rate_USB, rate_SPI]
    // Enable only UART2 (rate=1) and disable all others (rate=0)
    uint8_t ubx_cfg_nmea_rmc[] = {
        0xB5, 0x62,         // UBX header
        0x06, 0x01,         // CFG-MSGOUT message
        0x08, 0x00,         // Payload length = 8 bytes
        0xF0, 0x04,         // NMEA RMC (MsgClass=F0, MsgID=04)
        0x00,               // rate on UART1 = 0
        0x01,               // rate on UART2 = 1
        0x00,               // rate on USB   = 0
        0x00,               // rate on SPI   = 0
        0x04, 0x44          // Precalculated checksum (CK_A=0x04, CK_B=0x44)
    };

    ESP_LOGI(TAG, "Configuring NMEA output on UART2");
    send_ubx_message(ubx_cfg_nmea_gga, sizeof(ubx_cfg_nmea_gga));
    send_ubx_message(ubx_cfg_nmea_rmc, sizeof(ubx_cfg_nmea_rmc));

    // Optionally, disable other unwanted messages (e.g., GSA) here by sending similar commands.
}

// Save the current configuration to battery-backed RAM (BBR)
static void save_configuration(void)
{
    // UBX-CFG-CFG message to save configuration to BBR.
    // Message structure:
    //  - Header: 0xB5, 0x62
    //  - Message Class: 0x06, Message ID: 0x09
    //  - Length: 0x0C 00 (12 bytes payload)
    //  - Payload: clearMask (4 bytes), saveMask (4 bytes), loadMask (4 bytes)
    //    Here: clearMask = 0, saveMask = 0x00000001 (save current config to BBR), loadMask = 0
    //  - Checksum: pre-calculated as 0x1C, 0x97
    uint8_t ubx_cfg_cfg_save[] = {
        0xB5, 0x62,       // UBX header
        0x06, 0x09,       // CFG-CFG message
        0x0C, 0x00,       // Payload length: 12 bytes
        0x00, 0x00, 0x00, 0x00,  // clearMask = 0
        0x01, 0x00, 0x00, 0x00,  // saveMask = 0x00000001 (save current config to BBR)
        0x00, 0x00, 0x00, 0x00,  // loadMask = 0
        0x1C, 0x97        // Checksum (CK_A, CK_B)
    };

    ESP_LOGI(TAG, "Saving configuration to BBR");
    send_ubx_message(ubx_cfg_cfg_save, sizeof(ubx_cfg_cfg_save));
}

/**
 * Initialize GPS.
 * Configure UART, set pins, install driver, and check for valid data.
*/
esp_err_t gpsInit(void)
{
    uart_config_t gps_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
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
        gps_init = false;
        return ESP_FAIL;
    }

    configure_nmea_output();
    save_configuration();

    gps_init = true;
    return ESP_OK;
}

/**
 * Parse GPS data.
 * Returns 0 for GNGGA refresh (lat, long, alt), returns 1 for GNRMC refresh (lat, long, speed), returns -1 for invalid data.
*/
// New parser function that takes a complete NMEA sentence
int parseGPS(const char *line)
{
    if(gps_init == false)
    {
        ESP_LOGE(TAG, "GPS not initialized.");
        return -1;
    }
    ESP_LOGI(TAG, "Parsing GPS data: %s", line);

    if(strncmp(line, "$GNGGA", 6) == 0)
    {
        ESP_LOGI(TAG, "Parsing GNGGA data: %s", line);
        char temp[100];
        strncpy(temp, line, sizeof(temp));
        char *tokens[20];
        int token_count = 0;
        char *token = strtok(temp, ",");

        while(token != NULL && token_count < 20)
        {
            tokens[token_count++] = token;
            token = strtok(NULL, ",");
        }

        if(token_count >= 10)
        {
            latitude = atof(tokens[2]);
            if(tokens[3][0] == 'S')
            {
                latitude *= -1;
            }

            longitude = atof(tokens[4]);
            if(tokens[5][0] == 'W')
            {
                longitude *= -1;
            }

            altitude = atof(tokens[9]);

            return 0;
        }
    }
    else if(strncmp(line, "$GNRMC", 6) == 0)
    {
        ESP_LOGI(TAG, "Parsing GNRMC data: %s", line);
        char temp[100];
        strncpy(temp, line, sizeof(temp));
        char *tokens[20];
        int token_count = 0;
        char *token = strtok(temp, ",");

        while(token != NULL && token_count < 20)
        {
            tokens[token_count++] = token;
            token = strtok(NULL, ",");
        }

        if(token_count >= 8)
        {
            latitude = atof(tokens[3]);
            if(tokens[4][0] == 'S')
            {
                latitude *= -1;
            }

            longitude = atof(tokens[5]);
            if(tokens[6][0] == 'W')
            {
                longitude *= -1;
            }

            gps_ground_speed = atof(tokens[7]);

            return 1;
        }
    }
    return -1;
}

double convertToDecimalDegrees(double ddmm) {
    double absVal = fabs(ddmm);
    double degrees = floor(absVal / 100.0);
    double minutes = absVal - (degrees * 100.0);
    double decimalDegrees = degrees + minutes / 60.0;
    // Reapply the negative sign if needed.
    if (ddmm < 0) {
        decimalDegrees = -decimalDegrees;
    }
    return decimalDegrees;
}

/**
 * Refresh GPS data.
 * Reads from the UART buffer and parses the data.
*/
void refreshGps(void)
{
    if(gps_init == false)
    {
        ESP_LOGE(TAG, "GPS not initialized.");
        return;
    }
    ESP_LOGI(TAG, "Refreshing GPS data...");
    char buffer[256];  // Use a larger buffer
    int length = uart_read_bytes(UART_NUM, buffer, sizeof(buffer) - 1, 1000 / portTICK_PERIOD_MS);
    if(length > 0)
    {
        buffer[length] = '\0';  // Null-terminate
    }
    
    bool refreshed_alt = false;
    bool refreshed_speed = false;

    // Split the buffer into lines by "\r\n"
    char *line = strtok(buffer, "\r\n");
    while(line != NULL)
    {
        ESP_LOGI(TAG, "Processing line: %s", line);
        int result = parseGPS(line);
        ESP_LOGI(TAG, "Result: %d", result);

        if(result == 0)
        {
            refreshed_alt = true;
        }
        else if(result == 1)
        {
            refreshed_speed = true;
        }

        if(refreshed_alt && refreshed_speed)
        {
            break;
        }

        line = strtok(NULL, "\r\n");
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    if (fabs(latitude) > 100) {
        latitude = convertToDecimalDegrees(latitude);
    }
    if (fabs(longitude) > 100) {
        longitude = convertToDecimalDegrees(longitude);
    }

    ESP_LOGI(TAG, "Parsed GPS data: lat: %f, long: %f, alt: %f, speed: %f", latitude, longitude, altitude, gps_ground_speed);
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
            ESP_LOGI(TAG, "Strobe LEDs");
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
double distanceFromOrbitPoint(void)
{
    refreshGps();
    double earth_radius_meters = 6378137;
    double meters_to_ft = 3.28084;
    double current_lat = latitude;
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

        vTaskDelay(500 / portTICK_PERIOD_MS);
        led_stop = !led_stop;
        ESP_LOGI(TAG, "LED Strobe: %s", led_stop ? "OFF" : "ON");

        vTaskDelay(AUTONOMOUS_START_DELAY / portTICK_PERIOD_MS);

        //refreshGps();

        // start autonomous flight
        stop = !stop;
        // set start drop distance var for flightpath spiral formula
        start_drop_distance = distanceFromOrbitPoint(); 
        gettimeofday(&tv, NULL);
        start_time = tv.tv_sec + tv.tv_usec / 1e6;
        start_altitude = altitude;

        // in case gps isnt giving a valid altitude on init
        if(start_altitude < CONTROL_ALTITUDE)
        {
            start_altitude = CONTROL_ALTITUDE;
        }
    }
}

static uint32_t servo_per_degree_init(uint32_t user_angle, servo_params_t params) {
    // Convert user angle (0–180) to native servo angle (-90 to +90)
    int native_angle = (int)user_angle - 90;  
    // Shift native angle from [-90, +90] to [0, 180]
    int shifted = native_angle + 90;
    // Map the shifted value [0, 180] to the pulse width range defined in params.
    uint32_t pulsewidth = ((params.max_us - params.min_us) * shifted) / 180 + params.min_us;
    return pulsewidth;
}

// Modified set_servo_angle now takes calibration parameters as well.
void set_servo_angle(ledc_channel_config_t *channel, uint32_t user_angle, servo_params_t params)
{
    uint32_t pulsewidth = servo_per_degree_init(user_angle, params);
    // For a 50 Hz PWM signal, period = 20000 µs.
    // With 13-bit resolution, maximum duty is 8192.
    uint32_t duty = (pulsewidth * 8192) / 20000;
    ledc_set_duty(channel->speed_mode, channel->channel, duty);
    ledc_update_duty(channel->speed_mode, channel->channel);
}

void setupControlSurfaces(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,  // 13-bit resolution (8192 steps)
        .freq_hz          = 50,                 // 50 Hz for servo control
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // 2. Configure the LEDC channel for the left servo on SERVO_LEFT_GPIO (GPIO_NUM_2)
    left_servo_channel.speed_mode     = LEDC_LOW_SPEED_MODE,
    left_servo_channel.channel        = LEDC_CHANNEL_0,
    left_servo_channel.timer_sel      = LEDC_TIMER_0,
    left_servo_channel.intr_type      = LEDC_INTR_DISABLE,
    left_servo_channel.gpio_num       = SERVO_LEFT_GPIO,
    left_servo_channel.duty           = 0,      // Initially off
    left_servo_channel.hpoint         = 0,
    ledc_channel_config(&left_servo_channel);

    // 3. Configure the LEDC channel for the right servo on SERVO_RIGHT_GPIO (GPIO_NUM_4)
    right_servo_channel.speed_mode     = LEDC_LOW_SPEED_MODE,
    right_servo_channel.channel        = LEDC_CHANNEL_1,
    right_servo_channel.timer_sel      = LEDC_TIMER_0,
    right_servo_channel.intr_type      = LEDC_INTR_DISABLE,
    right_servo_channel.gpio_num       = SERVO_RIGHT_GPIO,
    right_servo_channel.duty           = 0,      // Initially off
    right_servo_channel.hpoint         = 0,
    ledc_channel_config(&right_servo_channel);


}

/**
 * get the current distance from the orbit point
 * 
 * returns the distance in feet, - for left correction needed, + for right correction needed
*/
double getCorrectionNeeded(void)
{
    double current_distance = distanceFromOrbitPoint();
    gettimeofday(&tv, NULL);
    double current_time = tv.tv_sec + tv.tv_usec / 1e6;
    double estimated_distance = start_drop_distance * pow(E, -(current_time - start_time) / (2 * PI));

    // if in 50' orbit radius, tolerance of 10'
    if((current_distance <= ORBIT_RADIUS + 10) && (current_distance >= ORBIT_RADIUS - 10))
    {
        estimated_distance = ORBIT_RADIUS;
    }

    return estimated_distance - current_distance;
}

double getAltitudeCorrection(void)
{
    gettimeofday(&tv, NULL);
    double current_time = tv.tv_sec + tv.tv_usec / 1e6;
    // y=mx+b
    double estimated_altitude = -1 * current_time + start_altitude;
    return estimated_altitude - altitude;
}

void setHorizontalControlSurfaces(void)
{
    double correction_needed = getCorrectionNeeded();
    ESP_LOGI(TAG, "Correction needed: %f", correction_needed);
    
    double set_to_angle = 4 * correction_needed;
    ESP_LOGI(TAG, "Setting control surfaces to angle: %f", set_to_angle);

    double set_elevator_angle = getAltitudeCorrection();

    if(latitude == 0 || longitude == 0){
        ESP_LOGI(TAG, "GPS data not valid, skipping control surface adjustment.");
        return;
    }

    if(set_to_angle + set_elevator_angle > MAX_BANK_ANGLE)
    {
        ESP_LOGI(TAG, "Setting control surfaces to max bank angle.");
        set_servo_angle(&right_servo_channel, 90 - MAX_BANK_ANGLE - set_elevator_angle, right_servo_params);
        set_servo_angle(&left_servo_channel, 90 - MAX_BANK_ANGLE + set_elevator_angle, left_servo_params);
    }
    else if(set_to_angle + set_elevator_angle < -MAX_BANK_ANGLE)
    {
        ESP_LOGI(TAG, "Setting control surfaces to max bank angle.");
        set_servo_angle(&right_servo_channel, 90 + MAX_BANK_ANGLE - set_elevator_angle, right_servo_params);
        set_servo_angle(&left_servo_channel, 90 + MAX_BANK_ANGLE + set_elevator_angle, left_servo_params);
    }
    else
    {
        ESP_LOGI(TAG, "Setting control surfaces to calculated angle.");
        set_servo_angle(&right_servo_channel, 90 - set_to_angle - set_elevator_angle, right_servo_params);
        set_servo_angle(&left_servo_channel, 90 - set_to_angle + set_elevator_angle, left_servo_params);
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
            setHorizontalControlSurfaces();
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
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
    /*if(bno08xInit() != 0)
    {
        imu_init = false;
        ESP_LOGE(TAG, "Failed to init IMU.");
    }
    else imu_init = true;*/

    if(gpsInit() != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init GPS.");
    }
    else ESP_LOGI(TAG, "GPS init successful.");

    sendStartSignal();
    setupControlSurfaces();

    set_servo_angle(&right_servo_channel, 90, right_servo_params);
    set_servo_angle(&left_servo_channel, 90, left_servo_params);

    /**
     * ----- CREATE TASKS -----
    */
    xTaskCreate(strobeTask, "StrobeTask", 1000, NULL, 10, &strobe_task_handle);
    //xTaskCreate(gpsTask, "GPS Task", 4096, NULL, 8, &gps_task_handle);
    xTaskCreate(autonomousFlightTask, "Auto Flight Task", 8192, NULL, 9, &autonomous_flight_task_handle);
    //xTaskCreate(backupFlightTask, "Backup Flight Task", 4096, NULL, 5, &backup_flight_task_handle);

    // every 10ms check to start the plane, stops once started
    while (stop)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        checkToStartFlight();
    }
}