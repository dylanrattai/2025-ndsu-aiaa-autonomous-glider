#include "driver/uart.h"
#include "gps_wrapper.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include "esp_mac.h"

#define TAG "GPS_WRAPPER"
#define GPS_UART_BUF_SIZE  (1024)
#define GPS_SENTENCE_MAX_LEN 128

// Internal function to parse NMEA RMC for lat/lon/speed
static bool parse_rmc_sentence(const char *sentence, gps_data_t *out_data) {
    if (strncmp(sentence, "$GPRMC", 6) != 0 && strncmp(sentence, "$GNRMC", 6) != 0) {
        return false;
    }
    char lat_str[16], lon_str[16], ns, ew;
    float speed_knots = 0.0f;
    if (sscanf(sentence, "$GP%*3c,%*[^,],A,%[^,],%c,%[^,],%c,%f",
               lat_str, &ns, lon_str, &ew, &speed_knots) < 5) {
        return false;
    }
    double lat_deg = atof(lat_str) / 100.0;
    double lon_deg = atof(lon_str) / 100.0;
    out_data->latitude = (ns == 'S') ? -lat_deg : lat_deg;
    out_data->longitude = (ew == 'W') ? -lon_deg : lon_deg;
    out_data->speed = speed_knots * 0.514444f;
    return true;
}

// Internal function to parse NMEA GGA for altitude
static bool parse_gga_sentence(const char *sentence, gps_data_t *out_data) {
    if (strncmp(sentence, "$GPGGA", 6) != 0 && strncmp(sentence, "$GNGGA", 6) != 0) {
        return false;
    }
    char alt_str[16];
    if (sscanf(sentence, "$GP%*3c,%*[^,],%*[^,],%*c,%*[^,],%*c,%*d,%*d,%*f,%[^,],", alt_str) < 1) {
        return false;
    }
    out_data->altitude = atof(alt_str);
    return true;
}

// Read and parse GPS data from UART
bool gps_read_data(gps_data_t *out_data) {
    static char sentence[GPS_SENTENCE_MAX_LEN];
    static size_t idx = 0;
    uint8_t byte;
    while (uart_read_bytes(UART_NUM_1, &byte, 1, pdMS_TO_TICKS(20)) > 0) {
        if (byte == '\n' || byte == '\r') {
            if (idx > 0) {
                sentence[idx] = '\0';
                idx = 0;
                if (parse_rmc_sentence(sentence, out_data) || parse_gga_sentence(sentence, out_data)) {
                    return true;
                }
            }
        } else {
            if (idx < GPS_SENTENCE_MAX_LEN - 1) {
                sentence[idx++] = byte;
            } else {
                idx = 0; // Prevent buffer overflow
            }
        }
    }
    return false;
}

// Initialize GPS UART
esp_err_t gps_init(uart_port_t uart_num, int tx_pin, int rx_pin) {
    uart_config_t uart_cfg = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_PIN_NO_CHANGE, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart_num, GPS_UART_BUF_SIZE, 0, 0, NULL, 0));
    return ESP_OK;
}
