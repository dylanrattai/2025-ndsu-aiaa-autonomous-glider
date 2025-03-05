#ifndef GPS_WRAPPER_H
#define GPS_WRAPPER_H

#include "esp_err.h"
#include "driver/uart.h"

// Structure to hold GPS data
typedef struct {
    double latitude;   // in decimal degrees
    double longitude;  // in decimal degrees
    float  speed;      // speed in m/s (converted from knots)
    float  altitude;   // altitude in meters (from GGA)
} gps_data_t;

// Initialize the GPS module (UART)
esp_err_t gps_init(uart_port_t uart_num, int tx_pin, int rx_pin);

// Read new data from GPS. Non-blocking; returns true if new data was parsed.
bool gps_read_data(gps_data_t *out_data);

#endif /* GPS_WRAPPER_H */
