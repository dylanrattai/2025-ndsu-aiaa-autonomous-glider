/*
// Wrapper so the BNO08X library can be used in main.c 
*/

#ifndef BNO08X_WRAPPER_H
#define BNO08X_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

// Structure for Euler angles
typedef struct {
    float x;
    float y;
    float z;
} bno08x_euler_t;

// Structure for Gyroscope Data
typedef struct {
    float x;
    float y;
    float z;
} bno08x_gyro_t;

// Structure for Accelerometer
typedef struct {
    float x;
    float y;
    float z;
} bno08x_accel_t;

typedef void (*bno08x_callback_t)(void);

void bno08x_register_callback(bno08x_callback_t callback);

// Initialization Function
bool bno08x_initialize();

// Polling Data Availability
bool bno08x_data_available();

// Fetch Euler Angles (Rotation Data)
bno08x_euler_t bno08x_get_euler_angles();

// Fetch Gyroscope Data
bno08x_gyro_t bno08x_get_gyro_data();

bno08x_accel_t bno08x_get_accel_data();

#ifdef __cplusplus
}
#endif

#endif // BNO08X_WRAPPER_H
