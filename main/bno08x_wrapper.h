#ifndef BNO08X_WRAPPER_H
#define BNO08X_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

// --- C Interface Types (renamed to avoid duplicate definitions) ---

typedef struct {
    float x;
    float y;
    float z;
} bno08x_euler_c_t;

typedef struct {
    float x;
    float y;
    float z;
} bno08x_gyro_c_t;

typedef struct {
    float x;
    float y;
    float z;
} bno08x_accel_c_t;

// Callback type for notifying data updates
typedef void (*bno08x_callback_t)(void);

// --- C Interface Function Prototypes ---

// Initializes the IMU; returns true if successful.
bool bno08x_initialize(void);

// Returns true if new data is available.
bool bno08x_data_available(void);

// Returns the latest Euler angles (as roll, pitch, yaw) in C format.
bno08x_euler_c_t bno08x_get_euler_angles(void);

// Returns the latest gyroscope data in C format.
bno08x_gyro_c_t bno08x_get_gyro_data(void);

// Returns the latest accelerometer data in C format.
bno08x_accel_c_t bno08x_get_accel_data(void);

// Registers a user callback that will be invoked when new IMU data is available.
void bno08x_register_callback(bno08x_callback_t callback);

#ifdef __cplusplus
}
#endif

#endif // BNO08X_WRAPPER_H
