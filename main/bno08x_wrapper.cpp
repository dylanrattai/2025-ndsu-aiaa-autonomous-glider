#include "BNO08x.hpp"
#include "bno08x_wrapper.h"
#include "esp_log.h"

static const constexpr char *TAG = "BNO08x_C_Wrapper";

static BNO08x imu;  // IMU Object
static bno08x_callback_t imu_callback = nullptr;  // Store user-defined callback function

// Internal function to handle IMU updates
static void imu_data_handler() {
    if (imu_callback) {
        imu_callback();  // Call the registered callback function
    }
}

extern "C" {

// Initialize the IMU
bool bno08x_initialize() {
    if (!imu.initialize()) {
        ESP_LOGE(TAG, "IMU Initialization Failed");
        return false;
    }
    imu.rpt.rv_game.enable(100000UL);  // Enable Game Rotation Vector (100ms interval)
    imu.rpt.cal_gyro.enable(100000UL); // Enable Calibrated Gyro Data (100ms interval)
    imu.rpt.linear_accelerometer.enable(100000UL);
    return true;
}

// Check if new data is available
bool bno08x_data_available() {
    return imu.data_available();
}

// Get Euler angles (Roll, Pitch, Yaw)
bno08x_euler_t bno08x_get_euler_angles() {
    bno08x_euler_t euler = {0};
    if (imu.rpt.rv_game.has_new_data()) {
        bno08x_euler_angle_t angles = imu.rpt.rv_game.get_euler();
        euler.x = angles.x;
        euler.y = angles.y;
        euler.z = angles.z;
    }
    return euler;
}

// Get Gyroscope Data
bno08x_gyro_t bno08x_get_gyro_data() {
    bno08x_gyro_t gyro = {0};
    if (imu.rpt.cal_gyro.has_new_data()) {
        bno08x_gyro_t velocity = imu.rpt.cal_gyro.get();
        gyro.x = velocity.x;
        gyro.y = velocity.y;
        gyro.z = velocity.z;
    }
    return gyro;
}

void bno08x_register_callback(bno08x_callback_t callback) {
    imu_callback = callback;  // Store the callback function
    imu.register_cb(imu_data_handler);  // Register the internal handler in C++
}

// Get Accelerometer Data
bno08x_accel_t bno08x_get_accel_data() {
    bno08x_accel_t accel = {0.0f, 0.0f, 0.0f};
    if (imu.rpt.linear_accelerometer.has_new_data()) {
        bno08x_accel_t acceleration = imu.rpt.linear_accelerometer.get();
        accel.x = acceleration.x;
        accel.y = acceleration.y;
        accel.z = acceleration.z;
    }
    return accel;
}

} // extern "C"
