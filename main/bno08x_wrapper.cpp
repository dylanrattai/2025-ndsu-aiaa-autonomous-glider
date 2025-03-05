// Optional fallback definitions for configuration macros.
// In production, these should come from your project's Kconfig/sdkconfig.
#ifndef CONFIG_ESP32_BNO08X_SPI_HOST
#define CONFIG_ESP32_BNO08X_SPI_HOST 1
#endif
#ifndef CONFIG_ESP32_BNO08X_GPIO_DI
#define CONFIG_ESP32_BNO08X_GPIO_DI 23
#endif
#ifndef CONFIG_ESP32_BNO08X_GPIO_SDA
#define CONFIG_ESP32_BNO08X_GPIO_SDA 19
#endif
#ifndef CONFIG_ESP32_BNO08X_GPIO_SCL
#define CONFIG_ESP32_BNO08X_GPIO_SCL 18
#endif
#ifndef CONFIG_ESP32_BNO08X_GPIO_CS
#define CONFIG_ESP32_BNO08X_GPIO_CS 33
#endif
#ifndef CONFIG_ESP32_BNO08X_GPIO_HINT
#define CONFIG_ESP32_BNO08X_GPIO_HINT 26
#endif
#ifndef CONFIG_ESP32_BNO08X_GPIO_RST
#define CONFIG_ESP32_BNO08X_GPIO_RST 32
#endif
#ifndef CONFIG_ESP32_BNO08X_SCL_SPEED_HZ
#define CONFIG_ESP32_BNO08X_SCL_SPEED_HZ 2000000
#endif
// (Define any other missing macros as needed.)

#include "BNO08x.hpp"
#include "bno08x_wrapper.h"
#include "esp_log.h"

// Tag for ESP logging
static const char *TAG = "BNO08x_C_Wrapper";

// Create a static instance of the C++ IMU object.
static BNO08x imu;

// Store the user callback (if any)
static bno08x_callback_t user_callback = NULL;

// Internal callback function that the C++ library will invoke.
static void internal_callback_handler() {
    if (user_callback) {
        user_callback();
    }
}

extern "C" {

bool bno08x_initialize(void) {
    if (!imu.initialize()) {
        ESP_LOGE(TAG, "IMU Initialization Failed");
        return false;
    }
    // Enable various sensor reports (using 100,000 microseconds = 100ms interval)
    imu.rpt.rv_game.enable(100000UL);
    imu.rpt.cal_gyro.enable(100000UL);
    imu.rpt.linear_accelerometer.enable(100000UL);
    return true;
}

bool bno08x_data_available(void) {
    return imu.data_available();
}

bno08x_euler_c_t bno08x_get_euler_angles(void) {
    bno08x_euler_c_t result = {0.0f, 0.0f, 0.0f};
    if (imu.rpt.rv_game.has_new_data()) {
        // The library returns a C++ type (e.g., bno08x_euler_angle_t) from get_euler()
        auto angles = imu.rpt.rv_game.get_euler();
        result.x = angles.x;
        result.y = angles.y;
        result.z = angles.z;
    }
    return result;
}

bno08x_gyro_c_t bno08x_get_gyro_data(void) {
    bno08x_gyro_c_t result = {0.0f, 0.0f, 0.0f};
    if (imu.rpt.cal_gyro.has_new_data()) {
        auto gyro = imu.rpt.cal_gyro.get();
        result.x = gyro.x;
        result.y = gyro.y;
        result.z = gyro.z;
    }
    return result;
}

bno08x_accel_c_t bno08x_get_accel_data(void) {
    bno08x_accel_c_t result = {0.0f, 0.0f, 0.0f};
    if (imu.rpt.linear_accelerometer.has_new_data()) {
        auto accel = imu.rpt.linear_accelerometer.get();
        result.x = accel.x;
        result.y = accel.y;
        result.z = accel.z;
    }
    return result;
}

void bno08x_register_callback(bno08x_callback_t callback) {
    user_callback = callback;
    imu.register_cb(internal_callback_handler);
}

} // extern "C"
