#include "bno08x_wrapper.h"
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

const char *TAGT = "TEST";

void printImuQuaternion(void)
{
    ESP_LOGI(TAGT, "Quaternion: \nPitch: %f, Roll: %f, Yaw: %f.\n\n", getPitch(), getRoll(), getYaw());
}

void printImuGyro(void)
{
    ESP_LOGI(TAGT, "Gyro: \nPitch: %f, Roll: %f, Yaw: %f.\n\n", getPitchAccel(), getRollAccel(), getYawAccel());
}

void printImuAccel(void)
{
    ESP_LOGI(TAGT, "Accelerometer: \nX: %f, Y: %f, Z: %f.\n\n", getXAccel(), getYAccel(), getZAccel());
}

/*void printImuMagnometer(void)
{
    EMP_LOGI(TAG, "Magnometer: %f.\n\n", getMagnometer());
}*/