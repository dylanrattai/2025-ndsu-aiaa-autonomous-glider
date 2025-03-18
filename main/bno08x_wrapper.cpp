#include "BNO08x.hpp"
#include "esp_log.h"

static BNO08x imu;

extern "C" {
    #define TAG "BNO08x_WRAPPER"

    int bno08xInit(void)
    {
        if(!imu.initialize())
        {
            ESP_LOGE(TAG, "Failed to init IMU.");
            return 1;
        }

        // enable game rotation vector and calibrated gyro reports
        imu.rpt.rv_game.enable(100000UL);  // 100,000us == 100ms report interval
        imu.rpt.cal_gyro.enable(100000UL); // 100,000us == 100ms report interval

        return 0;
    }

    /**
     * Fetches and returns the roll (y) of the imu;
     * Returns -1000 if no data available, logs if had no new data;
     * 
     * +- 180 degrees
    */
    double getRoll(void)
    {
        if(!imu.data_available())
        {
            ESP_LOGE(TAG, "No IMU data available.");
            return -1000;
        }

        if(!imu.rpt.rv_game.has_new_data())
        {
            ESP_LOGW(TAG, "IMU fetching stale roll data.");
        }

        bno08x_euler_angle_t e = imu.rpt.rv_game.get_euler();
        return e.y;
    }

    /**
     * Fetches and returns the pitch (x) of the imu;
     * Returns -1000 if no data available, logs if had no new data;
     * 
     * +- 180 degrees
    */
    double getPitch(void)
    {
        if(!imu.data_available())
        {
            ESP_LOGE(TAG, "No IMU data available.");
            return -1000;
        }

        if(!imu.rpt.rv_game.has_new_data())
        {
            ESP_LOGW(TAG, "IMU fetching stale pitch data.");
        }

        bno08x_euler_angle_t e = imu.rpt.rv_game.get_euler();
        return e.x;
    }

    /**
     * Fetches and returns the yaw (z) of the imu;
     * Returns -1000 if no data available, logs if had no new data;
     * 
     * +- 180 degrees
    */
    double getYaw(void)
    {
        if(!imu.data_available())
        {
            ESP_LOGE(TAG, "No IMU data available.");
            return -1000;
        }

        if(!imu.rpt.rv_game.has_new_data())
        {
            ESP_LOGW(TAG, "IMU fetching stale pitch data.");
        }

        bno08x_euler_angle_t e = imu.rpt.rv_game.get_euler();
        return e.z;
    }

    /**
     * Returns the roll acceleration of the imu in rad/s.
     * Returns -1000 if no data available, logs if had no new data;
    */
    double getRollAccel(void)
    {
        if(!imu.data_available())
        {
            ESP_LOGE(TAG, "No IMU data available.");
            return -1000;
        }

        if(!imu.rpt.cal_gyro.has_new_data())
        {
            ESP_LOGW(TAG, "IMU fetching stale gyro data: roll");
        }
        
        bno08x_gyro_t g = imu.rpt.cal_gyro.get();
        return g.y;
    }

    /**
     * Returns the pitch acceleration of the imu in rad/s.
     * Returns -1000 if no data available, logs if had no new data;
    */
    double getPitchAccel(void)
    {
        if(!imu.data_available())
        {
            ESP_LOGE(TAG, "No IMU data available.");
            return -1000;
        }

        if(!imu.rpt.cal_gyro.has_new_data())
        {
            ESP_LOGW(TAG, "IMU fetching stale gyro data: pitch");
        }
        
        bno08x_gyro_t g = imu.rpt.cal_gyro.get();
        return g.x;
    }

    /**
     * Returns the yaw acceleration of the imu in rad/s.
     * Returns -1000 if no data available, logs if had no new data;
    */
    double getYawAccel(void)
    {
        if(!imu.data_available())
        {
            ESP_LOGE(TAG, "No IMU data available.");
            return -1000;
        }

        if(!imu.rpt.cal_gyro.has_new_data())
        {
            ESP_LOGW(TAG, "IMU fetching stale gyro data: roll");
        }
        
        bno08x_gyro_t g = imu.rpt.cal_gyro.get();
        return g.z;
    }

    /**
     * Returns the linear y (forwards backwards) acceleration of the IMU in m/s^2.
     * Returns -1000 if no data available, logs if had no new data;
    */
    double getYAccel(void)
    {
        if(!imu.data_available())
        {
            ESP_LOGE(TAG, "No IMU data available.");
            return -1000;
        }

        if(!imu.rpt.linear_accelerometer.has_new_data())
        {
            ESP_LOGW(TAG, "IMU fetching stale linear acceleration: y.");
        }

        bno08x_accel_t a = imu.rpt.linear_accelerometer.get();
        return a.y;
    }

    /**
     * Returns the linear z (up down) acceleration of the IMU in m/s^2.
     * Returns -1000 if no data available, logs if had no new data;
    */
    double getZAccel(void)
    {
        if(!imu.data_available())
        {
            ESP_LOGE(TAG, "No IMU data available.");
            return -1000;
        }

        if(!imu.rpt.linear_accelerometer.has_new_data())
        {
            ESP_LOGW(TAG, "IMU fetching stale linear acceleration: z.");
        }

        bno08x_accel_t a = imu.rpt.linear_accelerometer.get();
        return a.z;
    }

    /**
     * Returns the linear x (left right) acceleration of the IMU in m/s^2.
     * Returns -1000 if no data available, logs if had no new data;
    */
    double getXAccel(void)
    {
        if(!imu.data_available())
        {
            ESP_LOGE(TAG, "No IMU data available.");
            return -1000;
        }

        if(!imu.rpt.linear_accelerometer.has_new_data())
        {
            ESP_LOGW(TAG, "IMU fetching stale linear acceleration: x.");
        }

        bno08x_accel_t a = imu.rpt.linear_accelerometer.get();
        return a.x;
    }


    int getMagnometer(float* mx, float* my, float* mz)
    {
        if(!imu.data_available())
        {
            ESP_LOGE(TAG, "No IMU data available.");
            return -1;
        }

        if(!imu.rpt.cal_magnetometer.has_new_data())
        {
            ESP_LOGW(TAG, "IMU fetching stale magnometer data.");
        }

        bno08x_magf_t m = imu.rpt.cal_magnetometer.get();
        *mx = m.x;
        *my = m.y;
        *mz = m.z;

        return 0;
    }
}