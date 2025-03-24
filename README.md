# PINOUT MAP

- GPIO 16 - Dead maybe

- GPIO 10 - LED - Gray
- GPIO 11 - LED 2 - Gray

- GPIO 2 - Left servo - White
- GPIO 4 - Right servo - Purple

## IMU, using SPI

- GPIO 6 - IMU INT - Purple
- GPIO 15 - IMU reset - Brown
- GPIO 7 - IMU chip select - Silver
- GPIO 18 - IMU SCL - Blue
- GPIO 17 - IMU SDA - Green
- GPIO 8 - IMU DI - Yellow

## GPS, using UART

- TX - GPS TX - White
- RX - GPS RX - Yellow

## Starter wire

- GPIO 45 - Yellow with green stripe
- GPIO 39 - Yellow with green stripe

# BNO085 Euler Info

## Gyro (degrees)

- type - bno08x_euler_t
- x - Roll
- y - Pitch
- z - Yaw

## Gyro Accelerometer (rad/s)

- type - bno08x_gyro_t
- x - Roll
- y - Pitch
- z - Yaw

## Linear Accelerometer (m/s^2)

- type - bno08x_accel_t
- x - x acceleration
- y - y acceleration
- z - z acceleration

# TODO:

- Test flightpath generation

# Verified working

- Strobe
- Test control surface manipulation
- Startup
- Test GPS
- Test IMU connection & output
