# PINOUT MAP

- GPIO 10 - LED - Gray
- GPIO 11 - LED 2 - Gray

- GPIO 2 - Left servo - White
- GPIO 4 - Right servo - Purple

## IMU, using SPI

- GPIO 16 - IMU INT - Purple
- GPIO 15 - IMU reset - Brown
- GPIO 7 - IMU chip select - Silver
- GPIO 18 - IMU SCL - Blue
- GPIO 17 - IMU SDA - Green
- GPIO 8 - IMU DI - Yellow
- PS0 solder jumper pad - Soldered
- PS1 solder jumper pad - Soldered

## GPS, using UART

- TX - GPS TX - White
- RX - GPS RX - Yellow

## Starter wire

- GPIO 45 - Yellow with green stripe
- GPIO 39 - Yellow with green stripe

# CORE JOBS

- Core 0 - Strobe
- Core 1 - Flightpath algorithm, servo manipuation

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

- Test GPS
- Test flightpath generation
- Test control surface manipulation

# Verified working

- Strobe
- Startup
- IMU
