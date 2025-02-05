# Lab 2 Report

In this lab, we familiarized ourselves with the Inertial Measurement Unit (IMU) provided in our lab kit.

## IMU Setup

Here is a picture of the connection I made between the Artemis board and IMU using the QWIIC connect cable. Notice that the IMU is plugged into the Artemis on the I2C side, not the SPI side.

![imu_connection](images/lab2/imu_connection.jpeg)

## IMU Demo Code

In this step, I downloaded the Example code from the Arduino library and ran it. Below are two videos showing that the IMU functions correctly:

Accelerometer demo video:

* Notice that the three sensor value traces oscillate around their steady-state value as I shake the breakout board along the corresponding axis.

[![lab2_accel_demo](images/lab2/accel_demo.png)](https://youtu.be/Kcku1Q7TimA "Accelerometer Demo Video")

Gyroscope demo video:

* Notice that the gyroscope sensor values (the second group of three numbers) oscillate between approximately +250 and -250 when I rotate the breakout board along the corresponding axis. I demonstrate the IMU functionality from the first column to the third column.

[![lab2_gyro_demo](images/lab2/gyro_demo.png)](https://youtu.be/TDgxyGB8UOc "Gyroscope Demo Video")

## `AD0_VAL` Discussion

According to the example code, `AD0_VAL` is the last bit of the I2C address of the IMU. On our breakout board, the default is 1, so we set it as such:

```cpp
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1
```



