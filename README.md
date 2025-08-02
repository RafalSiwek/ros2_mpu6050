# MPU6050 - For Robot Operating System 2 (ROS2)

## Overview

This repository contains the MPU6050 package dedicated for ROS2. It contains the driver and node to allow integration with [sensor_msgs/msg/Imu Message](https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html)

## Hardware

This package has been tested with the following hardwares: \

Raspberry Pi 4B, 4GB RAM. \
MPU6050 sensor breakout board used: https://components101.com/sensors/mpu6050-module \
The Raspberry Pi 4B runs this image: https://github.com/ros-realtime/ros-realtime-rpi4-image

### Preferred Environment Setup

To run this example without issues, the following environment setup are preferred.

|                  |                          |
|------------------|--------------------------|
| Operating System | ros-realtime-rpi4-image  |
| ROS2 Version     | ROS2 Humble              |

### Package Integration

**Just clone this repository on your ros2 workspace/src.

### Sensor Calibration

There are two methods to calibrate the sensor:

**Method 1: Manual Calibration**

This method is useful for finding initial offset values.

1.  Build the package
    ```bash
    colcon build --packages-select ros2_mpu6050
    ```
2.  Ensure your RPi is connected to the MPU6050 via I2C.
3.  Place the sensor on a flat, level surface. The Z-axis should be perpendicular to the ground.
4.  Run the calibration script:
    ```bash
    ros2 run ros2_mpu6050 ros2_mpu6050_calibrate
    ```
5.  The script will output the calculated offsets:
    ```bash
    I2c communication started. .
    MPU6050 initialization successful
    
    **** Starting calibration ****
    Ensure that the mpu6050 board is positioned in a surface perpendicular to the direction gravitational accelleration
    This may take a while depending upon the no of samples, please wait. . .
    
    
    In the params.yaml file under config directory, copy the following results accordingly
    
    Gyroscope Offsets: 
    gyro_x_offset --> -1.44153
    gyro_y_offset --> 0.596412
    gyro_z_offset --> 0.618626
    
    Accelerometer Offsets: 
    accel_x_offset --> 10.601
    accel_y_offset --> -0.424632
    accel_z_offset --> -3.21498
    ```
6.  Copy these values into the `config/params.yaml` file, replacing the default `0.0` values.

**Method 2: On-Demand Calibration via ROS2 Topic**

This method allows you to re-calibrate the sensor on-the-fly without stopping the node.

1.  Launch the main MPU6050 node:
    ```bash
    ros2 launch ros2_mpu6050 ros2_mpu6050.launch.py
    ```
2.  Place the sensor in its calibration position (Z-axis perpendicular to the ground).
3.  Publish a message to the `/imu/calibrate` topic to start the calibration process.
    ```bash
    ros2 topic pub /imu/calibrate std_msgs/msg/Bool "data: true"
    ```
4.  The node will collect a number of samples to determine the new offsets. During this time, it will not publish any IMU data. Once complete, the new offsets will be printed in the node's console, and it will resume publishing corrected IMU data.

## Starting the ros2_mpu6050 node

Build the package
```bash
colcon build --packages-select ros2_mpu6050