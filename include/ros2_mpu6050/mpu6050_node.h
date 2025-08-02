#ifndef MPU6050DRIVER_H
#define MPU6050DRIVER_H

#include "ros2_mpu6050/mpu6050.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp" // Add this include
#include "tf2/LinearMath/Quaternion.h"

class Mpu6050Node : public rclcpp::Node {
 public:
  Mpu6050Node(const std::string& name);

 private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr calibration_subscriber_; // Add this
  std::unique_ptr<Mpu6050> mpu6050_dev_;
  rclcpp::TimerBase::SharedPtr timer_;
  void ImuPubCallback();
  void CalibrationCallback(const std_msgs::msg::Bool::SharedPtr msg); // Add this

  double gyro_x_offset_ {0.0};
  double gyro_y_offset_ {0.0};
  double gyro_z_offset_ {0.0};
  double accel_x_offset_ {0.0};
  double accel_y_offset_ {0.0};
  double accel_z_offset_ {0.0};

  // Complementary filter variables
  double pitch_ {0.0};
  double roll_ {0.0};
  rclcpp::Time last_time_;
  tf2::Quaternion orientation_quaternion_;

  // Calibration variables
  bool in_calibration_mode_ {false};
  int calibration_samples_ {1000};
  int current_calibration_sample_ {0};
  double calib_gyro_x_sum_ {0.0};
  double calib_gyro_y_sum_ {0.0};
  double calib_gyro_z_sum_ {0.0};
  double calib_accel_x_sum_ {0.0};
  double calib_accel_y_sum_ {0.0};
  double calib_accel_z_sum_ {0.0};
};

#endif  // MPU6050DRIVER_H