#include "ros2_mpu6050/mpu6050_node.h"

#include <chrono>
#include <memory>
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

Mpu6050Node::Mpu6050Node(const std::string& name)
    : Node(name)
    , mpu6050_dev_{std::make_unique<Mpu6050>()}
{
    // ... (parameter declarations are the same)

    // Assign offset values from parameters as initial defaults
    gyro_x_offset_ = this->get_parameter("gyro_x_offset").as_double();
    gyro_y_offset_ = this->get_parameter("gyro_y_offset").as_double();
    gyro_z_offset_ = this->get_parameter("gyro_z_offset").as_double();
    accel_x_offset_ = this->get_parameter("accel_x_offset").as_double();
    accel_y_offset_ = this->get_parameter("accel_y_offset").as_double();
    accel_z_offset_ = this->get_parameter("accel_z_offset").as_double();

    // ... (sensor parameter assignments are the same)

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/mpu6050", 10);
    calibration_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "imu/calibrate", 10, std::bind(&Mpu6050Node::CalibrationCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(10ms, std::bind(&Mpu6050Node::ImuPubCallback, this));
    last_time_ = this->get_clock()->now();
    orientation_quaternion_.setRPY(0, 0, 0);
}

void Mpu6050Node::CalibrationCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "Starting IMU calibration. Do not move the sensor.");
        in_calibration_mode_ = true;
        current_calibration_sample_ = 0;
        calib_gyro_x_sum_ = 0.0;
        calib_gyro_y_sum_ = 0.0;
        calib_gyro_z_sum_ = 0.0;
        calib_accel_x_sum_ = 0.0;
        calib_accel_y_sum_ = 0.0;
        calib_accel_z_sum_ = 0.0;
    }
}

void Mpu6050Node::ImuPubCallback()
{
    Mpu6050::Mpu6050_AccelData_t AccelData;
    Mpu6050::Mpu6050_GyroData_t GyroData;

    mpu6050_dev_->Mpu6050_GetAccelData(AccelData);
    mpu6050_dev_->Mpu6050_GetGyroData(GyroData);

    if (in_calibration_mode_) {
        calib_gyro_x_sum_ += GyroData.Gyro_X;
        calib_gyro_y_sum_ += GyroData.Gyro_Y;
        calib_gyro_z_sum_ += GyroData.Gyro_Z;
        calib_accel_x_sum_ += AccelData.Accel_X;
        calib_accel_y_sum_ += AccelData.Accel_Y;
        calib_accel_z_sum_ += AccelData.Accel_Z;
        current_calibration_sample_++;

        if (current_calibration_sample_ >= calibration_samples_) {
            in_calibration_mode_ = false;
            gyro_x_offset_ = calib_gyro_x_sum_ / calibration_samples_;
            gyro_y_offset_ = calib_gyro_y_sum_ / calibration_samples_;
            gyro_z_offset_ = calib_gyro_z_sum_ / calibration_samples_;
            accel_x_offset_ = calib_accel_x_sum_ / calibration_samples_;
            accel_y_offset_ = calib_accel_y_sum_ / calibration_samples_;
            // Z-axis is perpendicular to gravity, so its reading should be 0.
            accel_z_offset_ = calib_accel_z_sum_ / calibration_samples_;

            RCLCPP_INFO(this->get_logger(), "IMU calibration finished.");
            RCLCPP_INFO(this->get_logger(), "New Gyro Offsets: x=%.4f, y=%.4f, z=%.4f", gyro_x_offset_, gyro_y_offset_, gyro_z_offset_);
            RCLCPP_INFO(this->get_logger(), "New Accel Offsets: x=%.4f, y=%.4f, z=%.4f", accel_x_offset_, accel_y_offset_, accel_z_offset_);
        }
        return; // Skip publishing during calibration
    }

    auto message = sensor_msgs::msg::Imu();
    rclcpp::Time current_time = this->get_clock()->now();
    message.header.stamp = current_time;
    message.header.frame_id = "base_link";

    message.linear_acceleration.x = AccelData.Accel_X - accel_x_offset_;
    message.linear_acceleration.y = AccelData.Accel_Y - accel_y_offset_;
    message.linear_acceleration.z = AccelData.Accel_Z - accel_z_offset_;
    message.linear_acceleration_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};

    message.angular_velocity.x = (GyroData.Gyro_X - gyro_x_offset_) * (M_PI / 180.0);
    message.angular_velocity.y = (GyroData.Gyro_Y - gyro_y_offset_) * (M_PI / 180.0);
    message.angular_velocity.z = (GyroData.Gyro_Z - gyro_z_offset_) * (M_PI / 180.0);
    message.angular_velocity_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};

    // Complementary filter for orientation
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    double pitch_acc = atan2(message.linear_acceleration.y, message.linear_acceleration.z) * 180 / M_PI;
    double roll_acc = atan2(-message.linear_acceleration.x, sqrt(pow(message.linear_acceleration.y, 2) + pow(message.linear_acceleration.z, 2))) * 180 / M_PI;

    double alpha = 0.98;
    pitch_ = alpha * (pitch_ + message.angular_velocity.y * dt * (180/M_PI)) + (1 - alpha) * pitch_acc;
    roll_ = alpha * (roll_ + message.angular_velocity.x * dt * (180/M_PI)) + (1 - alpha) * roll_acc;

    orientation_quaternion_.setRPY(roll_ * (M_PI/180), pitch_ * (M_PI/180), 0);

    message.orientation.x = orientation_quaternion_.x();
    message.orientation.y = orientation_quaternion_.y();
    message.orientation.z = orientation_quaternion_.z();
    message.orientation.w = orientation_quaternion_.w();
    message.orientation_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};

    publisher_->publish(message);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Mpu6050Node>("mpu6050_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}