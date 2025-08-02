#include "ros2_mpu6050/mpu6050_node.h"

#include <chrono>
#include <memory>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

Mpu6050Node::Mpu6050Node(const std::string& name)
    : Node(name)
    , mpu6050_dev_{std::make_unique<Mpu6050>()}
{
    // Declare parameters
    this->declare_parameter<int>("gyro_fs_sel", 0);
    this->declare_parameter<int>("accel_afs_sel", 0);
    this->declare_parameter<int>("dlpf_cfg", 0);
    this->declare_parameter<int>("clock_src", 0);
    this->declare_parameter<double>("gyro_x_offset", 0.0);
    this->declare_parameter<double>("gyro_y_offset", 0.0);
    this->declare_parameter<double>("gyro_z_offset", 0.0);
    this->declare_parameter<double>("accel_x_offset", 0.0);
    this->declare_parameter<double>("accel_y_offset", 0.0);
    this->declare_parameter<double>("accel_z_offset", 0.0);
    this->declare_parameter<double>("alpha", 0.98);


    /* Assign offset values */
    gyro_x_offset_ = this->get_parameter("gyro_x_offset").as_double();
    gyro_y_offset_ = this->get_parameter("gyro_y_offset").as_double();
    gyro_z_offset_ = this->get_parameter("gyro_z_offset").as_double();
    accel_x_offset_ = this->get_parameter("accel_x_offset").as_double();
    accel_y_offset_ = this->get_parameter("accel_y_offset").as_double();
    accel_z_offset_ = this->get_parameter("accel_z_offset").as_double();
    alpha_ = this->get_parameter("alpha").as_double();


    /* Assign sensor paramaters */
    mpu6050_dev_->Mpu6050_GyroFsSel(static_cast<Mpu6050::Mpu6050_FsSel_t>(this->get_parameter("gyro_fs_sel").as_int()));
    mpu6050_dev_->Mpu6050_AccelFsSel(static_cast<Mpu6050::Mpu6050_AfsSel_t>(this->get_parameter("accel_afs_sel").as_int()));
    mpu6050_dev_->Mpu6050_DlpfConfig(static_cast<Mpu6050::Mpu6050_DlpfCfg_t>(this->get_parameter("dlpf_cfg").as_int()));
    mpu6050_dev_->Mpu6050_ClockSelect(static_cast<Mpu6050::Mpu6050_ClkSrc_t>(this->get_parameter("clock_src").as_int()));

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/mpu6050", 10);

    timer_ = this->create_wall_timer(10ms, std::bind(&Mpu6050Node::ImuPubCallback, this));

    recalibrate_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
        "recalibrate_imu", 10, std::bind(&Mpu6050Node::RecalibrateCallback, this, std::placeholders::_1));

    last_time_ = this->get_clock()->now();
}

void Mpu6050Node::RecalibrateCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
    RCLCPP_INFO(this->get_logger(), "Recalibrating IMU orientation");
    roll_ = 0.0;
    pitch_ = 0.0;
    yaw_ = 0.0;
}

void Mpu6050Node::ImuPubCallback()
{
    auto message = sensor_msgs::msg::Imu();
    rclcpp::Time current_time = this->get_clock()->now();
    message.header.stamp = current_time;
    message.header.frame_id = "base_link";
    message.linear_acceleration_covariance = {0};

    /* Read IMU data */
    Mpu6050::Mpu6050_AccelData_t AccelData;
    Mpu6050::Mpu6050_GyroData_t GyroData;

    mpu6050_dev_->Mpu6050_GetAccelData(AccelData);
    mpu6050_dev_->Mpu6050_GetGyroData(GyroData);

    double dt = (current_time - last_time_).seconds();

    double accel_x = AccelData.Accel_X - accel_x_offset_;
    double accel_y = AccelData.Accel_Y - accel_y_offset_;
    double accel_z = AccelData.Accel_Z - accel_z_offset_;

    double gyro_x = (GyroData.Gyro_X - gyro_x_offset_) * (M_PI / 180.0);
    double gyro_y = (GyroData.Gyro_Y - gyro_y_offset_) * (M_PI / 180.0);
    double gyro_z = (GyroData.Gyro_Z - gyro_z_offset_) * (M_PI / 180.0);


    message.linear_acceleration.x = accel_x;
    message.linear_acceleration.y = accel_y;
    message.linear_acceleration.z = accel_z;
    message.angular_velocity_covariance[0] = {0};
    message.angular_velocity.x = gyro_x;
    message.angular_velocity.y = gyro_y;
    message.angular_velocity.z = gyro_z;

    // Calculate roll and pitch from accelerometer data
    double roll_acc = atan2(accel_y, accel_z);
    double pitch_acc = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z));

    // Integrate gyroscope data
    roll_ += gyro_x * dt;
    pitch_ += gyro_y * dt;
    yaw_ += gyro_z * dt;

    // Complementary filter
    roll_ = alpha_ * roll_ + (1.0 - alpha_) * roll_acc;
    pitch_ = alpha_ * pitch_ + (1.0 - alpha_) * pitch_acc;

    // Convert Euler to Quaternion
    tf2::Quaternion q;
    q.setRPY(roll_, pitch_, yaw_);
    message.orientation.x = q.x();
    message.orientation.y = q.y();
    message.orientation.z = q.z();
    message.orientation.w = q.w();
    message.orientation_covariance[0] = {0};

    publisher_->publish(message);
    last_time_ = current_time;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Mpu6050Node>("mpu6050publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}