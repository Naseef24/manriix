#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace manriix_controller
{

class Manriix4WSController : public controller_interface::ControllerInterface
{
public:
  Manriix4WSController();

  controller_interface::CallbackReturn on_init() override;
  
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum class MotionMode { STOP, LINEAR, ACKERMANN, PIVOT_TURN };
  
  // Vehicle parameters
  double wheelbase_, track_width_, wheel_radius_, steering_offset_;
  double max_linear_vel_, max_angular_vel_;
  double max_linear_accel_, max_angular_accel_;
  double max_steering_angle_, max_steering_rate_;
  double cmd_vel_timeout_, velocity_threshold_;
  
  // Angle scaling parameters
  bool enable_angle_scaling_;
  double max_scaled_angle_;
  double angle_scale_factor_;
  
  // Joint names
  std::string wheel_fl_name_, wheel_fr_name_, wheel_rl_name_, wheel_rr_name_;
  std::string steer_fl_name_, steer_fr_name_, steer_rl_name_, steer_rr_name_;
  
  // Command structure
  struct Command {
    double linear_x, linear_y, angular_z;
    rclcpp::Time stamp;
    Command() : linear_x(0.0), linear_y(0.0), angular_z(0.0), stamp(0, 0, RCL_ROS_TIME) {}
  };
  
  realtime_tools::RealtimeBuffer<Command> command_buffer_;
  Command current_command_;
  Command last_command_;
  
  // State
  MotionMode current_mode_;
  double current_steering_[4];
  double target_steering_[4];
  
  // Odometry
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> rt_odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double odom_x_, odom_y_, odom_theta_;
  std::string base_frame_id_, odom_frame_id_;
  bool enable_odom_tf_;
  double publish_rate_;
  rclcpp::Time last_update_time_, last_publish_time_;
  rclcpp::Duration publish_period_;
  
  // ROS communication
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  
  // Methods
  void cmdVelCallback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
  MotionMode determineMode(double vx, double vy, double omega);
  void computeAckermannSteering(double vx, double omega, double& fl, double& fr, double& rl, double& rr);
  void computePivotTurnSteering(double& fl, double& fr, double& rl, double& rr);
  void computeLinearSteering(double& fl, double& fr, double& rl, double& rr);
  void computeWheelVelocities(double vx, double omega, MotionMode mode, double& wfl, double& wfr, double& wrl, double& wrr);
  double interpolateAngle(double current, double target, double dt);
  void updateOdometry(const rclcpp::Time& time, const rclcpp::Duration& period, double vx, double omega);
  void publishOdometry(const rclcpp::Time& time, double vx, double omega);
  double clamp(double value, double min_val, double max_val);
  double limitAcceleration(double desired, double current, double max_accel, double dt);
};

}  // namespace manriix_controller






// #pragma once

// #include <memory>
// #include <string>
// #include <vector>

// #include "controller_interface/controller_interface.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp_lifecycle/state.hpp"
// #include "realtime_tools/realtime_buffer.hpp"
// #include "realtime_tools/realtime_publisher.hpp"
// #include "tf2_ros/transform_broadcaster.h"

// namespace manriix_controller
// {

// class Manriix4WSController : public controller_interface::ControllerInterface
// {
// public:
//   Manriix4WSController();

//   controller_interface::CallbackReturn on_init() override;
  
//   controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  
//   controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  
//   controller_interface::CallbackReturn on_configure(
//     const rclcpp_lifecycle::State & previous_state) override;
  
//   controller_interface::CallbackReturn on_activate(
//     const rclcpp_lifecycle::State & previous_state) override;
  
//   controller_interface::CallbackReturn on_deactivate(
//     const rclcpp_lifecycle::State & previous_state) override;
  
//   controller_interface::return_type update(
//     const rclcpp::Time & time, const rclcpp::Duration & period) override;

// private:
//   enum class MotionMode { STOP, LINEAR, ACKERMANN, PIVOT_TURN };
  
//   // Vehicle parameters
//   double wheelbase_, track_width_, wheel_radius_, steering_offset_;
//   double max_linear_vel_, max_angular_vel_;
//   double max_linear_accel_, max_angular_accel_;
//   double max_steering_angle_, max_steering_rate_;
//   double cmd_vel_timeout_, velocity_threshold_;
  
//   // Angle scaling parameters
//   bool enable_angle_scaling_;
//   double max_scaled_angle_;
//   double angle_scale_factor_;
  
//   // Joint names
//   std::string wheel_fl_name_, wheel_fr_name_, wheel_rl_name_, wheel_rr_name_;
//   std::string steer_fl_name_, steer_fr_name_, steer_rl_name_, steer_rr_name_;
  
//   // Command structure
//   struct Command {
//     double linear_x, linear_y, angular_z;
//     rclcpp::Time stamp;
//     Command() : linear_x(0.0), linear_y(0.0), angular_z(0.0), stamp(0, 0, RCL_ROS_TIME) {}
//   };
  
//   realtime_tools::RealtimeBuffer<Command> command_buffer_;
//   Command current_command_;
//   Command last_command_;
  
//   // State
//   MotionMode current_mode_;
//   double current_steering_[4];
//   double target_steering_[4];
  
//   // Odometry
//   std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub_;
//   std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> rt_odom_pub_;
//   std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
//   double odom_x_, odom_y_, odom_theta_;
//   std::string base_frame_id_, odom_frame_id_;
//   bool enable_odom_tf_;
//   double publish_rate_;
//   rclcpp::Time last_update_time_, last_publish_time_;
//   rclcpp::Duration publish_period_;
  
//   // ROS communication
//   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  
//   // Methods
//   void cmdVelCallback(const std::shared_ptr<geometry_msgs::msg::Twist> msg);
//   MotionMode determineMode(double vx, double vy, double omega);
//   void computeAckermannSteering(double vx, double omega, double& fl, double& fr, double& rl, double& rr);
//   void computePivotTurnSteering(double& fl, double& fr, double& rl, double& rr);
//   void computeLinearSteering(double& fl, double& fr, double& rl, double& rr);
//   void computeWheelVelocities(double vx, double omega, MotionMode mode, double& wfl, double& wfr, double& wrl, double& wrr);
//   double interpolateAngle(double current, double target, double dt);
//   void updateOdometry(const rclcpp::Time& time, const rclcpp::Duration& period, double vx, double omega);
//   void publishOdometry(const rclcpp::Time& time, double vx, double omega);
//   double clamp(double value, double min_val, double max_val);
//   double limitAcceleration(double desired, double current, double max_accel, double dt);
// };

// }  // namespace manriix_controller
