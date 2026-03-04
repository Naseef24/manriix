#include "manriix_4ws_controller/manriix_4ws_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace manriix_controller
{

Manriix4WSController::Manriix4WSController()
: controller_interface::ControllerInterface(),
  wheelbase_(0.44),
  track_width_(0.635),
  wheel_radius_(0.101),
  steering_offset_(0.085),
  max_linear_vel_(1.0),
  max_angular_vel_(1.0),
  max_linear_accel_(1.0),
  max_angular_accel_(1.0),
  max_steering_angle_(0.785),
  max_steering_rate_(2.0),
  cmd_vel_timeout_(0.5),
  velocity_threshold_(0.01),
  enable_angle_scaling_(true),
  max_scaled_angle_(0.15),
  angle_scale_factor_(0.7),
  current_mode_(MotionMode::STOP),
  odom_x_(0.0),
  odom_y_(0.0),
  odom_theta_(0.0),
  base_frame_id_("base_footprint"),
  odom_frame_id_("odom"),
  enable_odom_tf_(true),
  publish_rate_(50.0),
  last_update_time_(0, 0, RCL_ROS_TIME),
  last_publish_time_(0, 0, RCL_ROS_TIME),
  publish_period_(rclcpp::Duration::from_seconds(1.0 / 50.0))
{
  for (int i = 0; i < 4; ++i) {
    current_steering_[i] = 0.0;
    target_steering_[i] = 0.0;
  }
}

controller_interface::CallbackReturn Manriix4WSController::on_init()
{
  try {
    auto_declare<std::string>("front_left_wheel_joint", "wheel_front_left_joint");
    auto_declare<std::string>("front_right_wheel_joint", "wheel_front_right_joint");
    auto_declare<std::string>("rear_left_wheel_joint", "wheel_rear_left_joint");
    auto_declare<std::string>("rear_right_wheel_joint", "wheel_rear_right_joint");
    auto_declare<std::string>("front_left_steering_joint", "steering_front_left_joint");
    auto_declare<std::string>("front_right_steering_joint", "steering_front_right_joint");
    auto_declare<std::string>("rear_left_steering_joint", "steering_rear_left_joint");
    auto_declare<std::string>("rear_right_steering_joint", "steering_rear_right_joint");
    
    auto_declare<double>("wheelbase", wheelbase_);
    auto_declare<double>("track_width", track_width_);
    auto_declare<double>("wheel_radius", wheel_radius_);
    auto_declare<double>("steering_offset", steering_offset_);
    auto_declare<double>("max_linear_velocity", max_linear_vel_);
    auto_declare<double>("max_angular_velocity", max_angular_vel_);
    auto_declare<double>("max_linear_acceleration", max_linear_accel_);
    auto_declare<double>("max_angular_acceleration", max_angular_accel_);
    auto_declare<double>("max_steering_angle", max_steering_angle_);
    auto_declare<double>("max_steering_rate", max_steering_rate_);
    auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_);
    auto_declare<double>("velocity_threshold", velocity_threshold_);
    auto_declare<bool>("enable_angle_scaling", enable_angle_scaling_);
    auto_declare<double>("max_scaled_angle", max_scaled_angle_);
    auto_declare<double>("angle_scale_factor", angle_scale_factor_);
    auto_declare<std::string>("base_frame_id", base_frame_id_);
    auto_declare<std::string>("odom_frame_id", odom_frame_id_);
    auto_declare<bool>("enable_odom_tf", enable_odom_tf_);
    auto_declare<double>("publish_rate", publish_rate_);
    
    RCLCPP_INFO(get_node()->get_logger(), "Manriix 4WS Controller initialized");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
Manriix4WSController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  config.names.push_back(wheel_fl_name_ + "/velocity");
  config.names.push_back(wheel_fr_name_ + "/velocity");
  config.names.push_back(wheel_rl_name_ + "/velocity");
  config.names.push_back(wheel_rr_name_ + "/velocity");
  config.names.push_back(steer_fl_name_ + "/position");
  config.names.push_back(steer_fr_name_ + "/position");
  config.names.push_back(steer_rl_name_ + "/position");
  config.names.push_back(steer_rr_name_ + "/position");
  
  return config;
}

controller_interface::InterfaceConfiguration
Manriix4WSController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  config.names.push_back(wheel_fl_name_ + "/velocity");
  config.names.push_back(wheel_fr_name_ + "/velocity");
  config.names.push_back(wheel_rl_name_ + "/velocity");
  config.names.push_back(wheel_rr_name_ + "/velocity");
  config.names.push_back(steer_fl_name_ + "/position");
  config.names.push_back(steer_fr_name_ + "/position");
  config.names.push_back(steer_rl_name_ + "/position");
  config.names.push_back(steer_rr_name_ + "/position");
  
  return config;
}

controller_interface::CallbackReturn Manriix4WSController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Configuring Manriix 4WS Controller");
  
  wheel_fl_name_ = get_node()->get_parameter("front_left_wheel_joint").as_string();
  wheel_fr_name_ = get_node()->get_parameter("front_right_wheel_joint").as_string();
  wheel_rl_name_ = get_node()->get_parameter("rear_left_wheel_joint").as_string();
  wheel_rr_name_ = get_node()->get_parameter("rear_right_wheel_joint").as_string();
  steer_fl_name_ = get_node()->get_parameter("front_left_steering_joint").as_string();
  steer_fr_name_ = get_node()->get_parameter("front_right_steering_joint").as_string();
  steer_rl_name_ = get_node()->get_parameter("rear_left_steering_joint").as_string();
  steer_rr_name_ = get_node()->get_parameter("rear_right_steering_joint").as_string();
  
  wheelbase_ = get_node()->get_parameter("wheelbase").as_double();
  track_width_ = get_node()->get_parameter("track_width").as_double();
  wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
  steering_offset_ = get_node()->get_parameter("steering_offset").as_double();
  max_linear_vel_ = get_node()->get_parameter("max_linear_velocity").as_double();
  max_angular_vel_ = get_node()->get_parameter("max_angular_velocity").as_double();
  max_linear_accel_ = get_node()->get_parameter("max_linear_acceleration").as_double();
  max_angular_accel_ = get_node()->get_parameter("max_angular_acceleration").as_double();
  max_steering_angle_ = get_node()->get_parameter("max_steering_angle").as_double();
  max_steering_rate_ = get_node()->get_parameter("max_steering_rate").as_double();
  cmd_vel_timeout_ = get_node()->get_parameter("cmd_vel_timeout").as_double();
  velocity_threshold_ = get_node()->get_parameter("velocity_threshold").as_double();
  enable_angle_scaling_ = get_node()->get_parameter("enable_angle_scaling").as_bool();
  max_scaled_angle_ = get_node()->get_parameter("max_scaled_angle").as_double();
  angle_scale_factor_ = get_node()->get_parameter("angle_scale_factor").as_double();
  base_frame_id_ = get_node()->get_parameter("base_frame_id").as_string();
  odom_frame_id_ = get_node()->get_parameter("odom_frame_id").as_string();
  enable_odom_tf_ = get_node()->get_parameter("enable_odom_tf").as_bool();
  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
  
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
  
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "~/cmd_vel", 10,
    std::bind(&Manriix4WSController::cmdVelCallback, this, std::placeholders::_1));
  
  odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    "~/odom", rclcpp::SystemDefaultsQoS());
  
  rt_odom_pub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_pub_);
  
  if (enable_odom_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());
  }
  
  Command empty_cmd;
  command_buffer_.writeFromNonRT(empty_cmd);
  
  RCLCPP_INFO(get_node()->get_logger(), "Manriix 4WS Controller configured");
  RCLCPP_INFO(get_node()->get_logger(), "Angle scaling: %s (max: %.2f deg, factor: %.2f)",
    enable_angle_scaling_ ? "ENABLED" : "DISABLED",
    max_scaled_angle_ * 180.0 / M_PI, angle_scale_factor_);
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Manriix4WSController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Activating Manriix 4WS Controller");
  
  for (size_t i = 0; i < 4; ++i) {
    current_steering_[i] = state_interfaces_[i + 4].get_value();
  }
  
  command_interfaces_[0].set_value(0.0);
  command_interfaces_[1].set_value(0.0);
  command_interfaces_[2].set_value(0.0);
  command_interfaces_[3].set_value(0.0);
  
  current_command_ = Command();
  last_command_ = Command();
  
  last_update_time_ = get_node()->now();
  last_publish_time_ = get_node()->now();
  
  RCLCPP_INFO(get_node()->get_logger(), "Manriix 4WS Controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Manriix4WSController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivating Manriix 4WS Controller");
  
  for (size_t i = 0; i < 4; ++i) {
    command_interfaces_[i].set_value(0.0);
  }
  
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Manriix4WSController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Read latest command from buffer
  current_command_ = *(command_buffer_.readFromRT());
  
  // Check for command timeout
  double cmd_age = (time - current_command_.stamp).seconds();
  if (cmd_age > cmd_vel_timeout_) {
    current_command_.linear_x = 0.0;
    current_command_.linear_y = 0.0;
    current_command_.angular_z = 0.0;
  }
  
  // Apply acceleration limits
  double dt = period.seconds();
  double limited_vx = limitAcceleration(current_command_.linear_x, 
                                        last_command_.linear_x,
                                        max_linear_accel_, dt);
  double limited_omega = limitAcceleration(current_command_.angular_z,
                                           last_command_.angular_z,
                                           max_angular_accel_, dt);
  
  // Determine motion mode
  current_mode_ = determineMode(limited_vx, current_command_.linear_y, limited_omega);
  
  // Compute target steering angles
  switch (current_mode_) {
    case MotionMode::ACKERMANN:
      computeAckermannSteering(limited_vx, limited_omega,
                               target_steering_[0], target_steering_[1],
                               target_steering_[2], target_steering_[3]);
      break;
    case MotionMode::PIVOT_TURN:
      computePivotTurnSteering(target_steering_[0], target_steering_[1],
                               target_steering_[2], target_steering_[3]);
      break;
    case MotionMode::LINEAR:
      computeLinearSteering(target_steering_[0], target_steering_[1],
                            target_steering_[2], target_steering_[3]);
      break;
    case MotionMode::STOP:
      for (int i = 0; i < 4; ++i) {
        target_steering_[i] = 0.0;
      }
      break;
  }
  
  // Smoothly interpolate steering angles
  for (int i = 0; i < 4; ++i) {
    current_steering_[i] = interpolateAngle(current_steering_[i], 
                                             target_steering_[i], dt);
  }
  
  // Set steering commands
  command_interfaces_[4].set_value(current_steering_[0]);
  command_interfaces_[5].set_value(current_steering_[1]);
  command_interfaces_[6].set_value(current_steering_[2]);
  command_interfaces_[7].set_value(current_steering_[3]);
  
  // // Compute wheel velocities
  // double wfl, wfr, wrl, wrr;
  // computeWheelVelocities(limited_vx, limited_omega, current_mode_, wfl, wfr, wrl, wrr);
  
  // // Set wheel velocity commands
  // command_interfaces_[0].set_value(wfl);
  // command_interfaces_[1].set_value(wfr);
  // command_interfaces_[2].set_value(wrl);
  // command_interfaces_[3].set_value(wrr);
  
  // // Update odometry (always use commanded velocities)
  // // updateOdometry(time, period, limited_vx, limited_omega);
  // // Compute actual omega from physical steering angles (not commanded omega)
  // // This accounts for angle scaling and steering rate limiting
  // double actual_omega = limited_omega;  // default for PIVOT_TURN, LINEAR, STOP
  // if (current_mode_ == MotionMode::ACKERMANN && std::abs(limited_vx) > 0.001) {
  //   double avg_front_steer = (current_steering_[0] + current_steering_[1]) / 2.0;
  //   actual_omega = limited_vx * std::tan(avg_front_steer) / (wheelbase_ / 2.0);
  // }
  
  // // Update odometry with actual (not commanded) turn rate
  // updateOdometry(time, period, limited_vx, actual_omega);

// Compute actual omega from physical steering angles (not commanded omega)
  // This accounts for angle scaling and steering rate limiting
  // double actual_omega = limited_omega;  // default for PIVOT_TURN, LINEAR, STOP
  // if (current_mode_ == MotionMode::ACKERMANN && std::abs(limited_vx) > 0.001) {
  //   double avg_front_steer = (current_steering_[0] + current_steering_[1]) / 2.0;
  //   actual_omega = limited_vx * std::tan(avg_front_steer) / (wheelbase_ / 2.0);
  // }

  // --- FIXED: Proper geometric back-computation of actual omega ---
  double actual_omega = limited_omega;  // default for PIVOT_TURN, LINEAR, STOP
  
  if (current_mode_ == MotionMode::ACKERMANN && std::abs(limited_vx) > 0.001) {
    double steering_track = track_width_ - 2.0 * steering_offset_;
    
    // Back-compute omega from FL angle (exact inverse of Ackermann formula)
    double tan_fl = std::tan(current_steering_[0]);
    double tan_fr = std::tan(current_steering_[1]);
    
    double omega_from_fl = 0.0;
    double omega_from_fr = 0.0;
    bool fl_valid = std::abs(wheelbase_ + steering_track * tan_fl) > 0.001;
    bool fr_valid = std::abs(wheelbase_ - steering_track * tan_fr) > 0.001;
    
    if (fl_valid) {
      omega_from_fl = 2.0 * limited_vx * tan_fl / (wheelbase_ + steering_track * tan_fl);
    }
    if (fr_valid) {
      omega_from_fr = 2.0 * limited_vx * tan_fr / (wheelbase_ - steering_track * tan_fr);
    }
    
    // Average both for robustness during steering transitions
    if (fl_valid && fr_valid) {
      actual_omega = (omega_from_fl + omega_from_fr) / 2.0;
    } else if (fl_valid) {
      actual_omega = omega_from_fl;
    } else if (fr_valid) {
      actual_omega = omega_from_fr;
    }
    // else: keep limited_omega as fallback
  }  
  
  // Compute wheel velocities using actual omega (matches physical steering geometry)
  double wfl, wfr, wrl, wrr;
  computeWheelVelocities(limited_vx, actual_omega, current_mode_, wfl, wfr, wrl, wrr);
  
  // Set wheel velocity commands
  command_interfaces_[0].set_value(wfl);
  command_interfaces_[1].set_value(wfr);
  command_interfaces_[2].set_value(wrl);
  command_interfaces_[3].set_value(wrr);
  
  // Update odometry with actual (not commanded) turn rate
  updateOdometry(time, period, limited_vx, actual_omega);
    
  // Publish odometry
  if (time - last_publish_time_ >= publish_period_) {
    publishOdometry(time, limited_vx, actual_omega);
    last_publish_time_ = time;
  }
  
  // Save command for next iteration
  last_command_.linear_x = limited_vx;
  last_command_.angular_z = limited_omega;
  // last_command_.angular_z = actual_omega;  // was: limited_omega
  last_update_time_ = time;
  
  return controller_interface::return_type::OK;
}

void Manriix4WSController::cmdVelCallback(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
{
  Command cmd;
  cmd.linear_x = clamp(msg->linear.x, -max_linear_vel_, max_linear_vel_);
  cmd.linear_y = msg->linear.y;
  cmd.angular_z = clamp(msg->angular.z, -max_angular_vel_, max_angular_vel_);
  cmd.stamp = get_node()->now();
  
  command_buffer_.writeFromNonRT(cmd);
}

Manriix4WSController::MotionMode Manriix4WSController::determineMode(
  double vx, double vy, double omega)
{
  bool has_linear = (std::abs(vx) > velocity_threshold_ || std::abs(vy) > velocity_threshold_);
  bool has_angular = std::abs(omega) > velocity_threshold_;
  
  if (!has_linear && !has_angular) {
    return MotionMode::STOP;
  } else if (has_linear && !has_angular) {
    return MotionMode::LINEAR;
  } else if (has_linear && has_angular) {
    return MotionMode::ACKERMANN;
  } else {
    return MotionMode::PIVOT_TURN;
  }
}

void Manriix4WSController::computeAckermannSteering(
  double vx, double omega, double& fl, double& fr, double& rl, double& rr)
{
  if (std::abs(omega) < 1e-6) {
    fl = fr = rl = rr = 0.0;
    return;
  }
  
  double steering_track = track_width_ - 2.0*steering_offset_;
  double denominator_left = 2.0*vx - omega*steering_track;
  double denominator_right = 2.0*vx + omega*steering_track;
  
  double raw_fl, raw_fr;
  
  if (std::abs(denominator_left) > 0.001 && std::abs(denominator_right) > 0.001) {
    raw_fl = std::atan(omega * wheelbase_ / denominator_left);
    raw_fr = std::atan(omega * wheelbase_ / denominator_right);
  } else {
    raw_fl = std::copysign(max_steering_angle_, omega);
    raw_fr = std::copysign(max_steering_angle_, omega);
  }
  
  // if (enable_angle_scaling_) {
  //   fl = std::copysign(
  //       std::min(max_scaled_angle_, std::abs(raw_fl) * angle_scale_factor_ * 
  //       (1.0 - 0.4 * std::abs(raw_fl) / M_PI_2)), 
  //       raw_fl);
        
  //   fr = std::copysign(
  //       std::min(max_scaled_angle_, std::abs(raw_fr) * angle_scale_factor_ * 
  //       (1.0 - 0.4 * std::abs(raw_fr) / M_PI_2)), 
  //       raw_fr);
  // } else {
  //   fl = raw_fl;
  //   fr = raw_fr;
  // }

  // --- FIXED: Uniform scaling preserves Ackermann ratio ---
  if (enable_angle_scaling_) {
    double max_raw = std::max(std::abs(raw_fl), std::abs(raw_fr));
    
    if (max_raw > 1e-6) {
      // Compute scale factor based on the larger angle (inner wheel)
      // This ensures both angles are scaled identically
      double scale = 1.0;
      
      if (max_raw > max_scaled_angle_) {
        // Hard limit: clamp so largest angle = max_scaled_angle
        scale = max_scaled_angle_ / max_raw;
      } else {
        // Soft compression: apply angle_scale_factor uniformly
        scale = angle_scale_factor_;
      }
      
      fl = raw_fl * scale;
      fr = raw_fr * scale;
    } else {
      fl = raw_fl;
      fr = raw_fr;
    }
  } else {
    fl = raw_fl;
    fr = raw_fr;
  }

  rl = -fl;
  rr = -fr;
  
  fl = clamp(fl, -max_steering_angle_, max_steering_angle_);
  fr = clamp(fr, -max_steering_angle_, max_steering_angle_);
  rl = clamp(rl, -max_steering_angle_, max_steering_angle_);
  rr = clamp(rr, -max_steering_angle_, max_steering_angle_);
}

void Manriix4WSController::computePivotTurnSteering(double& fl, double& fr, double& rl, double& rr)
{
  double angle = std::atan2(wheelbase_/2.0, track_width_/2.0);
  fl = -angle;
  fr = angle;
  rl = angle;
  rr = -angle;
}

void Manriix4WSController::computeLinearSteering(double& fl, double& fr, double& rl, double& rr)
{
  fl = fr = rl = rr = 0.0;
}

void Manriix4WSController::computeWheelVelocities(
  double vx, double omega, MotionMode mode, double& wfl, double& wfr, double& wrl, double& wrr)
{
  switch (mode) {
    case MotionMode::LINEAR:
      wfl = wfr = wrl = wrr = vx / wheel_radius_;
      break;
      
    case MotionMode::ACKERMANN: {
      if (std::abs(omega) < 1e-6) {
        wfl = wfr = wrl = wrr = vx / wheel_radius_;
        break;
      }
      
      double R = vx / omega;
      double R_fl = std::sqrt(std::pow(R - track_width_/2.0, 2) + std::pow(wheelbase_/2.0, 2));
      double R_fr = std::sqrt(std::pow(R + track_width_/2.0, 2) + std::pow(wheelbase_/2.0, 2));
      double R_rl = std::sqrt(std::pow(R - track_width_/2.0, 2) + std::pow(wheelbase_/2.0, 2));
      double R_rr = std::sqrt(std::pow(R + track_width_/2.0, 2) + std::pow(wheelbase_/2.0, 2));
      
      double sign = (vx >= 0) ? 1.0 : -1.0;
      wfl = sign * std::abs(omega * R_fl) / wheel_radius_;
      wfr = sign * std::abs(omega * R_fr) / wheel_radius_;
      wrl = sign * std::abs(omega * R_rl) / wheel_radius_;
      wrr = sign * std::abs(omega * R_rr) / wheel_radius_;
      break;
    }
    
    case MotionMode::PIVOT_TURN: {
      double d_wheel = std::sqrt(std::pow(wheelbase_/2.0, 2) + std::pow(track_width_/2.0, 2));
      double v_wheel = (omega * d_wheel) / wheel_radius_;
      wfl = -v_wheel;
      wfr = v_wheel;
      wrl = -v_wheel;
      wrr = v_wheel;
      break;
    }
    
    case MotionMode::STOP:
    default:
      wfl = wfr = wrl = wrr = 0.0;
      break;
  }
  
  wfl = clamp(wfl, -max_linear_vel_/wheel_radius_, max_linear_vel_/wheel_radius_);
  wfr = clamp(wfr, -max_linear_vel_/wheel_radius_, max_linear_vel_/wheel_radius_);
  wrl = clamp(wrl, -max_linear_vel_/wheel_radius_, max_linear_vel_/wheel_radius_);
  wrr = clamp(wrr, -max_linear_vel_/wheel_radius_, max_linear_vel_/wheel_radius_);
}

double Manriix4WSController::interpolateAngle(double current, double target, double dt)
{
  double diff = target - current;
  while (diff > M_PI) diff -= 2.0 * M_PI;
  while (diff < -M_PI) diff += 2.0 * M_PI;
  
  double max_change = max_steering_rate_ * dt;
  if (std::abs(diff) <= max_change) return target;
  
  return current + std::copysign(max_change, diff);
}

void Manriix4WSController::updateOdometry(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& period, double vx, double omega)
{
  double dt = period.seconds();
  if (dt < 1e-6) return;
  
  double delta_theta = omega * dt;
  double delta_x = vx * std::cos(odom_theta_ + delta_theta/2.0) * dt;
  double delta_y = vx * std::sin(odom_theta_ + delta_theta/2.0) * dt;
  
  odom_x_ += delta_x;
  odom_y_ += delta_y;
  odom_theta_ += delta_theta;
  
  while (odom_theta_ > M_PI) odom_theta_ -= 2.0 * M_PI;
  while (odom_theta_ < -M_PI) odom_theta_ += 2.0 * M_PI;
}

void Manriix4WSController::publishOdometry(const rclcpp::Time& time, double vx, double omega)
{
  if (rt_odom_pub_->trylock()) {
    auto & odom_msg = rt_odom_pub_->msg_;
    odom_msg.header.stamp = time;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;
    
    odom_msg.pose.pose.position.x = odom_x_;
    odom_msg.pose.pose.position.y = odom_y_;
    odom_msg.pose.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom_theta_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = omega;
    
    for (size_t i = 0; i < 36; ++i) {
      odom_msg.pose.covariance[i] = 0.0;
      odom_msg.twist.covariance[i] = 0.0;
    }
    odom_msg.pose.covariance[0] = 0.05;
    odom_msg.pose.covariance[7] = 0.05;
    odom_msg.pose.covariance[35] = 0.1;
    odom_msg.twist.covariance[0] = 0.02;
    odom_msg.twist.covariance[35] = 0.05;
    
    rt_odom_pub_->unlockAndPublish();
  }
  
  if (enable_odom_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = time;
    odom_tf.header.frame_id = odom_frame_id_;
    odom_tf.child_frame_id = base_frame_id_;
    odom_tf.transform.translation.x = odom_x_;
    odom_tf.transform.translation.y = odom_y_;
    odom_tf.transform.translation.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom_theta_);
    odom_tf.transform.rotation.x = q.x();
    odom_tf.transform.rotation.y = q.y();
    odom_tf.transform.rotation.z = q.z();
    odom_tf.transform.rotation.w = q.w();
    
    tf_broadcaster_->sendTransform(odom_tf);
  }
}

double Manriix4WSController::clamp(double value, double min_val, double max_val)
{
  return std::max(min_val, std::min(value, max_val));
}

double Manriix4WSController::limitAcceleration(
  double desired, double current, double max_accel, double dt)
{
  double diff = desired - current;
  double max_change = max_accel * dt;
  if (std::abs(diff) <= max_change) return desired;
  return current + std::copysign(max_change, diff);
}

}  // namespace manriix_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  manriix_controller::Manriix4WSController,
  controller_interface::ControllerInterface)


// #include "manriix_4ws_controller/manriix_4ws_controller.hpp"

// #include <algorithm>
// #include <cmath>
// #include <memory>
// #include <string>
// #include <vector>

// #include "hardware_interface/types/hardware_interface_type_values.hpp"
// #include "lifecycle_msgs/msg/state.hpp"
// #include "rclcpp/logging.hpp"
// #include "tf2/LinearMath/Quaternion.h"
// #include "geometry_msgs/msg/transform_stamped.hpp"

// namespace manriix_controller
// {

// Manriix4WSController::Manriix4WSController()
// : controller_interface::ControllerInterface(),
//   wheelbase_(0.349),
//   track_width_(0.538),
//   wheel_radius_(0.101),
//   steering_offset_(0.085),
//   max_linear_vel_(1.667),
//   max_angular_vel_(3.14),
//   max_linear_accel_(1.0),
//   max_angular_accel_(1.0),
//   max_steering_angle_(0.785),
//   max_steering_rate_(2.0),
//   cmd_vel_timeout_(0.5),
//   velocity_threshold_(0.01),
//   enable_angle_scaling_(true),
//   max_scaled_angle_(0.15),
//   angle_scale_factor_(0.7),
//   current_mode_(MotionMode::STOP),
//   odom_x_(0.0),
//   odom_y_(0.0),
//   odom_theta_(0.0),
//   base_frame_id_("base_footprint"),
//   odom_frame_id_("odom"),
//   enable_odom_tf_(true),
//   publish_rate_(50.0),
//   last_update_time_(0, 0, RCL_ROS_TIME),
//   last_publish_time_(0, 0, RCL_ROS_TIME),
//   publish_period_(rclcpp::Duration::from_seconds(1.0 / 50.0))
// {
//   for (int i = 0; i < 4; ++i) {
//     current_steering_[i] = 0.0;
//     target_steering_[i] = 0.0;
//   }
// }

// controller_interface::CallbackReturn Manriix4WSController::on_init()
// {
//   try {
//     auto_declare<std::string>("front_left_wheel_joint", "wheel_front_left_joint");
//     auto_declare<std::string>("front_right_wheel_joint", "wheel_front_right_joint");
//     auto_declare<std::string>("rear_left_wheel_joint", "wheel_rear_left_joint");
//     auto_declare<std::string>("rear_right_wheel_joint", "wheel_rear_right_joint");
//     auto_declare<std::string>("front_left_steering_joint", "steering_front_left_joint");
//     auto_declare<std::string>("front_right_steering_joint", "steering_front_right_joint");
//     auto_declare<std::string>("rear_left_steering_joint", "steering_rear_left_joint");
//     auto_declare<std::string>("rear_right_steering_joint", "steering_rear_right_joint");
    
//     auto_declare<double>("wheelbase", wheelbase_);
//     auto_declare<double>("track_width", track_width_);
//     auto_declare<double>("wheel_radius", wheel_radius_);
//     auto_declare<double>("steering_offset", steering_offset_);
//     auto_declare<double>("max_linear_velocity", max_linear_vel_);
//     auto_declare<double>("max_angular_velocity", max_angular_vel_);
//     auto_declare<double>("max_linear_acceleration", max_linear_accel_);
//     auto_declare<double>("max_angular_acceleration", max_angular_accel_);
//     auto_declare<double>("max_steering_angle", max_steering_angle_);
//     auto_declare<double>("max_steering_rate", max_steering_rate_);
//     auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_);
//     auto_declare<double>("velocity_threshold", velocity_threshold_);
//     auto_declare<bool>("enable_angle_scaling", enable_angle_scaling_);
//     auto_declare<double>("max_scaled_angle", max_scaled_angle_);
//     auto_declare<double>("angle_scale_factor", angle_scale_factor_);
//     auto_declare<std::string>("base_frame_id", base_frame_id_);
//     auto_declare<std::string>("odom_frame_id", odom_frame_id_);
//     auto_declare<bool>("enable_odom_tf", enable_odom_tf_);
//     auto_declare<double>("publish_rate", publish_rate_);
    
//     RCLCPP_INFO(get_node()->get_logger(), "Manriix 4WS Controller initialized");
//   } catch (const std::exception & e) {
//     RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
//     return controller_interface::CallbackReturn::ERROR;
//   }
//   return controller_interface::CallbackReturn::SUCCESS;
// }

// controller_interface::InterfaceConfiguration
// Manriix4WSController::command_interface_configuration() const
// {
//   controller_interface::InterfaceConfiguration config;
//   config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
//   config.names.push_back(wheel_fl_name_ + "/velocity");
//   config.names.push_back(wheel_fr_name_ + "/velocity");
//   config.names.push_back(wheel_rl_name_ + "/velocity");
//   config.names.push_back(wheel_rr_name_ + "/velocity");
//   config.names.push_back(steer_fl_name_ + "/position");
//   config.names.push_back(steer_fr_name_ + "/position");
//   config.names.push_back(steer_rl_name_ + "/position");
//   config.names.push_back(steer_rr_name_ + "/position");
  
//   return config;
// }

// controller_interface::InterfaceConfiguration
// Manriix4WSController::state_interface_configuration() const
// {
//   controller_interface::InterfaceConfiguration config;
//   config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
//   config.names.push_back(wheel_fl_name_ + "/velocity");
//   config.names.push_back(wheel_fr_name_ + "/velocity");
//   config.names.push_back(wheel_rl_name_ + "/velocity");
//   config.names.push_back(wheel_rr_name_ + "/velocity");
//   config.names.push_back(steer_fl_name_ + "/position");
//   config.names.push_back(steer_fr_name_ + "/position");
//   config.names.push_back(steer_rl_name_ + "/position");
//   config.names.push_back(steer_rr_name_ + "/position");
  
//   return config;
// }

// controller_interface::CallbackReturn Manriix4WSController::on_configure(
//   const rclcpp_lifecycle::State & /*previous_state*/)
// {
//   RCLCPP_INFO(get_node()->get_logger(), "Configuring Manriix 4WS Controller");
  
//   wheel_fl_name_ = get_node()->get_parameter("front_left_wheel_joint").as_string();
//   wheel_fr_name_ = get_node()->get_parameter("front_right_wheel_joint").as_string();
//   wheel_rl_name_ = get_node()->get_parameter("rear_left_wheel_joint").as_string();
//   wheel_rr_name_ = get_node()->get_parameter("rear_right_wheel_joint").as_string();
//   steer_fl_name_ = get_node()->get_parameter("front_left_steering_joint").as_string();
//   steer_fr_name_ = get_node()->get_parameter("front_right_steering_joint").as_string();
//   steer_rl_name_ = get_node()->get_parameter("rear_left_steering_joint").as_string();
//   steer_rr_name_ = get_node()->get_parameter("rear_right_steering_joint").as_string();
  
//   wheelbase_ = get_node()->get_parameter("wheelbase").as_double();
//   track_width_ = get_node()->get_parameter("track_width").as_double();
//   wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
//   steering_offset_ = get_node()->get_parameter("steering_offset").as_double();
//   max_linear_vel_ = get_node()->get_parameter("max_linear_velocity").as_double();
//   max_angular_vel_ = get_node()->get_parameter("max_angular_velocity").as_double();
//   max_linear_accel_ = get_node()->get_parameter("max_linear_acceleration").as_double();
//   max_angular_accel_ = get_node()->get_parameter("max_angular_acceleration").as_double();
//   max_steering_angle_ = get_node()->get_parameter("max_steering_angle").as_double();
//   max_steering_rate_ = get_node()->get_parameter("max_steering_rate").as_double();
//   cmd_vel_timeout_ = get_node()->get_parameter("cmd_vel_timeout").as_double();
//   velocity_threshold_ = get_node()->get_parameter("velocity_threshold").as_double();
//   enable_angle_scaling_ = get_node()->get_parameter("enable_angle_scaling").as_bool();
//   max_scaled_angle_ = get_node()->get_parameter("max_scaled_angle").as_double();
//   angle_scale_factor_ = get_node()->get_parameter("angle_scale_factor").as_double();
//   base_frame_id_ = get_node()->get_parameter("base_frame_id").as_string();
//   odom_frame_id_ = get_node()->get_parameter("odom_frame_id").as_string();
//   enable_odom_tf_ = get_node()->get_parameter("enable_odom_tf").as_bool();
//   publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
  
//   publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
  
//   cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
//     "~/cmd_vel", 10,
//     std::bind(&Manriix4WSController::cmdVelCallback, this, std::placeholders::_1));
  
//   odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
//     "~/odom", rclcpp::SystemDefaultsQoS());
  
//   rt_odom_pub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_pub_);
  
//   if (enable_odom_tf_) {
//     tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());
//   }
  
//   Command empty_cmd;
//   command_buffer_.writeFromNonRT(empty_cmd);
  
//   RCLCPP_INFO(get_node()->get_logger(), "Manriix 4WS Controller configured");
//   RCLCPP_INFO(get_node()->get_logger(), "Angle scaling: %s (max: %.2f deg, factor: %.2f)",
//     enable_angle_scaling_ ? "ENABLED" : "DISABLED",
//     max_scaled_angle_ * 180.0 / M_PI, angle_scale_factor_);
  
//   return controller_interface::CallbackReturn::SUCCESS;
// }

// controller_interface::CallbackReturn Manriix4WSController::on_activate(
//   const rclcpp_lifecycle::State & /*previous_state*/)
// {
//   RCLCPP_INFO(get_node()->get_logger(), "Activating Manriix 4WS Controller");
  
//   for (size_t i = 0; i < 4; ++i) {
//     current_steering_[i] = state_interfaces_[i + 4].get_value();
//   }
  
//   command_interfaces_[0].set_value(0.0);
//   command_interfaces_[1].set_value(0.0);
//   command_interfaces_[2].set_value(0.0);
//   command_interfaces_[3].set_value(0.0);
  
//   current_command_ = Command();
//   last_command_ = Command();
  
//   last_update_time_ = get_node()->now();
//   last_publish_time_ = get_node()->now();
  
//   RCLCPP_INFO(get_node()->get_logger(), "Manriix 4WS Controller activated");
//   return controller_interface::CallbackReturn::SUCCESS;
// }

// controller_interface::CallbackReturn Manriix4WSController::on_deactivate(
//   const rclcpp_lifecycle::State & /*previous_state*/)
// {
//   RCLCPP_INFO(get_node()->get_logger(), "Deactivating Manriix 4WS Controller");
  
//   for (size_t i = 0; i < 4; ++i) {
//     command_interfaces_[i].set_value(0.0);
//   }
  
//   return controller_interface::CallbackReturn::SUCCESS;
// }

// controller_interface::return_type Manriix4WSController::update(
//   const rclcpp::Time & time, const rclcpp::Duration & period)
// {
//   // Read latest command from buffer
//   current_command_ = *(command_buffer_.readFromRT());
  
//   // Check for command timeout
//   double cmd_age = (time - current_command_.stamp).seconds();
//   if (cmd_age > cmd_vel_timeout_) {
//     current_command_.linear_x = 0.0;
//     current_command_.linear_y = 0.0;
//     current_command_.angular_z = 0.0;
//   }
  
//   // Apply acceleration limits
//   double dt = period.seconds();
//   double limited_vx = limitAcceleration(current_command_.linear_x, 
//                                         last_command_.linear_x,
//                                         max_linear_accel_, dt);
//   double limited_omega = limitAcceleration(current_command_.angular_z,
//                                            last_command_.angular_z,
//                                            max_angular_accel_, dt);
  
//   // Determine motion mode
//   current_mode_ = determineMode(limited_vx, current_command_.linear_y, limited_omega);
  
//   // Compute target steering angles
//   switch (current_mode_) {
//     case MotionMode::ACKERMANN:
//       computeAckermannSteering(limited_vx, limited_omega,
//                                target_steering_[0], target_steering_[1],
//                                target_steering_[2], target_steering_[3]);
//       break;
//     case MotionMode::PIVOT_TURN:
//       computePivotTurnSteering(target_steering_[0], target_steering_[1],
//                                target_steering_[2], target_steering_[3]);
//       break;
//     case MotionMode::LINEAR:
//       computeLinearSteering(target_steering_[0], target_steering_[1],
//                             target_steering_[2], target_steering_[3]);
//       break;
//     case MotionMode::STOP:
//       for (int i = 0; i < 4; ++i) {
//         target_steering_[i] = 0.0;
//       }
//       break;
//   }
  
//   // Smoothly interpolate steering angles
//   for (int i = 0; i < 4; ++i) {
//     current_steering_[i] = interpolateAngle(current_steering_[i], 
//                                              target_steering_[i], dt);
//   }
  
//   // Set steering commands
//   command_interfaces_[4].set_value(current_steering_[0]);
//   command_interfaces_[5].set_value(current_steering_[1]);
//   command_interfaces_[6].set_value(current_steering_[2]);
//   command_interfaces_[7].set_value(current_steering_[3]);
  
//   // Compute wheel velocities
//   double wfl, wfr, wrl, wrr;
//   computeWheelVelocities(limited_vx, limited_omega, current_mode_, wfl, wfr, wrl, wrr);
  
//   // Set wheel velocity commands
//   command_interfaces_[0].set_value(wfl);
//   command_interfaces_[1].set_value(wfr);
//   command_interfaces_[2].set_value(wrl);
//   command_interfaces_[3].set_value(wrr);
  
//   // Update odometry
//   updateOdometry(time, period, limited_vx, limited_omega);
  
//   // Publish odometry
//   if (time - last_publish_time_ >= publish_period_) {
//     publishOdometry(time, limited_vx, limited_omega);
//     last_publish_time_ = time;
//   }
  
//   // Save command for next iteration
//   last_command_.linear_x = limited_vx;
//   last_command_.angular_z = limited_omega;
//   last_update_time_ = time;
  
//   return controller_interface::return_type::OK;
// }

// void Manriix4WSController::cmdVelCallback(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
// {
//   Command cmd;
//   cmd.linear_x = clamp(msg->linear.x, -max_linear_vel_, max_linear_vel_);
//   cmd.linear_y = msg->linear.y;
//   cmd.angular_z = clamp(msg->angular.z, -max_angular_vel_, max_angular_vel_);
//   cmd.stamp = get_node()->now();
  
//   command_buffer_.writeFromNonRT(cmd);
// }

// Manriix4WSController::MotionMode Manriix4WSController::determineMode(
//   double vx, double vy, double omega)
// {
//   bool has_linear = (std::abs(vx) > velocity_threshold_ || std::abs(vy) > velocity_threshold_);
//   bool has_angular = std::abs(omega) > velocity_threshold_;
  
//   if (!has_linear && !has_angular) {
//     return MotionMode::STOP;
//   } else if (has_linear && !has_angular) {
//     return MotionMode::LINEAR;
//   } else if (has_linear && has_angular) {
//     return MotionMode::ACKERMANN;
//   } else {
//     return MotionMode::PIVOT_TURN;
//   }
// }

// void Manriix4WSController::computeAckermannSteering(
//   double vx, double omega, double& fl, double& fr, double& rl, double& rr)
// {
//   if (std::abs(omega) < 1e-6) {
//     fl = fr = rl = rr = 0.0;
//     return;
//   }
  
//   double steering_track = track_width_ - 2.0*steering_offset_;
//   double denominator_left = 2.0*vx - omega*steering_track;
//   double denominator_right = 2.0*vx + omega*steering_track;
  
//   double raw_fl, raw_fr;
  
//   if (std::abs(denominator_left) > 0.001 && std::abs(denominator_right) > 0.001) {
//     raw_fl = std::atan(omega * wheelbase_ / denominator_left);
//     raw_fr = std::atan(omega * wheelbase_ / denominator_right);
//   } else {
//     raw_fl = std::copysign(max_steering_angle_, omega);
//     raw_fr = std::copysign(max_steering_angle_, omega);
//   }
  
//   if (enable_angle_scaling_) {
//     fl = std::copysign(
//         std::min(max_scaled_angle_, std::abs(raw_fl) * angle_scale_factor_ * 
//         (1.0 - 0.4 * std::abs(raw_fl) / M_PI_2)), 
//         raw_fl);
        
//     fr = std::copysign(
//         std::min(max_scaled_angle_, std::abs(raw_fr) * angle_scale_factor_ * 
//         (1.0 - 0.4 * std::abs(raw_fr) / M_PI_2)), 
//         raw_fr);
//   } else {
//     fl = raw_fl;
//     fr = raw_fr;
//   }
  
//   rl = -fl;
//   rr = -fr;
  
//   fl = clamp(fl, -max_steering_angle_, max_steering_angle_);
//   fr = clamp(fr, -max_steering_angle_, max_steering_angle_);
//   rl = clamp(rl, -max_steering_angle_, max_steering_angle_);
//   rr = clamp(rr, -max_steering_angle_, max_steering_angle_);
// }

// void Manriix4WSController::computePivotTurnSteering(double& fl, double& fr, double& rl, double& rr)
// {
//   double angle = std::atan2(wheelbase_/2.0, track_width_/2.0);
//   fl = -angle;
//   fr = angle;
//   rl = angle;
//   rr = -angle;
// }

// void Manriix4WSController::computeLinearSteering(double& fl, double& fr, double& rl, double& rr)
// {
//   fl = fr = rl = rr = 0.0;
// }

// void Manriix4WSController::computeWheelVelocities(
//   double vx, double omega, MotionMode mode, double& wfl, double& wfr, double& wrl, double& wrr)
// {
//   switch (mode) {
//     case MotionMode::LINEAR:
//       wfl = wfr = wrl = wrr = vx / wheel_radius_;
//       break;
      
//     case MotionMode::ACKERMANN: {
//       if (std::abs(omega) < 1e-6) {
//         wfl = wfr = wrl = wrr = vx / wheel_radius_;
//         break;
//       }
      
//       double R = vx / omega;
//       double R_fl = std::sqrt(std::pow(R - track_width_/2.0, 2) + std::pow(wheelbase_/2.0, 2));
//       double R_fr = std::sqrt(std::pow(R + track_width_/2.0, 2) + std::pow(wheelbase_/2.0, 2));
//       double R_rl = std::sqrt(std::pow(R - track_width_/2.0, 2) + std::pow(wheelbase_/2.0, 2));
//       double R_rr = std::sqrt(std::pow(R + track_width_/2.0, 2) + std::pow(wheelbase_/2.0, 2));
      
//       double sign = (vx >= 0) ? 1.0 : -1.0;
//       wfl = sign * std::abs(omega * R_fl) / wheel_radius_;
//       wfr = sign * std::abs(omega * R_fr) / wheel_radius_;
//       wrl = sign * std::abs(omega * R_rl) / wheel_radius_;
//       wrr = sign * std::abs(omega * R_rr) / wheel_radius_;
//       break;
//     }
    
//     case MotionMode::PIVOT_TURN: {
//       double d_wheel = std::sqrt(std::pow(wheelbase_/2.0, 2) + std::pow(track_width_/2.0, 2));
//       double v_wheel = (omega * d_wheel) / wheel_radius_;
//       wfl = -v_wheel;
//       wfr = v_wheel;
//       wrl = -v_wheel;
//       wrr = v_wheel;
//       break;
//     }
    
//     case MotionMode::STOP:
//     default:
//       wfl = wfr = wrl = wrr = 0.0;
//       break;
//   }
  
//   wfl = clamp(wfl, -max_linear_vel_/wheel_radius_, max_linear_vel_/wheel_radius_);
//   wfr = clamp(wfr, -max_linear_vel_/wheel_radius_, max_linear_vel_/wheel_radius_);
//   wrl = clamp(wrl, -max_linear_vel_/wheel_radius_, max_linear_vel_/wheel_radius_);
//   wrr = clamp(wrr, -max_linear_vel_/wheel_radius_, max_linear_vel_/wheel_radius_);
// }

// double Manriix4WSController::interpolateAngle(double current, double target, double dt)
// {
//   double diff = target - current;
//   while (diff > M_PI) diff -= 2.0 * M_PI;
//   while (diff < -M_PI) diff += 2.0 * M_PI;
  
//   double max_change = max_steering_rate_ * dt;
//   if (std::abs(diff) <= max_change) return target;
  
//   return current + std::copysign(max_change, diff);
// }

// void Manriix4WSController::updateOdometry(
//   const rclcpp::Time& /*time*/, const rclcpp::Duration& period, double vx, double omega)
// {
//   double dt = period.seconds();
//   if (dt < 1e-6) return;
  
//   double delta_theta = omega * dt;
//   double delta_x = vx * std::cos(odom_theta_ + delta_theta/2.0) * dt;
//   double delta_y = vx * std::sin(odom_theta_ + delta_theta/2.0) * dt;
  
//   odom_x_ += delta_x;
//   odom_y_ += delta_y;
//   odom_theta_ += delta_theta;
  
//   while (odom_theta_ > M_PI) odom_theta_ -= 2.0 * M_PI;
//   while (odom_theta_ < -M_PI) odom_theta_ += 2.0 * M_PI;
// }

// void Manriix4WSController::publishOdometry(const rclcpp::Time& time, double vx, double omega)
// {
//   if (rt_odom_pub_->trylock()) {
//     auto & odom_msg = rt_odom_pub_->msg_;
//     odom_msg.header.stamp = time;
//     odom_msg.header.frame_id = odom_frame_id_;
//     odom_msg.child_frame_id = base_frame_id_;
    
//     odom_msg.pose.pose.position.x = odom_x_;
//     odom_msg.pose.pose.position.y = odom_y_;
//     odom_msg.pose.pose.position.z = 0.0;
    
//     tf2::Quaternion q;
//     q.setRPY(0.0, 0.0, odom_theta_);
//     odom_msg.pose.pose.orientation.x = q.x();
//     odom_msg.pose.pose.orientation.y = q.y();
//     odom_msg.pose.pose.orientation.z = q.z();
//     odom_msg.pose.pose.orientation.w = q.w();
    
//     odom_msg.twist.twist.linear.x = vx;
//     odom_msg.twist.twist.linear.y = 0.0;
//     odom_msg.twist.twist.linear.z = 0.0;
//     odom_msg.twist.twist.angular.x = 0.0;
//     odom_msg.twist.twist.angular.y = 0.0;
//     odom_msg.twist.twist.angular.z = omega;
    
//     for (size_t i = 0; i < 36; ++i) {
//       odom_msg.pose.covariance[i] = 0.0;
//       odom_msg.twist.covariance[i] = 0.0;
//     }
//     odom_msg.pose.covariance[0] = 0.05;
//     odom_msg.pose.covariance[7] = 0.05;
//     odom_msg.pose.covariance[35] = 0.1;
//     odom_msg.twist.covariance[0] = 0.02;
//     odom_msg.twist.covariance[35] = 0.05;
    
//     rt_odom_pub_->unlockAndPublish();
//   }
  
//   if (enable_odom_tf_ && tf_broadcaster_) {
//     geometry_msgs::msg::TransformStamped odom_tf;
//     odom_tf.header.stamp = time;
//     odom_tf.header.frame_id = odom_frame_id_;
//     odom_tf.child_frame_id = base_frame_id_;
//     odom_tf.transform.translation.x = odom_x_;
//     odom_tf.transform.translation.y = odom_y_;
//     odom_tf.transform.translation.z = 0.0;
    
//     tf2::Quaternion q;
//     q.setRPY(0.0, 0.0, odom_theta_);
//     odom_tf.transform.rotation.x = q.x();
//     odom_tf.transform.rotation.y = q.y();
//     odom_tf.transform.rotation.z = q.z();
//     odom_tf.transform.rotation.w = q.w();
    
//     tf_broadcaster_->sendTransform(odom_tf);
//   }
// }

// double Manriix4WSController::clamp(double value, double min_val, double max_val)
// {
//   return std::max(min_val, std::min(value, max_val));
// }

// double Manriix4WSController::limitAcceleration(
//   double desired, double current, double max_accel, double dt)
// {
//   double diff = desired - current;
//   double max_change = max_accel * dt;
//   if (std::abs(diff) <= max_change) return desired;
//   return current + std::copysign(max_change, diff);
// }

// }  // namespace manriix_controller

// #include "pluginlib/class_list_macros.hpp"

// PLUGINLIB_EXPORT_CLASS(
//   manriix_controller::Manriix4WSController,
//   controller_interface::ControllerInterface)
