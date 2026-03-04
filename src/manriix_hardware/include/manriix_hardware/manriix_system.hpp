#pragma once

#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "manriix_hardware/cubemars_interface.hpp"
#include "manriix_hardware/zltech_interface.hpp"
// REMOVED: #include "manriix_hardware/gimbal_interface.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace manriix_hardware
{

class ManriixSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ManriixSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Hardware interfaces (MOTORS ONLY)
  std::unique_ptr<CubeMarsInterface> steering_interface_;
  std::unique_ptr<ZltechInterface> wheel_interface_;
  // REMOVED: std::unique_ptr<GimbalInterface> gimbal_interface_;

  // Joint names (MOTORS ONLY)
  std::vector<std::string> steering_joint_names_;
  std::vector<std::string> wheel_joint_names_;
  // REMOVED: std::vector<std::string> gimbal_joint_names_;

  // State storage (MOTORS ONLY)
  std::vector<double> steering_positions_;
  std::vector<double> steering_velocities_;
  std::vector<double> wheel_positions_;
  std::vector<double> wheel_velocities_;
  // REMOVED: gimbal positions/velocities
  
  // Command storage (MOTORS ONLY)
  std::vector<double> steering_position_commands_;
  std::vector<double> wheel_velocity_commands_;
  // REMOVED: gimbal position commands

  // Hardware availability flags (MOTORS ONLY)
  bool steering_available_;
  bool wheels_available_;
  // REMOVED: bool gimbal_available_;

  // Configuration (MOTORS ONLY - can2)
  std::string can_interface_;
  // REMOVED: std::string gimbal_can_interface_;
  int cubemars_base_id_;
  uint16_t zltech_driver1_id_;
  uint16_t zltech_driver2_id_;

  // Conversion constants
  static constexpr double DEG_TO_RAD = M_PI / 180.0;
  static constexpr double RAD_TO_DEG = 180.0 / M_PI;
  static constexpr double RPM_TO_RAD_S = 2.0 * M_PI / 60.0;
  static constexpr double RAD_S_TO_RPM = 60.0 / (2.0 * M_PI);
  static constexpr double WHEEL_RADIUS = 0.101;  // meters
};

} // namespace manriix_hardware