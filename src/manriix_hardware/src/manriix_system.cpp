#include "manriix_hardware/manriix_system.hpp"
#include <cstddef>
#include <limits>
#include <vector>
#include <string>
#include <iostream>

#define C_RESET        "\033[0m"
#define C_BOLD         "\033[1m"
#define C_UNCONFIGURED "\033[90m"   // dark grey
#define C_INACTIVE     "\033[33m"   // yellow
#define C_ACTIVE       "\033[32m"   // green
#define C_SHUTDOWN     "\033[31m"   // red
#define C_THREAD       "\033[36m"   // cyan
#define C_FEEDBACK     "\033[35m"   // magenta

namespace manriix_hardware
{

hardware_interface::CallbackReturn ManriixSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != 
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get configuration from URDF (MOTORS ONLY - can2)
  can_interface_ = info_.hardware_parameters["can_interface"];

  cubemars_base_id_ = std::stoi(info_.hardware_parameters["cubemars_base_id"]);
  zltech_driver1_id_ = std::stoi(info_.hardware_parameters["zltech_driver1_id"], nullptr, 16);
  zltech_driver2_id_ = std::stoi(info_.hardware_parameters["zltech_driver2_id"], nullptr, 16);

  // Initialize joint name lists (MOTORS ONLY)
  for (const auto & joint : info_.joints)
  {
    if (joint.name.find("steering") != std::string::npos) {
      steering_joint_names_.push_back(joint.name);
    } else if (joint.name.find("wheel") != std::string::npos) {
      wheel_joint_names_.push_back(joint.name);
    }
  }

  // Initialize state and command vectors (MOTORS ONLY)
  steering_positions_.resize(steering_joint_names_.size(), 0.0);
  steering_velocities_.resize(steering_joint_names_.size(), 0.0);
  steering_position_commands_.resize(steering_joint_names_.size(), 0.0);

  wheel_positions_.resize(wheel_joint_names_.size(), 0.0);
  wheel_velocities_.resize(wheel_joint_names_.size(), 0.0);
  wheel_velocity_commands_.resize(wheel_joint_names_.size(), 0.0);

  // Create hardware interfaces (MOTORS ONLY)
  steering_interface_ = std::make_unique<CubeMarsInterface>();
  wheel_interface_ = std::make_unique<ZltechInterface>();
  
  // Initialize availability flags
  steering_available_ = false;
  wheels_available_ = false;
  
  std::cout << "\n╔══════════════════════════════════════════╗" << std::endl;
  std::cout << "║   Manriix Hardware System Init (Motors)  ║" << std::endl;
  std::cout << "╚══════════════════════════════════════════╝" << std::endl;
  std::cout << "  CAN Interface: " << can_interface_ << std::endl;
  std::cout << "  Steering joints: " << steering_joint_names_.size() << std::endl;
  std::cout << "  Wheel joints: " << wheel_joint_names_.size() << std::endl;
  std::cout << "  NOTE: Gimbal controlled separately via Python" << std::endl;
  std::cout << std::endl;
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ManriixSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Wheel joints: velocity and position
  for (size_t i = 0; i < wheel_joint_names_.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_joint_names_[i], "velocity", &wheel_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wheel_joint_names_[i], "position", &wheel_positions_[i]));
  }

  // Steering joints: position and velocity
  for (size_t i = 0; i < steering_joint_names_.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      steering_joint_names_[i], "position", &steering_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      steering_joint_names_[i], "velocity", &steering_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ManriixSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Wheel joints: velocity command
  for (size_t i = 0; i < wheel_joint_names_.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wheel_joint_names_[i], "velocity", &wheel_velocity_commands_[i]));
  }

  // Steering joints: position command
  for (size_t i = 0; i < steering_joint_names_.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      steering_joint_names_[i], "position", &steering_position_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ManriixSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::cout << "\n╔══════════════════════════════════════════╗" << std::endl;
  std::cout << "║  Activating Manriix Hardware (Motors)    ║" << std::endl;
  std::cout << "║  Auto-detecting available devices...     ║" << std::endl;
  std::cout << "╚══════════════════════════════════════════╝\n" << std::endl;
  
  // Try to initialize each motor subsystem
  
  // 1. Try Steering Motors (CubeMars)
  if (steering_joint_names_.size() > 0) {
    std::cout << "→ Attempting to initialize CubeMars steering actuators..." << std::endl;
    if (steering_interface_->init(can_interface_, cubemars_base_id_)) {
      steering_available_ = true;
      std::cout << "  ✓ Steering actuators initialized successfully" << std::endl;
    } else {
      std::cout << "  ✗ Steering actuators not available (will skip)" << std::endl;
    }
  } else {
    std::cout << "→ No steering joints defined, skipping steering init" << std::endl;
  }
  std::cout << std::endl;

  // 2. Try Wheel Motors (Zltech)
  if (wheel_joint_names_.size() > 0) {
    std::cout << "→ Attempting to initialize Zltech hub motors..." << std::endl;
    if (wheel_interface_->init(can_interface_, zltech_driver1_id_, zltech_driver2_id_)) {
      std::cout << "  ✓ Wheel interface initialized" << std::endl;
      
      // Try to initiate drivers
      std::cout << "  → Initiating Zltech drivers..." << std::endl;
      if (wheel_interface_->initiateDrivers()) {
        wheels_available_ = true;
        std::cout << "  ✓ Zltech drivers ready" << std::endl;
      } else {
        std::cout << "  ✗ Failed to initiate drivers (will skip wheel control)" << std::endl;
      }
    } else {
      std::cout << "  ✗ Wheel motors not available (will skip)" << std::endl;
    }
  } else {
    std::cout << "→ No wheel joints defined, skipping wheel init" << std::endl;
  }
  std::cout << std::endl;

  // Summary
  std::cout << "╔══════════════════════════════════════════╗" << std::endl;
  std::cout << "║        Motor Hardware Summary            ║" << std::endl;
  std::cout << "╚══════════════════════════════════════════╝" << std::endl;
  std::cout << "  Steering:  " << (steering_available_ ? "✓ AVAILABLE" : "✗ Not available") << std::endl;
  std::cout << "  Wheels:    " << (wheels_available_ ? "✓ AVAILABLE" : "✗ Not available") << std::endl;
  std::cout << "  Gimbal:    Managed separately (Python/can3)" << std::endl;
  std::cout << std::endl;

  // At least one device must be available (or allow running without hardware for testing)
  if (!steering_available_ && !wheels_available_) {
    std::cout << "⚠ WARNING: No motor hardware available!" << std::endl;
    std::cout << "  System will continue but motors won't respond." << std::endl;
    std::cout << "  Check CAN bus connections and power." << std::endl;
  }

  // std::cout << "✓ Motor system activation complete\n" << std::endl;
  
  // return hardware_interface::CallbackReturn::SUCCESS;
  // Start background receive threads for closed-loop feedback
  if (steering_available_) {
    steering_interface_->startReceiveThread();
    std::cout << C_THREAD C_BOLD "[THREAD]" C_RESET
              << " CubeMars receive thread started" << std::endl;
  }

  if (wheels_available_) {
    wheel_interface_->startReceiveThread();
    std::cout << C_THREAD C_BOLD "[THREAD]" C_RESET
              << " Zltech receive thread started" << std::endl;
  }

  std::cout << C_ACTIVE C_BOLD "[ACTIVE]" C_RESET
            << " Motor system activation complete\n" << std::endl;

  return hardware_interface::CallbackReturn::SUCCESS;  
}

// hardware_interface::CallbackReturn ManriixSystem::on_deactivate(
//   const rclcpp_lifecycle::State & /*previous_state*/)
// {
//   std::cout << "\n→ Deactivating Manriix Motor Hardware..." << std::endl;
  
//   // Stop and shutdown available motors
//   if (wheels_available_) {
//     std::cout << "  → Stopping wheel motors..." << std::endl;
//     wheel_interface_->stopMotors();
//   }
  
//   if (steering_available_) {
//     std::cout << "  → Shutting down steering interface..." << std::endl;
//     steering_interface_->shutdown();
//   }
  
//   if (wheels_available_) {
//     std::cout << "  → Shutting down wheel interface..." << std::endl;
//     wheel_interface_->shutdown();
//   }
  
//   std::cout << "✓ Motor system deactivated\n" << std::endl;

//   return hardware_interface::CallbackReturn::SUCCESS;
// }

hardware_interface::CallbackReturn ManriixSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::cout << C_INACTIVE C_BOLD "[INACTIVE]" C_RESET
            << " Deactivating Manriix Motor Hardware..." << std::endl;

  // Stop receive threads first — before closing sockets
  if (steering_available_) {
    std::cout << "  → Stopping CubeMars receive thread..." << std::endl;
    steering_interface_->stopReceiveThread();
  }

  if (wheels_available_) {
    std::cout << "  → Stopping Zltech receive thread..." << std::endl;
    wheel_interface_->stopReceiveThread();
  }

  // Stop motors
  if (wheels_available_) {
    std::cout << "  → Stopping wheel motors..." << std::endl;
    wheel_interface_->stopMotors();
  }

  if (steering_available_) {
    std::cout << "  → Shutting down steering interface..." << std::endl;
    steering_interface_->shutdown();
  }

  if (wheels_available_) {
    std::cout << "  → Shutting down wheel interface..." << std::endl;
    wheel_interface_->shutdown();
  }

  std::cout << C_INACTIVE C_BOLD "[INACTIVE]" C_RESET
            << " Motor system deactivated\n" << std::endl;

  return hardware_interface::CallbackReturn::SUCCESS;
}

// hardware_interface::return_type ManriixSystem::read(
//   const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
// {
//   // Non-blocking read - motors broadcast state automatically via CAN
//   // State variables maintain last known values from CAN broadcasts
//   return hardware_interface::return_type::OK;
// }
hardware_interface::return_type ManriixSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Copy actual feedback from background threads into state interfaces
  // Both getActual*() calls are mutex-protected and complete in microseconds
  // No blocking, no CAN communication in this function

  if (steering_available_) {
    std::vector<double> positions_deg;
    steering_interface_->getActualPositions(positions_deg);
    for (size_t i = 0; i < steering_positions_.size() && i < positions_deg.size(); i++) {
      steering_positions_[i] = positions_deg[i] * DEG_TO_RAD;
    }
  }

  if (wheels_available_) {
    std::vector<double> velocities_rpm;
    wheel_interface_->getActualVelocities(velocities_rpm);
    for (size_t i = 0; i < wheel_velocities_.size() && i < velocities_rpm.size(); i++) {
      wheel_velocities_[i] = velocities_rpm[i] * RPM_TO_RAD_S;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ManriixSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Send commands only to available motor devices
  
  // Steering commands
  if (steering_available_) {
    for (size_t i = 0; i < steering_position_commands_.size(); i++)
    {
      double position_deg = steering_position_commands_[i] * RAD_TO_DEG;
      steering_interface_->setPosition(i, position_deg);
    }
  }

  // Wheel commands
  if (wheels_available_) {
    std::vector<double> wheel_velocities_rpm(wheel_velocity_commands_.size());
    for (size_t i = 0; i < wheel_velocity_commands_.size(); i++)
    {
      wheel_velocities_rpm[i] = wheel_velocity_commands_[i] * RAD_S_TO_RPM;
    }
    wheel_interface_->setVelocities(wheel_velocities_rpm);
  }
  
  return hardware_interface::return_type::OK;
}

} // namespace manriix_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  manriix_hardware::ManriixSystem, hardware_interface::SystemInterface)