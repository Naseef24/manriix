#pragma once

#include <string>
#include <vector>
#include <memory>
#include <linux/can.h>

namespace manriix_hardware
{

class ZltechInterface
{
public:
  ZltechInterface();
  ~ZltechInterface();

  // Initialize CAN connection
  bool init(const std::string& can_interface, uint16_t driver1_id, uint16_t driver2_id);

  // Initialize motor drivers (must be called before use)
  bool initiateDrivers();

  // Set velocity for all 4 wheels (RPM)
  // Order: FL, FR, RL, RR
  bool setVelocities(const std::vector<double>& velocities_rpm);

  // Read velocity feedback from encoders
  bool readVelocities(std::vector<double>& velocities_rpm);

  // Emergency stop all motors
  bool stopMotors();

  // Check if interface is active
  bool isActive() const { return can_socket_ >= 0; }

  // Cleanup
  void shutdown();

private:
  // CAN communication
  int can_socket_;
  std::string can_interface_;
  uint16_t driver1_id_;  // Left side (FL + RL)
  uint16_t driver2_id_;  // Right side (FR + RR)

  // Helper functions
  bool sendCANMessage(uint16_t cob_id, const std::vector<uint8_t>& data);
  bool receiveCANMessage(can_frame& frame, double timeout_sec = 0.1);
  
  // Driver control
  bool sendVelocityCommand(uint16_t driver_id, int16_t motor1_rpm, int16_t motor2_rpm);
  bool readEncoderVelocity(uint16_t driver_id, uint8_t motor_index, int32_t& velocity);

  // Last known velocities
  std::vector<double> last_velocities_;
  
  // Invert right side motors
  bool invert_right_motors_;
};

} // namespace manriix_hardware
