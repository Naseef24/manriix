#pragma once

#include <string>
#include <vector>
#include <memory>
#include <linux/can.h>

namespace manriix_hardware
{

class CubeMarsInterface
{
public:
  CubeMarsInterface();
  ~CubeMarsInterface();

  // Initialize CAN connection
  bool init(const std::string& can_interface, int base_motor_id);

  // Set position for a specific steering actuator (0-3)
  bool setPosition(int actuator_index, double position_degrees);

  // Read position feedback from actuators
  bool readPositions(std::vector<double>& positions);

  // Check if interface is active
  bool isActive() const { return can_socket_ >= 0; }

  // Cleanup
  void shutdown();

private:
  // CAN communication
  int can_socket_;
  std::string can_interface_;
  int base_motor_id_;

  // Servo control modes
  static constexpr int SERVO_MODE_POSITION = 4;

  // Helper functions
  bool sendCANMessage(int motor_id, int mode, const std::vector<uint8_t>& data);
  bool receiveCANMessage(can_frame& frame, double timeout_sec = 0.05);
  std::vector<uint8_t> encodePosition(double position_degrees);
  double decodePosition(const can_frame& frame);

  // Last known positions
  std::vector<double> last_positions_;
};

} // namespace manriix_hardware
