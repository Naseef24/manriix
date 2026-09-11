#pragma once

#include <string>
#include <vector>
#include <memory>
#include <linux/can.h>
#include <thread>
#include <mutex>
#include <atomic>

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

  // Background receive thread control
  void startReceiveThread();
  void stopReceiveThread();

  // Get actual feedback from background thread (thread-safe)
  void getActualVelocities(std::vector<double>& velocities_rpm);

  // Cleanup
  void shutdown();

private:
  // CAN communication
  int can_socket_;
  std::string can_interface_;
  uint16_t driver1_id_;  // Left side (FL + RL)
  uint16_t driver2_id_;  // Right side (FR + RR)

    // TPDO COB-IDs (0x180 + node_id)
  // driver1 node_id = driver1_id_ & 0x7F → TPDO = 0x180 + node_id
  // driver2 node_id = driver2_id_ & 0x7F → TPDO = 0x180 + node_id
  uint16_t driver1_tpdo_id_;
  uint16_t driver2_tpdo_id_;

  // Helper functions
  bool sendCANMessage(uint16_t cob_id, const std::vector<uint8_t>& data);
  bool receiveCANMessage(can_frame& frame, double timeout_sec = 0.1);
  
  // Driver control
  bool sendVelocityCommand(uint16_t driver_id, int16_t motor1_rpm, int16_t motor2_rpm);
  bool readEncoderVelocity(uint16_t driver_id, uint8_t motor_index, int32_t& velocity);

    // TPDO configuration
  bool configureTPDO(uint16_t driver_id, uint16_t tpdo_id);

  // Last known velocities
  std::vector<double> last_velocities_;
  
  // Invert right side motors
  bool invert_right_motors_;

  // Background receive thread
  std::thread receive_thread_;
  std::atomic<bool> thread_running_{false};
  std::mutex velocities_mutex_;

  // Actual velocity arrays — updated by background thread
  // Order: FL, FR, RL, RR (0.1 RPM units from TPDO, converted to RPM)
  std::vector<double> actual_velocities_;

  // Background thread function
  void receiveLoop();
};

} // namespace manriix_hardware
