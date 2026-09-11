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

  // Background receive thread control
  void startReceiveThread();
  void stopReceiveThread();

  // Get actual feedback from background thread (thread-safe)
  void getActualPositions(std::vector<double>& positions_deg);
  void getActualVelocities(std::vector<double>& velocities_erpm);
  void getActualCurrents(std::vector<double>& currents_a);
  void getActualTemperatures(std::vector<double>& temps_c);

  // Cleanup
  void shutdown();

private:
  // CAN communication
  int can_socket_;
  std::string can_interface_;
  int base_motor_id_;

  // Servo control modes
  static constexpr int SERVO_MODE_POSITION = 4;

  // Feedback frame identifier (high byte of extended CAN ID)
  static constexpr int FEEDBACK_MODE_ID = 0x29;

  // Helper functions
  bool sendCANMessage(int motor_id, int mode, const std::vector<uint8_t>& data);
  bool receiveCANMessage(can_frame& frame, double timeout_sec = 0.05);
  std::vector<uint8_t> encodePosition(double position_degrees);
  // double decodePosition(const can_frame& frame);
  void decodeFeedback(const can_frame& frame, int motor_index);

  // Last known positions
  std::vector<double> last_positions_;
  
  // Background receive thread
  std::thread receive_thread_;
  std::atomic<bool> thread_running_{false};
  std::mutex feedback_mutex_;

  // Actual feedback arrays — updated by background thread
  std::vector<double> actual_positions_;    // degrees
  std::vector<double> actual_velocities_;   // ERPM
  std::vector<double> actual_currents_;     // Amperes
  std::vector<double> actual_temperatures_; // Celsius

  // Background thread function
  void receiveLoop();  
};

} // namespace manriix_hardware
