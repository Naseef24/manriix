#include "manriix_hardware/cubemars_interface.hpp"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <chrono>

// ANSI colors for terminal output
#define C_RESET    "\033[0m"
#define C_BOLD     "\033[1m"
#define C_ACTIVE   "\033[32m"   // green
#define C_THREAD   "\033[36m"   // cyan
#define C_FEEDBACK "\033[35m"   // magenta
#define C_WARN     "\033[33m"   // yellow
#define C_ERROR    "\033[31m"   // red

namespace manriix_hardware
{

CubeMarsInterface::CubeMarsInterface()
  : can_socket_(-1), base_motor_id_(0),
    last_positions_(4, 0.0),
    actual_positions_(4, 0.0),
    actual_velocities_(4, 0.0),
    actual_currents_(4, 0.0),
    actual_temperatures_(4, 0.0)    
{
}

CubeMarsInterface::~CubeMarsInterface()
{
  shutdown();
}

bool CubeMarsInterface::init(const std::string& can_interface, int base_motor_id)
{
  can_interface_ = can_interface;
  base_motor_id_ = base_motor_id;

  // Create CAN socket
  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    std::cerr << "Failed to create CAN socket" << std::endl;
    return false;
  }

  // Get interface index
  struct ifreq ifr;
  std::strcpy(ifr.ifr_name, can_interface.c_str());
  if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    std::cerr << "Failed to get interface index for " << can_interface << std::endl;
    close(can_socket_);
    can_socket_ = -1;
    return false;
  }

  // Bind socket to CAN interface
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    std::cerr << "Failed to bind CAN socket" << std::endl;
    close(can_socket_);
    can_socket_ = -1;
    return false;
  }

  // Set socket to non-blocking
  int flags = fcntl(can_socket_, F_GETFL, 0);
  fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);

  std::cout << "CubeMars interface initialized on " << can_interface 
            << " with base ID " << base_motor_id << std::endl;
  return true;
}

bool CubeMarsInterface::setPosition(int actuator_index, double position_degrees)
{
  if (actuator_index < 0 || actuator_index > 3) {
    return false;
  }

  int motor_id = base_motor_id_ + actuator_index;
  auto data = encodePosition(position_degrees);
  
  return sendCANMessage(motor_id, SERVO_MODE_POSITION, data);
}

bool CubeMarsInterface::readPositions(std::vector<double>& positions)
{
  positions.resize(4);
  
  // Use last known positions (motors broadcast automatically)
  for (int i = 0; i < 4; i++) {
    positions[i] = last_positions_[i];
  }
  
  return true;
}

void CubeMarsInterface::shutdown()
// {
//   if (can_socket_ >= 0) {
//     close(can_socket_);
//     can_socket_ = -1;
//     std::cout << "CubeMars interface shutdown" << std::endl;
//   }
// }
{
  stopReceiveThread();
  if (can_socket_ >= 0) {
    close(can_socket_);
    can_socket_ = -1;
    std::cout << C_WARN "[CUBEMARS]" C_RESET
              << " Interface shutdown" << std::endl;
  }
}

bool CubeMarsInterface::sendCANMessage(int motor_id, int mode, const std::vector<uint8_t>& data)
{
  if (can_socket_ < 0) return false;

  struct can_frame frame;
  frame.can_id = (mode << 8) | motor_id;
  frame.can_id |= CAN_EFF_FLAG;  // Extended ID
  frame.can_dlc = data.size();
  
  for (size_t i = 0; i < data.size() && i < 8; i++) {
    frame.data[i] = data[i];
  }

  if (write(can_socket_, &frame, sizeof(frame)) != sizeof(frame)) {
    return false;
  }

  return true;
}

bool CubeMarsInterface::receiveCANMessage(can_frame& frame, double timeout_sec)
{
  if (can_socket_ < 0) return false;

  struct timeval tv;
  tv.tv_sec = static_cast<long>(timeout_sec);
  tv.tv_usec = static_cast<long>((timeout_sec - tv.tv_sec) * 1000000);

  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(can_socket_, &readfds);

  int ret = select(can_socket_ + 1, &readfds, nullptr, nullptr, &tv);
  
  if (ret > 0) {
    ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
    return nbytes == sizeof(frame);
  }

  return false;
}

std::vector<uint8_t> CubeMarsInterface::encodePosition(double position_degrees)
{
  int32_t position_int = static_cast<int32_t>(position_degrees * 10000.0);
  
  std::vector<uint8_t> data(4);
  data[0] = (position_int >> 24) & 0xFF;
  data[1] = (position_int >> 16) & 0xFF;
  data[2] = (position_int >> 8) & 0xFF;
  data[3] = position_int & 0xFF;
  
  return data;
}

// double CubeMarsInterface::decodePosition(const can_frame& frame)
// {
//   // Position feedback is in first 2 bytes (signed)
//   int16_t pos_raw = (static_cast<int16_t>(frame.data[0]) << 8) | frame.data[1];
//   return pos_raw / 10.0;  // Convert to degrees
// }

void CubeMarsInterface::decodeFeedback(const can_frame& frame, int motor_index)
{
  if (motor_index < 0 || motor_index > 3) return;

  // Per manual section 4.3.1 — 8 byte feedback frame:
  // data[0-1]: position  int16  range -32000..32000 = -3200°..3200°  × 0.1 = degrees
  // data[2-3]: speed     int16  range -32000..32000 = -320000..320000 ERPM
  // data[4-5]: current   int16  range -6000..6000   = -60..60 A      × 0.01 = A
  // data[6]:   temp      int8   range -20..127       = °C
  // data[7]:   error     uint8  0=ok 1-7=fault codes

  int16_t pos_raw  = (static_cast<int16_t>(frame.data[0]) << 8) | frame.data[1];
  int16_t spd_raw  = (static_cast<int16_t>(frame.data[2]) << 8) | frame.data[3];
  int16_t cur_raw  = (static_cast<int16_t>(frame.data[4]) << 8) | frame.data[5];
  int8_t  temp_raw = static_cast<int8_t>(frame.data[6]);
  uint8_t err_raw  = frame.data[7];

  std::lock_guard<std::mutex> lock(feedback_mutex_);
  actual_positions_[motor_index]    = pos_raw  * 0.1;   // degrees
  actual_velocities_[motor_index]   = spd_raw  * 10.0;  // ERPM
  actual_currents_[motor_index]     = cur_raw  * 0.01;  // Amperes
  actual_temperatures_[motor_index] = static_cast<double>(temp_raw); // °C

  if (err_raw != 0) {
    std::cerr << C_ERROR C_BOLD "[CUBEMARS FAULT]" C_RESET
              << " motor " << (motor_index + 1)
              << " error code: " << static_cast<int>(err_raw) << std::endl;
  }
}

void CubeMarsInterface::startReceiveThread()
{
  if (thread_running_) return;
  thread_running_ = true;
  receive_thread_ = std::thread(&CubeMarsInterface::receiveLoop, this);
  std::cout << C_THREAD C_BOLD "[THREAD]" C_RESET
            << " CubeMars receive thread started on " << can_interface_ << std::endl;
}

void CubeMarsInterface::stopReceiveThread()
{
  if (!thread_running_) return;
  thread_running_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  std::cout << C_THREAD C_BOLD "[THREAD]" C_RESET
            << " CubeMars receive thread stopped" << std::endl;
}

void CubeMarsInterface::receiveLoop()
{
  struct can_frame frame;

  while (thread_running_) {
    if (can_socket_ < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    // Non-blocking read — returns immediately if no frame available
    ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));

    if (nbytes == sizeof(frame)) {
      // Only process extended frames (CubeMars feedback)
      if (!(frame.can_id & CAN_EFF_FLAG)) continue;

      // Extract mode ID and motor ID from extended CAN ID
      uint32_t raw_id   = frame.can_id & CAN_EFF_MASK;
      uint8_t  mode_id  = (raw_id >> 8) & 0xFF;
      uint8_t  motor_id = raw_id & 0xFF;

      // Only process feedback frames (mode_id == 0x29)
      if (mode_id != FEEDBACK_MODE_ID) continue;

      // Convert motor_id to array index (0-3)
      int motor_index = static_cast<int>(motor_id) - base_motor_id_;
      if (motor_index < 0 || motor_index > 3) continue;

      decodeFeedback(frame, motor_index);

    } else {
      // No frame available — sleep briefly to avoid busy-waiting
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
  }
}

void CubeMarsInterface::getActualPositions(std::vector<double>& positions_deg)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  positions_deg = actual_positions_;
}

void CubeMarsInterface::getActualVelocities(std::vector<double>& velocities_erpm)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  velocities_erpm = actual_velocities_;
}

void CubeMarsInterface::getActualCurrents(std::vector<double>& currents_a)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  currents_a = actual_currents_;
}

void CubeMarsInterface::getActualTemperatures(std::vector<double>& temps_c)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  temps_c = actual_temperatures_;
}

} // namespace manriix_hardware
