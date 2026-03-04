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

namespace manriix_hardware
{

CubeMarsInterface::CubeMarsInterface()
  : can_socket_(-1), base_motor_id_(0), last_positions_(4, 0.0)
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
{
  if (can_socket_ >= 0) {
    close(can_socket_);
    can_socket_ = -1;
    std::cout << "CubeMars interface shutdown" << std::endl;
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

double CubeMarsInterface::decodePosition(const can_frame& frame)
{
  // Position feedback is in first 2 bytes (signed)
  int16_t pos_raw = (static_cast<int16_t>(frame.data[0]) << 8) | frame.data[1];
  return pos_raw / 10.0;  // Convert to degrees
}

} // namespace manriix_hardware
