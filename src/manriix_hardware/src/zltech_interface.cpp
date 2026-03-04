#include "manriix_hardware/zltech_interface.hpp"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

namespace manriix_hardware
{

ZltechInterface::ZltechInterface()
  : can_socket_(-1), driver1_id_(0x601), driver2_id_(0x602),
    last_velocities_(4, 0.0), invert_right_motors_(true)
{
}

ZltechInterface::~ZltechInterface()
{
  shutdown();
}

bool ZltechInterface::init(const std::string& can_interface, uint16_t driver1_id, uint16_t driver2_id)
{
  can_interface_ = can_interface;
  driver1_id_ = driver1_id;
  driver2_id_ = driver2_id;

  // Create CAN socket
  can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    std::cerr << "Failed to create CAN socket for Zltech" << std::endl;
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
    std::cerr << "Failed to bind CAN socket for Zltech" << std::endl;
    close(can_socket_);
    can_socket_ = -1;
    return false;
  }

  std::cout << "Zltech interface initialized on " << can_interface 
            << " with driver IDs 0x" << std::hex << driver1_id 
            << ", 0x" << driver2_id << std::dec << std::endl;
  
  return true;
}

bool ZltechInterface::initiateDrivers()
{
  std::vector<uint8_t> mode_cmd = {0x2F, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};
  std::vector<uint8_t> enable_cmd1 = {0x2B, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
  std::vector<uint8_t> enable_cmd2 = {0x2B, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00};
  std::vector<uint8_t> enable_cmd3 = {0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00};

  for (auto driver_id : {driver1_id_, driver2_id_}) {
    if (!sendCANMessage(driver_id, mode_cmd)) return false;
    usleep(10000);
    if (!sendCANMessage(driver_id, enable_cmd1)) return false;
    usleep(10000);
    if (!sendCANMessage(driver_id, enable_cmd2)) return false;
    usleep(10000);
    if (!sendCANMessage(driver_id, enable_cmd3)) return false;
    usleep(10000);
  }

  std::cout << "Zltech drivers initiated" << std::endl;
  return true;
}

bool ZltechInterface::setVelocities(const std::vector<double>& velocities_rpm)
{
  if (velocities_rpm.size() != 4) return false;

  // FL, FR, RL, RR -> Driver1(FL, RL), Driver2(FR, RR)
  int16_t left_front = static_cast<int16_t>(velocities_rpm[0]);
  int16_t left_rear = static_cast<int16_t>(velocities_rpm[2]);
  int16_t right_front = static_cast<int16_t>(velocities_rpm[1]);
  int16_t right_rear = static_cast<int16_t>(velocities_rpm[3]);

  // Invert right motors if needed
  if (invert_right_motors_) {
    right_front = -right_front;
    right_rear = -right_rear;
  }

  // Send to driver 1 (left side)
  if (!sendVelocityCommand(driver1_id_, left_front, left_rear)) return false;
  
  // Send to driver 2 (right side)
  if (!sendVelocityCommand(driver2_id_, right_front, right_rear)) return false;

  return true;
}

bool ZltechInterface::readVelocities(std::vector<double>& velocities_rpm)
{
  velocities_rpm.resize(4);

  int32_t left_front, left_rear, right_front, right_rear;

  // Read from driver 1 (left side)
  if (!readEncoderVelocity(driver1_id_, 1, left_front)) left_front = last_velocities_[0];
  if (!readEncoderVelocity(driver1_id_, 2, left_rear)) left_rear = last_velocities_[2];

  // Read from driver 2 (right side)
  if (!readEncoderVelocity(driver2_id_, 1, right_front)) right_front = last_velocities_[1];
  if (!readEncoderVelocity(driver2_id_, 2, right_rear)) right_rear = last_velocities_[3];

  // Convert to RPM and apply corrections
  velocities_rpm[0] = -left_front / 10.0;  // Invert left side
  velocities_rpm[2] = -left_rear / 10.0;
  velocities_rpm[1] = right_front / 10.0;
  velocities_rpm[3] = right_rear / 10.0;

  // Store last known values
  last_velocities_ = velocities_rpm;

  return true;
}

bool ZltechInterface::stopMotors()
{
  std::vector<uint8_t> stop_cmd = {0x23, 0xFF, 0x60, 0x03, 0x00, 0x00, 0x00, 0x00};
  
  bool success = true;
  success &= sendCANMessage(driver1_id_, stop_cmd);
  success &= sendCANMessage(driver2_id_, stop_cmd);
  
  std::cout << "Zltech motors stopped" << std::endl;
  return success;
}

void ZltechInterface::shutdown()
{
  if (can_socket_ >= 0) {
    stopMotors();
    close(can_socket_);
    can_socket_ = -1;
    std::cout << "Zltech interface shutdown" << std::endl;
  }
}

bool ZltechInterface::sendCANMessage(uint16_t cob_id, const std::vector<uint8_t>& data)
{
  if (can_socket_ < 0) return false;

  struct can_frame frame;
  frame.can_id = cob_id;
  frame.can_dlc = data.size();
  
  for (size_t i = 0; i < data.size() && i < 8; i++) {
    frame.data[i] = data[i];
  }

  if (write(can_socket_, &frame, sizeof(frame)) != sizeof(frame)) {
    return false;
  }

  return true;
}

bool ZltechInterface::receiveCANMessage(can_frame& frame, double timeout_sec)
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

bool ZltechInterface::sendVelocityCommand(uint16_t driver_id, int16_t motor1_rpm, int16_t motor2_rpm)
{
  std::vector<uint8_t> cmd = {
    0x23, 0xFF, 0x60, 0x03,
    static_cast<uint8_t>(motor1_rpm & 0xFF),
    static_cast<uint8_t>((motor1_rpm >> 8) & 0xFF),
    static_cast<uint8_t>(motor2_rpm & 0xFF),
    static_cast<uint8_t>((motor2_rpm >> 8) & 0xFF)
  };

  return sendCANMessage(driver_id, cmd);
}

bool ZltechInterface::readEncoderVelocity(uint16_t driver_id, uint8_t motor_index, int32_t& velocity)
{
  std::vector<uint8_t> read_cmd = {0x40, 0x6C, 0x60, motor_index, 0x00, 0x00, 0x00, 0x00};
  
  if (!sendCANMessage(driver_id, read_cmd)) return false;

  can_frame frame;
  if (!receiveCANMessage(frame, 0.1)) return false;

  // Extract velocity from last 4 bytes (little-endian, signed)
  velocity = static_cast<int32_t>(
    frame.data[4] | 
    (frame.data[5] << 8) | 
    (frame.data[6] << 16) | 
    (frame.data[7] << 24)
  );

  return true;
}

} // namespace manriix_hardware
