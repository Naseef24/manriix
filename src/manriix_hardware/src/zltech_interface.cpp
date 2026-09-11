#include "manriix_hardware/zltech_interface.hpp"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>

#define C_RESET    "\033[0m"
#define C_BOLD     "\033[1m"
#define C_THREAD   "\033[36m"   // cyan
#define C_WARN     "\033[33m"   // yellow
#define C_ERROR    "\033[31m"   // red
#define C_ACTIVE   "\033[32m"   // green

namespace manriix_hardware
{

ZltechInterface::ZltechInterface()
  : can_socket_(-1), driver1_id_(0x601), driver2_id_(0x602),
    driver1_tpdo_id_(0x181), driver2_tpdo_id_(0x182),
    last_velocities_(4, 0.0), invert_right_motors_(true),
    actual_velocities_(4, 0.0)
{
}

// Previous setup (older reference, keep for easy swap):
// ZltechInterface::ZltechInterface()
//   : can_socket_(-1), driver1_id_(0x601), driver2_id_(0x602),
//     driver1_tpdo_id_(0x181), driver2_tpdo_id_(0x182),
//     last_velocities_(4, 0.0), invert_right_motors_(false),
//     actual_velocities_(4, 0.0)
// {
// }

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

  // std::cout << "Zltech interface initialized on " << can_interface 
  //           << " with driver IDs 0x" << std::hex << driver1_id 
  //           << ", 0x" << driver2_id << std::dec << std::endl;
  
  // return true;

  // Calculate TPDO COB-IDs from node IDs
  // CANopen: TPDO0 COB-ID = 0x180 + node_id
  // driver1_id = 0x601 → node_id = 0x01 → TPDO = 0x181
  // driver2_id = 0x602 → node_id = 0x02 → TPDO = 0x182
  driver1_tpdo_id_ = 0x180 + (driver1_id & 0x7F);
  driver2_tpdo_id_ = 0x180 + (driver2_id & 0x7F);

  std::cout << C_ACTIVE "[ZLTECH]" C_RESET
            << " Initialized on " << can_interface
            << " driver IDs 0x" << std::hex << driver1_id
            << "/0x" << driver2_id
            << " TPDO IDs 0x" << driver1_tpdo_id_
            << "/0x" << driver2_tpdo_id_ << std::dec << std::endl;

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

  // std::cout << "Zltech drivers initiated" << std::endl;
  // return true;
  std::cout << C_ACTIVE "[ZLTECH]" C_RESET
            << " Drivers initiated — configuring TPDO..." << std::endl;

  // Configure TPDO0 on each driver for automatic velocity broadcast
  if (!configureTPDO(driver1_id_, driver1_tpdo_id_)) {
    std::cout << C_WARN "[ZLTECH]" C_RESET
              << " TPDO config failed for driver1 — will use SDO fallback" << std::endl;
  }
  usleep(20000);
  if (!configureTPDO(driver2_id_, driver2_tpdo_id_)) {
    std::cout << C_WARN "[ZLTECH]" C_RESET
              << " TPDO config failed for driver2 — will use SDO fallback" << std::endl;
  }
  usleep(20000);

  // Send NMT start command to enable PDO on all nodes
  // COB-ID 0x000, data: 01 00 (start all nodes)
  std::vector<uint8_t> nmt_start = {0x01, 0x00};
  if (!sendCANMessage(0x000, nmt_start)) {
    std::cout << C_WARN "[ZLTECH]" C_RESET
              << " NMT start command failed" << std::endl;
  }
  usleep(10000);

  std::cout << C_ACTIVE "[ZLTECH]" C_RESET
            << " Drivers ready — TPDO configured" << std::endl;
  return true;
}

bool ZltechInterface::configureTPDO(uint16_t driver_id, uint16_t tpdo_id)
{
  // Per Zltech manual section 6.1 TPDO MAPPING
  // Map 0x606C sub-index 03 (combined left+right velocity) to TPDO0
  // TPDO0 will broadcast on COB-ID 0x180 + node_id automatically

  // Step 1 — clear existing TPDO0 mapping
  std::vector<uint8_t> clear_map = {0x2F, 0x00, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00};
  if (!sendCANMessage(driver_id, clear_map)) return false;
  usleep(10000);

  // Step 2 — map 0x606C sub-index 03 to TPDO0 mapping 1
  // 0x606C = velocity feedback, sub 03 = combined L+R, 0x20 = 32 bits
  std::vector<uint8_t> map_vel = {0x23, 0x00, 0x1A, 0x01, 0x20, 0x03, 0x6C, 0x60};
  if (!sendCANMessage(driver_id, map_vel)) return false;
  usleep(10000);

  // Step 3 — set transmission type to timer trigger (0xFF = event, 0xFE = timer)
  std::vector<uint8_t> set_timer = {0x2F, 0x00, 0x18, 0x02, 0xFF, 0x00, 0x00, 0x00};
  if (!sendCANMessage(driver_id, set_timer)) return false;
  usleep(10000);

  // Step 4 — set event timer to 100ms (0x64 = 100 in ms units)
  std::vector<uint8_t> set_interval = {0x2B, 0x00, 0x18, 0x05, 0x64, 0x00, 0x00, 0x00};
  if (!sendCANMessage(driver_id, set_interval)) return false;
  usleep(10000);

  // Step 5 — enable 1 mapping on TPDO0
  std::vector<uint8_t> enable_map = {0x2F, 0x00, 0x1A, 0x00, 0x01, 0x00, 0x00, 0x00};
  if (!sendCANMessage(driver_id, enable_map)) return false;
  usleep(10000);

  // Step 6 — save to EEPROM so config persists across power cycles
  std::vector<uint8_t> save = {0x2B, 0x10, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
  if (!sendCANMessage(driver_id, save)) return false;
  usleep(50000);  // EEPROM write needs longer delay

  std::cout << C_THREAD "[TPDO]" C_RESET
            << " Driver 0x" << std::hex << driver_id
            << " configured → broadcasting on 0x" << tpdo_id
            << std::dec << std::endl;
  return true;
}

void ZltechInterface::startReceiveThread()
{
  if (thread_running_) return;
  thread_running_ = true;
  receive_thread_ = std::thread(&ZltechInterface::receiveLoop, this);
  std::cout << C_THREAD C_BOLD "[THREAD]" C_RESET
            << " Zltech receive thread started on " << can_interface_ << std::endl;
}

void ZltechInterface::stopReceiveThread()
{
  if (!thread_running_) return;
  thread_running_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  std::cout << C_THREAD C_BOLD "[THREAD]" C_RESET
            << " Zltech receive thread stopped" << std::endl;
}

void ZltechInterface::receiveLoop()
{
  struct can_frame frame;

  while (thread_running_) {
    if (can_socket_ < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    // Non-blocking read
    ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));

    if (nbytes == sizeof(frame)) {
      // Only process standard frames (Zltech TPDO)
      // Skip extended frames (those are CubeMars on the same bus)
      if (frame.can_id & CAN_EFF_FLAG) continue;

      uint16_t cob_id = frame.can_id & 0x7FF;

      // Only process TPDO frames from our two drivers
      if (cob_id != driver1_tpdo_id_ && cob_id != driver2_tpdo_id_) continue;

      // TPDO data layout from manual section 6.1:
      // data[0-1]: left motor speed  (int16, 0.1 RPM units, little-endian)
      // data[2-3]: right motor speed (int16, 0.1 RPM units, little-endian)
      if (frame.can_dlc < 4) continue;

      int16_t left_raw  = static_cast<int16_t>(frame.data[0] | (frame.data[1] << 8));
      int16_t right_raw = static_cast<int16_t>(frame.data[2] | (frame.data[3] << 8));

      // Convert from 0.1 RPM units to RPM
      double left_rpm  = left_raw  * 0.1;
      double right_rpm = right_raw * 0.1;

      std::lock_guard<std::mutex> lock(velocities_mutex_);

      if (cob_id == driver1_tpdo_id_) {
        // Driver 1 = left side: FL=index 0, RL=index 2
        actual_velocities_[0] = -left_rpm;   // FL (inverted — left side)
        actual_velocities_[2] = -right_rpm;  // RL (inverted — left side)
      } else {
        // Driver 2 = right side: FR=index 1, RR=index 3
        actual_velocities_[1] = left_rpm;    // FR
        actual_velocities_[3] = right_rpm;   // RR
      }

    } else {
      std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
  }
}

void ZltechInterface::getActualVelocities(std::vector<double>& velocities_rpm)
{
  std::lock_guard<std::mutex> lock(velocities_mutex_);
  velocities_rpm = actual_velocities_;
}

bool ZltechInterface::setVelocities(const std::vector<double>& velocities_rpm)
{
  if (velocities_rpm.size() != 4) return false;

  // FL, FR, RL, RR -> Driver1(FL, RL), Driver2(FR, RR)
  int16_t left_front = static_cast<int16_t>(velocities_rpm[0]);
  int16_t left_rear = static_cast<int16_t>(velocities_rpm[2]);
  int16_t right_front = static_cast<int16_t>(velocities_rpm[1]);
  int16_t right_rear = static_cast<int16_t>(velocities_rpm[3]);

//if USE_ZLAC8015D_POLARITY
  // Current ZLAC8015D setup: invert the right-side motors.
  if (invert_right_motors_) {
    right_front = -right_front;
    right_rear = -right_rear;
  }
//else
  // Previous setup (older reference kept here for easy swap):
  // Invert the left-side driver commands to match the earlier ZLAC8030D behavior.
  // left_front = -left_front;
  // left_rear = -left_rear;
//endif

  // Send to driver 1 (left side)
  if (!sendVelocityCommand(driver1_id_, left_front, left_rear)) return false;

  // Send to driver 2 (right side)
  if (!sendVelocityCommand(driver2_id_, right_front, right_rear)) return false;

  return true;
}

// Older reference block kept for easy swap back:
// bool ZltechInterface::setVelocities(const std::vector<double>& velocities_rpm)
// {
//   if (velocities_rpm.size() != 4) return false;

//   // FL=0, FR=1, RL=2, RR=3
//   int16_t left_front = static_cast<int16_t>(velocities_rpm[0]);
//   int16_t left_rear  = static_cast<int16_t>(velocities_rpm[2]);
//   int16_t right_front = static_cast<int16_t>(velocities_rpm[1]);
//   int16_t right_rear  = static_cast<int16_t>(velocities_rpm[3]);

//   // LEFT side needs inversion (driver1/601 goes backward with positive)
//   left_front = -left_front;
//   left_rear  = -left_rear;

//   // Driver 1 (601) = LEFT side
//   if (!sendVelocityCommand(driver1_id_, left_front, left_rear)) return false;

//   // Driver 2 (602) = RIGHT side
//   if (!sendVelocityCommand(driver2_id_, right_front, right_rear)) return false;

//   return true;
// }

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
// {
//   if (can_socket_ >= 0) {
//     stopMotors();
//     close(can_socket_);
//     can_socket_ = -1;
//     std::cout << "Zltech interface shutdown" << std::endl;
//   }
// }

{
  stopReceiveThread();
  if (can_socket_ >= 0) {
    stopMotors();
    close(can_socket_);
    can_socket_ = -1;
    std::cout << C_WARN "[ZLTECH]" C_RESET
              << " Interface shutdown" << std::endl;
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
