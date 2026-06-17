#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{
struct __attribute__((packed)) GimbalToVision
{
  float yaw;
  float pitch;
  float roll;
  int mode;
  int color;
};

static_assert(sizeof(GimbalToVision) == 20);

struct __attribute__((packed)) ShootToVision
{
  float bullet_speed;
};

static_assert(sizeof(ShootToVision) == 4);

struct __attribute__((packed)) VisionToGimbal
{
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
};

struct __attribute__((packed)) VisionGimbalCommand
{
  float yaw;
  float pitch;
};

static_assert(sizeof(VisionGimbalCommand) == 8);

struct __attribute__((packed)) VisionShootCommand
{
  int fire_flag;
};

static_assert(sizeof(VisionShootCommand) == 4);

enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF     // 大符
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

  void send(io::VisionToGimbal VisionToGimbal);

private:
  serial::Serial serial_;
  std::string configured_port_;
  std::vector<std::string> candidate_ports_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  GimbalToVision rx_data_;
  VisionToGimbal tx_data_;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};

  bool read(uint8_t * buffer, size_t size);
  bool open_serial();
  std::vector<std::string> make_candidate_ports(const std::string & configured_port) const;
  bool read_packet(std::vector<uint8_t> & payload);
  void handle_packet(const std::vector<uint8_t> & payload, std::chrono::steady_clock::time_point t);
  void read_thread();
  void reconnect();
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
