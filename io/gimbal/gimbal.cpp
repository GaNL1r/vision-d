#include "gimbal.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr int16_t GIMBAL_RECV_ID = 1;
constexpr int16_t SHOOT_RECV_ID = 2;
constexpr int16_t MIN_PAYLOAD_SIZE =
  sizeof(int16_t) + sizeof(GimbalToVision) + sizeof(int16_t) + sizeof(ShootToVision);
constexpr int16_t MAX_PAYLOAD_SIZE = 254;

template <typename T>
void append_value(std::vector<uint8_t> & buffer, const T & value)
{
  auto ptr = reinterpret_cast<const uint8_t *>(&value);
  buffer.insert(buffer.end(), ptr, ptr + sizeof(T));
}

template <typename T>
bool read_value(const std::vector<uint8_t> & buffer, size_t & pos, T & value)
{
  if (pos + sizeof(T) > buffer.size()) return false;
  std::memcpy(&value, buffer.data() + pos, sizeof(T));
  pos += sizeof(T);
  return true;
}

std::vector<std::string> glob_ports(const std::filesystem::path & dir, const std::string & prefix)
{
  std::vector<std::string> ports;
  std::error_code ec;
  if (!std::filesystem::exists(dir, ec)) return ports;

  for (const auto & entry : std::filesystem::directory_iterator(dir, ec)) {
    if (ec) break;
    auto path = entry.path().string();
    auto name = entry.path().filename().string();
    if (name.rfind(prefix, 0) == 0) ports.push_back(path);
  }

  std::sort(ports.begin(), ports.end());
  return ports;
}
}  // namespace

Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  configured_port_ = tools::read<std::string>(yaml, "com_port");
  candidate_ports_ = make_candidate_ports(configured_port_);

  serial_.setBaudrate(921600);
  serial_.setTimeout(20, 20, 0, 20, 0);

  if (!open_serial()) {
    tools::logger()->error("[Gimbal] Failed to open any serial port.");
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;

  VisionGimbalCommand gimbal_command{
    static_cast<float>(VisionToGimbal.yaw * RAD_TO_DEG),
    static_cast<float>(VisionToGimbal.pitch * RAD_TO_DEG)};
  VisionShootCommand shoot_command{VisionToGimbal.mode == 2 ? 1 : 0};

  std::vector<uint8_t> packet;
  int16_t payload_size = sizeof(int16_t) + sizeof(gimbal_command) + sizeof(int16_t) +
                         sizeof(shoot_command);
  packet.reserve(sizeof(payload_size) + payload_size);
  append_value(packet, payload_size);
  append_value(packet, GIMBAL_RECV_ID);
  append_value(packet, gimbal_command);
  append_value(packet, SHOOT_RECV_ID);
  append_value(packet, shoot_command);

  try {
    serial_.write(packet.data(), packet.size());
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  send(tx_data_);
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

bool Gimbal::open_serial()
{
  candidate_ports_ = make_candidate_ports(configured_port_);

  for (const auto & port : candidate_ports_) {
    try {
      if (serial_.isOpen()) serial_.close();
      serial_.setPort(port);
      serial_.open();
      serial_.flushInput();
      tools::logger()->info("[Gimbal] Opened serial: {}", port);
      return true;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Failed to open {}: {}", port, e.what());
    }
  }

  return false;
}

std::vector<std::string> Gimbal::make_candidate_ports(const std::string & configured_port) const
{
  std::vector<std::string> ports;
  auto add_port = [&](const std::string & port) {
    if (port.empty()) return;
    if (std::find(ports.begin(), ports.end(), port) == ports.end()) ports.push_back(port);
  };

  add_port(configured_port);
  add_port("/dev/gimbal");

  auto by_id_ports = glob_ports("/dev/serial/by-id", "");
  for (const auto & port : by_id_ports) add_port(port);

  auto acm_ports = glob_ports("/dev", "ttyACM");
  for (const auto & port : acm_ports) add_port(port);

  return ports;
}

bool Gimbal::read_packet(std::vector<uint8_t> & payload)
{
  int16_t payload_size = 0;
  if (!read(reinterpret_cast<uint8_t *>(&payload_size), sizeof(payload_size))) return false;

  if (payload_size < MIN_PAYLOAD_SIZE || payload_size > MAX_PAYLOAD_SIZE) {
    tools::logger()->debug("[Gimbal] Invalid SRM payload size: {}", payload_size);
    return false;
  }

  payload.resize(payload_size);
  return read(payload.data(), payload.size());
}

void Gimbal::handle_packet(
  const std::vector<uint8_t> & payload, std::chrono::steady_clock::time_point t)
{
  size_t pos = 0;
  GimbalToVision gimbal_data{};
  ShootToVision shoot_data{};
  bool has_gimbal_data = false;
  bool has_shoot_data = false;

  while (pos + sizeof(int16_t) <= payload.size()) {
    int16_t id = 0;
    if (!read_value(payload, pos, id)) return;

    if (id == GIMBAL_RECV_ID) {
      if (!read_value(payload, pos, gimbal_data)) return;
      has_gimbal_data = true;
    } else if (id == SHOOT_RECV_ID) {
      if (!read_value(payload, pos, shoot_data)) return;
      has_shoot_data = true;
    } else {
      tools::logger()->debug("[Gimbal] Unknown SRM message id: {}", id);
      return;
    }
  }

  if (!has_gimbal_data) return;

  auto yaw = static_cast<float>(gimbal_data.yaw * DEG_TO_RAD);
  auto pitch = static_cast<float>(gimbal_data.pitch * DEG_TO_RAD);
  auto roll = static_cast<float>(gimbal_data.roll * DEG_TO_RAD);
  Eigen::Quaterniond q =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  queue_.push({q.normalized(), t});

  std::lock_guard<std::mutex> lock(mutex_);
  state_.yaw = yaw;
  state_.yaw_vel = 0;
  state_.pitch = pitch;
  state_.pitch_vel = 0;
  if (has_shoot_data) state_.bullet_speed = shoot_data.bullet_speed;

  switch (gimbal_data.mode) {
    case 0:
      mode_ = GimbalMode::AUTO_AIM;
      break;
    case 1:
      mode_ = GimbalMode::AUTO_AIM;
      break;
    case 2:
      mode_ = GimbalMode::SMALL_BUFF;
      break;
    case 3:
      mode_ = GimbalMode::BIG_BUFF;
      break;
    default:
      mode_ = GimbalMode::IDLE;
      tools::logger()->warn("[Gimbal] Invalid mode: {}", gimbal_data.mode);
      break;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;
  std::vector<uint8_t> payload;

  while (!quit_) {
    if (error_count > 100) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    auto t = std::chrono::steady_clock::now();
    if (!read_packet(payload)) {
      error_count++;
      continue;
    }

    error_count = 0;
    handle_packet(payload, t);
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      if (!open_serial()) continue;
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io
