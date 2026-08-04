#include "camera_config.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "tools/yaml.hpp"

namespace io
{
namespace
{
template <typename T>
T required(const YAML::Node & node, const std::string & key, const std::string & role)
{
  if (!node[key]) {
    throw std::runtime_error("Camera '" + role + "' is missing '" + key + "'");
  }
  return node[key].as<T>();
}
}  // namespace

std::vector<CameraConfig> load_camera_configs(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_nodes = yaml["cameras"];
  if (!camera_nodes) return {};
  if (!camera_nodes.IsSequence()) {
    throw std::runtime_error("'cameras' must be a sequence");
  }

  std::vector<CameraConfig> cameras;
  std::unordered_set<std::string> roles;
  cameras.reserve(camera_nodes.size());

  for (const auto & node : camera_nodes) {
    if (!node.IsMap()) {
      throw std::runtime_error("Each entry in 'cameras' must be a map");
    }

    CameraConfig camera;
    camera.role = required<std::string>(node, "role", "unknown");
    camera.type = required<std::string>(node, "type", camera.role);
    camera.yaw_offset_deg = required<double>(node, "yaw_offset_deg", camera.role);
    camera.pitch_offset_deg = required<double>(node, "pitch_offset_deg", camera.role);

    if (!roles.insert(camera.role).second) {
      throw std::runtime_error("Duplicate camera role: " + camera.role);
    }

    if (camera.type == "uvc") {
      camera.device = required<std::string>(node, "device", camera.role);
      camera.fov_h_deg = required<double>(node, "fov_h_deg", camera.role);
      camera.fov_v_deg = required<double>(node, "fov_v_deg", camera.role);
      if (camera.fov_h_deg <= 0 || camera.fov_v_deg <= 0) {
        throw std::runtime_error("Camera '" + camera.role + "' must have a positive FOV");
      }
    } else if (camera.type == "hikrobot") {
      camera.vid_pid = required<std::string>(node, "vid_pid", camera.role);
      camera.exposure_ms = required<double>(node, "exposure_ms", camera.role);
      camera.gain = required<double>(node, "gain", camera.role);
    } else if (camera.type == "mindvision") {
      camera.vid_pid = required<std::string>(node, "vid_pid", camera.role);
      camera.exposure_ms = required<double>(node, "exposure_ms", camera.role);
      camera.gamma = required<double>(node, "gamma", camera.role);
    } else {
      throw std::runtime_error(
        "Camera '" + camera.role + "' has unsupported type '" + camera.type + "'");
    }

    cameras.push_back(std::move(camera));
  }

  return cameras;
}

const CameraConfig & camera_config_for_role(
  const std::vector<CameraConfig> & cameras, std::string_view role)
{
  for (const auto & camera : cameras) {
    if (camera.role == role) return camera;
  }
  throw std::runtime_error("Camera role not configured: " + std::string(role));
}

}  // namespace io
