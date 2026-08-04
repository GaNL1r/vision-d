#ifndef IO__CAMERA_CONFIG_HPP
#define IO__CAMERA_CONFIG_HPP

#include <string>
#include <string_view>
#include <vector>

namespace io
{
struct CameraConfig
{
  std::string role;
  std::string type;
  std::string device;
  std::string vid_pid;
  double exposure_ms = 0;
  double gain = 0;
  double gamma = 0;
  double yaw_offset_deg = 0;
  double pitch_offset_deg = 0;
  double fov_h_deg = 0;
  double fov_v_deg = 0;
};

std::vector<CameraConfig> load_camera_configs(const std::string & config_path);

const CameraConfig & camera_config_for_role(
  const std::vector<CameraConfig> & cameras, std::string_view role);

}  // namespace io

#endif  // IO__CAMERA_CONFIG_HPP
