#include "camera.hpp"

#include <stdexcept>

#include "camera_config.hpp"
#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#include "tools/yaml.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto cameras = load_camera_configs(config_path);

  std::string camera_name;
  std::string vid_pid;
  double exposure_ms;
  double gamma = 0;
  double gain = 0;

  if (cameras.empty()) {
    camera_name = tools::read<std::string>(yaml, "camera_name");
    exposure_ms = tools::read<double>(yaml, "exposure_ms");
    vid_pid = tools::read<std::string>(yaml, "vid_pid");
    if (camera_name == "mindvision") gamma = tools::read<double>(yaml, "gamma");
    if (camera_name == "hikrobot") gain = tools::read<double>(yaml, "gain");
  } else {
    const auto & front_camera = camera_config_for_role(cameras, "front");
    camera_name = front_camera.type;
    exposure_ms = front_camera.exposure_ms;
    vid_pid = front_camera.vid_pid;
    gamma = front_camera.gamma;
    gain = front_camera.gain;
  }

  if (camera_name == "mindvision") {
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  }

  else if (camera_name == "hikrobot") {
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
  }

  else {
    throw std::runtime_error("Unknown camera type: " + camera_name);
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

}  // namespace io
