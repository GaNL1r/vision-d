#include <stdexcept>
#include <string>
#include <vector>

#include "io/camera_config.hpp"

namespace
{
void expect_camera(
  const std::vector<io::CameraConfig> & cameras, const std::string & role,
  const std::string & type, const std::string & device = "")
{
  const auto & camera = io::camera_config_for_role(cameras, role);
  if (camera.type != type) {
    throw std::runtime_error(role + " camera type is " + camera.type + ", expected " + type);
  }
  if (camera.device != device) {
    throw std::runtime_error(
      role + " camera device is " + camera.device + ", expected " + device);
  }
}
}  // namespace

int main(int argc, char * argv[])
{
  auto config_path = argc > 1 ? argv[1] : "configs/sentry.yaml";
  auto cameras = io::load_camera_configs(config_path);
  if (cameras.size() != 3) {
    throw std::runtime_error("Expected three sentry cameras");
  }

  expect_camera(cameras, "front", "hikrobot");
  expect_camera(cameras, "left", "uvc", "video0");
  expect_camera(cameras, "right", "uvc", "video2");
  return 0;
}
