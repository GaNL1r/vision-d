#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <list>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <vector>

#include "io/camera.hpp"
#include "io/camera_config.hpp"
#include "io/gimbal/gimbal.hpp"
#ifdef SRM_VISION_WITH_ROS2
#include "io/ros2/ros2.hpp"
#endif
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"
#include "tasks/omniperception/perceptron.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{debug d          | false               | 启用调试可视化}"
  "{detection-view   | false               | 仅显示原始检测状态}"
  "{view-camera      | front               | 检测画面: front/left/right}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

namespace
{
std::vector<const io::CameraConfig *> validate_camera_topology(
  const std::vector<io::CameraConfig> & cameras)
{
  if (cameras.size() != 3) {
    throw std::runtime_error("Sentry requires exactly three cameras: front, left and right");
  }

  const auto & front = io::camera_config_for_role(cameras, "front");
  if (front.type != "hikrobot") {
    throw std::runtime_error("The front camera must be a Hikrobot camera");
  }

  std::vector<const io::CameraConfig *> uvc_cameras;
  for (const auto * role : {"left", "right"}) {
    const auto & camera = io::camera_config_for_role(cameras, role);
    if (camera.type != "uvc") {
      throw std::runtime_error("The " + std::string(role) + " camera must be a UVC camera");
    }
    uvc_cameras.push_back(&camera);
  }
  return uvc_cameras;
}

cv::Mat make_detection_view(
  const cv::Mat & source, const std::list<auto_aim::Armor> & armors,
  const std::string & camera_role)
{
  constexpr int view_width = 960;
  constexpr int view_height = 600;
  constexpr int header_height = 56;
  cv::Mat view(view_height, view_width, CV_8UC3, cv::Scalar{24, 24, 24});

  auto detected = !armors.empty();
  auto status_color = detected ? cv::Scalar{40, 210, 80} : cv::Scalar{40, 40, 220};
  cv::circle(view, {25, header_height / 2}, 10, status_color, -1);
  tools::draw_text(
    view,
    fmt::format(
      "{}  {}  raw detections: {}", camera_role, detected ? "DETECTED" : "NO DETECTION",
      armors.size()),
    {48, 37}, status_color, 0.8, 2);

  if (source.empty()) {
    tools::draw_text(
      view, fmt::format("WAITING FOR {} FRAME", camera_role), {270, 320}, {180, 180, 180}, 0.8,
      2);
    return view;
  }

  auto annotated = source.clone();
  for (const auto & armor : armors) {
    tools::draw_points(annotated, armor.points, {0, 255, 255});
    auto label = fmt::format("{} {:.2f}", auto_aim::ARMOR_NAMES[armor.name], armor.confidence);
    auto label_point = cv::Point{static_cast<int>(armor.center.x), static_cast<int>(armor.center.y)};
    tools::draw_text(annotated, label, label_point, {0, 255, 255}, 0.7, 2);
  }

  auto image_height = view_height - header_height;
  auto scale = std::min(
    static_cast<double>(view_width) / annotated.cols,
    static_cast<double>(image_height) / annotated.rows);
  cv::Mat resized;
  cv::resize(annotated, resized, {}, scale, scale);
  auto x = (view_width - resized.cols) / 2;
  auto y = header_height + (image_height - resized.rows) / 2;
  resized.copyTo(view(cv::Rect{x, y, resized.cols, resized.rows}));
  return view;
}
}  // namespace

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Recorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto debug = cli.get<bool>("debug");
  auto detection_view = cli.get<bool>("detection-view");
  auto view_camera = cli.get<std::string>("view-camera");
  auto yaml = tools::load(config_path);
  auto gimbal_command_enabled =
    yaml["gimbal_command_enabled"] ? yaml["gimbal_command_enabled"].as<bool>() : true;
  auto camera_configs = io::load_camera_configs(config_path);
  auto uvc_camera_configs = validate_camera_topology(camera_configs);
  const auto & view_camera_config = io::camera_config_for_role(camera_configs, view_camera);

  if (!gimbal_command_enabled) {
    tools::logger()->warn("[Gimbal] Command output disabled by configuration.");
  }

  auto display_available = std::getenv("DISPLAY") != nullptr ||
                           std::getenv("WAYLAND_DISPLAY") != nullptr;
  auto show_debug_window = debug && display_available;
  auto show_detection_window = (debug || detection_view) && display_available;
  std::unique_ptr<tools::Plotter> plotter;
  if (debug) {
    plotter = std::make_unique<tools::Plotter>();
    if (!show_debug_window) {
      tools::logger()->warn(
        "[Debug] No display found; OpenCV window disabled, UDP plot output remains enabled.");
    }
  }
  if (detection_view && !display_available) {
    tools::logger()->warn("[Detection] No display found; detection window disabled.");
  }
  if (show_detection_window) {
    tools::logger()->info("[Detection] Showing raw detection view for '{}'.", view_camera);
  }

#ifdef SRM_VISION_WITH_ROS2
  io::ROS2 ros2;
#endif
  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);
  std::vector<std::unique_ptr<io::USBCamera>> uvc_cameras;
  std::vector<io::USBCamera *> uvc_camera_ptrs;
  for (const auto * camera_config : uvc_camera_configs) {
    auto camera = std::make_unique<io::USBCamera>(
      camera_config->device, config_path, camera_config->role);
    uvc_camera_ptrs.push_back(camera.get());
    uvc_cameras.push_back(std::move(camera));
  }

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  omniperception::Decider decider(config_path);
  auto uvc_debug_role =
    show_detection_window && view_camera_config.type == "uvc" ? view_camera : "";
  omniperception::Perceptron perceptron(uvc_camera_ptrs, config_path, uvc_debug_role);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  constexpr auto detection_hold_time = 500ms;
  std::chrono::steady_clock::time_point last_front_detection;
  bool front_detection_seen = false;
  bool detection_log_initialized = false;
  bool last_front_target_detected = false;
  bool last_omni_target_detected = false;

  while (!exiter.exit()) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = gimbal.q(timestamp - 1ms);
    recorder.record(img, q, timestamp);
    /// 自瞄核心逻辑
    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = yolo.detect(img);
    std::list<auto_aim::Armor> front_raw_armors;
    if (show_detection_window && view_camera == "front") front_raw_armors = armors;

    auto detection_time = std::chrono::steady_clock::now();
    if (!armors.empty()) {
      last_front_detection = detection_time;
      front_detection_seen = true;
    }

    decider.armor_filter(armors);

    decider.set_priority(armors);

    auto detection_queue = perceptron.get_detection_queue();

    decider.sort(detection_queue);

    auto front_target_detected =
      front_detection_seen && detection_time - last_front_detection <= detection_hold_time;
    auto omni_target_detected = perceptron.raw_target_detected(detection_hold_time);

    if (
      !detection_log_initialized || front_target_detected != last_front_target_detected ||
      omni_target_detected != last_omni_target_detected) {
      if (front_target_detected || omni_target_detected) {
        tools::logger()->info(
          "[Detection] Target detected (front={}, omni={}).", front_target_detected,
          omni_target_detected);
      } else {
        tools::logger()->info("[Detection] No target detected.");
      }
      detection_log_initialized = true;
      last_front_target_detected = front_target_detected;
      last_omni_target_detected = omni_target_detected;
    }

    auto [switch_target, targets] = tracker.track(detection_queue, armors, timestamp);

    io::Command command{false, false, 0, 0};

    /// 全向感知逻辑
    if (tracker.state() == "switching") {
      command.control = switch_target.armors.empty() ? false : true;
      command.shoot = false;
      command.pitch = tools::limit_rad(switch_target.delta_pitch);
      command.yaw = tools::limit_rad(switch_target.delta_yaw + gimbal_pos[0]);
    }

    else if (tracker.state() == "lost") {
      command = decider.decide(detection_queue);
      command.yaw = tools::limit_rad(command.yaw + gimbal_pos[0]);
    }

    else {
      command = aimer.aim(targets, timestamp, gimbal.state().bullet_speed);
    }

    /// 发射逻辑
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);
    // command.shoot = false;

    if (gimbal_command_enabled) {
      gimbal.send(command.control, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);
    }

#ifdef SRM_VISION_WITH_ROS2
    /// ROS2通信
    Eigen::Vector4d target_info = decider.get_target_info(armors, targets);

    ros2.publish(target_info);
#endif

    if (debug) {
      auto debug_img = img.clone();
      auto detection_source = front_target_detected && omni_target_detected
                                ? "front+omni"
                              : front_target_detected ? "front"
                              : omni_target_detected  ? "omni"
                                                      : "none";
      auto detection_color = front_target_detected || omni_target_detected
                               ? cv::Scalar{0, 255, 0}
                               : cv::Scalar{0, 0, 255};
      tools::draw_text(
        debug_img, fmt::format("[{}] target: {}", tracker.state(), detection_source), {10, 30},
        detection_color);

      for (const auto & armor : armors) {
        tools::draw_points(debug_img, armor.points, {0, 255, 255});
      }

      nlohmann::json data;
      data["target_detected"] = front_target_detected || omni_target_detected ? 1 : 0;
      data["front_target_detected"] = front_target_detected ? 1 : 0;
      data["omni_target_detected"] = omni_target_detected ? 1 : 0;
      data["armor_num"] = armors.size();

      if (!armors.empty()) {
        const auto * debug_armor = &armors.front();
        for (const auto & armor : armors) {
          if (armor.center.x < debug_armor->center.x) debug_armor = &armor;
        }
        auto solved_armor = *debug_armor;
        solver.solve(solved_armor);
        data["armor_x"] = solved_armor.xyz_in_world[0];
        data["armor_y"] = solved_armor.xyz_in_world[1];
        data["armor_yaw"] = solved_armor.ypr_in_world[0] * 57.3;
        data["armor_yaw_raw"] = solved_armor.yaw_raw * 57.3;
      }

      if (!targets.empty()) {
        auto target = targets.front();
        for (const auto & xyza : target.armor_xyza_list()) {
          auto image_points =
            solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          tools::draw_points(debug_img, image_points, {0, 255, 0});
        }

        if (tracker.state() != "switching") {
          auto aim_point = aimer.debug_aim_point;
          auto image_points = solver.reproject_armor(
            aim_point.xyza.head(3), aim_point.xyza[3], target.armor_type, target.name);
          tools::draw_points(
            debug_img, image_points, aim_point.valid ? cv::Scalar{0, 0, 255}
                                                     : cv::Scalar{255, 0, 0});
        }

        auto x = target.ekf_x();
        data["x"] = x[0];
        data["vx"] = x[1];
        data["y"] = x[2];
        data["vy"] = x[3];
        data["z"] = x[4];
        data["vz"] = x[5];
        data["a"] = x[6] * 57.3;
        data["w"] = x[7];
        data["r"] = x[8];
        data["l"] = x[9];
        data["h"] = x[10];
        data["last_id"] = target.last_id;
        data["residual_yaw"] = target.ekf().data.at("residual_yaw");
        data["residual_pitch"] = target.ekf().data.at("residual_pitch");
        data["residual_distance"] = target.ekf().data.at("residual_distance");
        data["residual_angle"] = target.ekf().data.at("residual_angle");
        data["nis"] = target.ekf().data.at("nis");
        data["nees"] = target.ekf().data.at("nees");
        data["nis_fail"] = target.ekf().data.at("nis_fail");
        data["nees_fail"] = target.ekf().data.at("nees_fail");
        data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
      }

      auto gimbal_state = gimbal.state();
      data["gimbal_yaw"] = gimbal_pos[0] * 57.3;
      data["gimbal_pitch"] = -gimbal_pos[1] * 57.3;
      data["bullet_speed"] = gimbal_state.bullet_speed;
      data["cmd_control"] = command.control ? 1 : 0;
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
      data["cmd_shoot"] = command.shoot ? 1 : 0;
      plotter->plot(data);

      if (show_debug_window) {
        cv::resize(debug_img, debug_img, {}, 0.5, 0.5);
        cv::imshow("sentry_multithread_debug", debug_img);
      }
    }

    if (show_detection_window) {
      cv::Mat selected_image;
      std::list<auto_aim::Armor> selected_armors;
      if (view_camera == "front") {
        selected_image = img;
        selected_armors = std::move(front_raw_armors);
      } else if (auto frame = perceptron.raw_detection_frame()) {
        if (std::chrono::steady_clock::now() - frame->timestamp <= 2s) {
          selected_image = frame->image;
          selected_armors = std::move(frame->armors);
        }
      }
      cv::imshow(
        "sentry_detection_view", make_detection_view(selected_image, selected_armors, view_camera));
    }

    if ((show_debug_window || show_detection_window) && cv::waitKey(1) == 'q') break;
  }

  if (show_debug_window) cv::destroyWindow("sentry_multithread_debug");
  if (show_detection_window) cv::destroyWindow("sentry_detection_view");

  return 0;
}
