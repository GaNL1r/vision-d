#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
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

// 开火状态（画面显示用）：区分“自瞄判定要开火”与“实际发到下位机的开火位”
struct FireIndicator
{
  bool display_fire = false;  // 大字显示状态（FIRE 带保持时间，避免单帧闪烁）
  bool decision = false;      // 当前帧 command.shoot 判定
  bool sent = false;          // 当前帧实际发出的开火位（gimbal mode==2）
  double distance = -1.0;     // 目标水平距离(m)，无目标为 -1
  std::string title;          // 大字文字
  std::string detail;         // 明细文字（cv::putText 不支持中文，只能英文）
};

cv::Scalar fire_color(bool fire)
{
  return fire ? cv::Scalar{0, 0, 255} : cv::Scalar{150, 150, 150};
}

cv::Mat make_detection_view(
  const cv::Mat & source, const std::list<auto_aim::Armor> & armors,
  const std::string & camera_role, const FireIndicator & fire)
{
  constexpr int view_width = 960;
  constexpr int view_height = 600;
  constexpr int header_height = 118;
  cv::Mat view(view_height, view_width, CV_8UC3, cv::Scalar{24, 24, 24});

  auto detected = !armors.empty();
  auto status_color = detected ? cv::Scalar{40, 210, 80} : cv::Scalar{40, 40, 220};
  cv::circle(view, {25, 27}, 10, status_color, -1);
  tools::draw_text(
    view,
    fmt::format(
      "{}  {}  raw detections: {}", camera_role, detected ? "DETECTED" : "NO DETECTION",
      armors.size()),
    {48, 36}, status_color, 0.8, 2);

  // 开火状态：第一行大字，第二行明细（距离 / 是否真的发出 / 未开火原因）
  tools::draw_text(view, fire.title, {14, 80}, fire_color(fire.display_fire), 1.2, 3);
  auto detail = fire.detail;
  // 左右相机画面来自 perceptron 异步帧，开火状态是主循环最新帧的，标注避免误读
  if (camera_role != "front") detail += " | async frame";
  tools::draw_text(view, detail, {14, 108}, {205, 205, 205}, 0.55, 1);

  if (source.empty()) {
    tools::draw_text(
      view, fmt::format("WAITING FOR {} FRAME", camera_role), {270, 320}, {180, 180, 180}, 0.8, 2);
    return view;
  }

  auto annotated = source.clone();
  for (const auto & armor : armors) {
    tools::draw_points(annotated, armor.points, {0, 255, 255});
    auto label = fmt::format("{} {:.2f}", auto_aim::ARMOR_NAMES[armor.name], armor.confidence);
    auto label_point =
      cv::Point{static_cast<int>(armor.center.x), static_cast<int>(armor.center.y)};
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
  // 目标水平距离超过该值(m)时禁止开火，未配置时默认 5m
  auto max_shoot_distance =
    yaml["max_shoot_distance"] ? yaml["max_shoot_distance"].as<double>() : 5.0;
  // 仅用于画面显示未开火原因（与 shooter 内部读的是同一个键）
  auto auto_fire = yaml["auto_fire"] ? yaml["auto_fire"].as<bool>() : false;
  auto camera_configs = io::load_camera_configs(config_path);
  auto uvc_camera_configs = validate_camera_topology(camera_configs);
  const auto & view_camera_config = io::camera_config_for_role(camera_configs, view_camera);

  if (!gimbal_command_enabled) {
    tools::logger()->warn("[Gimbal] Command output disabled by configuration.");
  }

  auto display_available =
    std::getenv("DISPLAY") != nullptr || std::getenv("WAYLAND_DISPLAY") != nullptr;
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
    auto camera =
      std::make_unique<io::USBCamera>(camera_config->device, config_path, camera_config->role);
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

  // ========== 画面开火状态显示：FIRE 保持时间，避免单帧判定看不见 ==========
  constexpr auto fire_hold_time = 500ms;
  std::chrono::steady_clock::time_point last_fire_decision;
  bool fire_decision_seen = false;
  // ==================================================

  // ========== 锁定转向相关变量（switching 与 lost 通用） ==========
  double locked_target_yaw = 0.0;
  bool locked_turn = false;
  double locked_yaw_vel = 0.0;
  std::chrono::steady_clock::time_point lock_start;  // 进入锁定的时刻（超时计时起点）
  constexpr auto LOCK_TIMEOUT = 1500ms;  // 转向超时：超过则解锁退出（须 < tracker 的200帧≈3.3s）
  std::string prev_state = "lost";
  // ==================================================

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
    std::string current_state = tracker.state();

    // ========== 锁定转向状态机（switching / lost 通用） ==========
    // ① 触发锁定：switching 有切换目标，或 lost 时 decide 出 omni 目标
    if (current_state == "switching" && !locked_turn) {
      locked_target_yaw = tools::limit_rad(switch_target.delta_yaw + gimbal_pos[0]);
      locked_turn = true;
      lock_start = detection_time;
      tools::logger()->info("[Switching] 锁定目标方位: {}°", locked_target_yaw * 57.3);
    } else if (current_state == "lost" && !locked_turn) {
      io::Command dcmd = decider.decide(detection_queue);
      if (dcmd.control) {
        locked_target_yaw = tools::limit_rad(dcmd.yaw + gimbal_pos[0]);
        locked_turn = true;
        lock_start = detection_time;
        tools::logger()->info("[Lost] 锁定 omni 目标方位: {}°", locked_target_yaw * 57.3);
      }
    }
    // ② 离开 switching/lost（进入 tracking）：清除锁定
    if (current_state != "switching" && current_state != "lost" && locked_turn) {
      locked_turn = false;
      locked_yaw_vel = 0.0;
      tools::logger()->info("[Lock] 进入 tracking，清除锁定");
    }
    // ==================================================

    io::Command command{false, false, 0, 0};

    /// 全向感知逻辑
    if (locked_turn) {
      double yaw_error = locked_target_yaw - gimbal_pos[0];
      // ========== ① 到位解锁：转到目标位置后才交还控制权 ==========
      if (std::abs(yaw_error) < 0.03) {  // 约1.7°内认为到位
        locked_turn = false;
        locked_yaw_vel = 0.0;
        tools::logger()->info("[Lock] 转向到位({:.1f}°)，解锁", yaw_error * 57.3);
      }
      // ========== ② 超时退出：目标一直没被前视确认 → 放弃 ==========
      else if (detection_time - lock_start > LOCK_TIMEOUT) {
        locked_turn = false;
        locked_yaw_vel = 0.0;
        tools::logger()->warn("[Lock] 超时({}ms)未确认目标，解锁", LOCK_TIMEOUT.count());
      }
      // ========== ③ 转向中：只发锁定角，不接收其他指令 ==========
      else {
        command.control = true;
        command.shoot = false;
        command.pitch = gimbal_pos[1];  // 保持当前俯仰，不参与转向（修正：不用 delta_pitch 增量）
        const double Kp = 2.0;       // 比例增益，可调整
        const double max_vel = 2.0;  // 最大角速度 rad/s，可调整
        locked_yaw_vel = std::clamp(Kp * yaw_error, -max_vel, max_vel);
        // 死区：误差小于 0.03rad（约1.7°）认为到位
        if (std::abs(yaw_error) < 0.03) {
          locked_yaw_vel = 0.0;
        }
        command.yaw = locked_target_yaw;  // 用锁定的目标方位，不每帧更新
      }
    }
    // 未锁定：按状态正常处理
    if (!locked_turn) {
      if (current_state == "switching") {
        command.control = false;
        command.shoot = false;
        command.yaw = gimbal_pos[0];
        command.pitch = gimbal_pos[1];  // 保持当前俯仰，避免 pitch 归零
      } else if (current_state == "lost") {
        command = decider.decide(detection_queue);
        command.yaw = tools::limit_rad(command.yaw + gimbal_pos[0]);
      } else {
        command = aimer.aim(targets, timestamp, gimbal.state().bullet_speed);
      }
    }

    /// 发射逻辑
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);
    // command.shoot = false;

    /// 距离限制：目标水平距离超过 max_shoot_distance 时不开火
    auto target_distance = -1.0;  // 目标水平距离(m)，无目标保持 -1
    if (!targets.empty()) {
      auto target_x = targets.front().ekf_x();
      target_distance = std::sqrt(tools::square(target_x[0]) + tools::square(target_x[2]));
    }
    const bool blocked_by_distance = command.shoot && target_distance > max_shoot_distance;
    if (blocked_by_distance) command.shoot = false;

    // ========== 开火状态（画面显示：判定 vs 实际发出） ==========
    FireIndicator fire;
    fire.decision = command.shoot;  // 自瞄判定（已含 5m 闸门）
    // gimbal.send 里 mode = (control && fire) ? 1 : 0，只有 mode==1 才会下发开火位
    fire.sent = gimbal_command_enabled && command.control && command.shoot;
    fire.distance = target_distance;
    if (command.shoot) {
      fire_decision_seen = true;
      last_fire_decision = detection_time;
    }
    fire.display_fire = command.shoot || (fire_decision_seen &&
                                          detection_time - last_fire_decision <= fire_hold_time);
    fire.title = command.shoot ? "FIRE" : (fire.display_fire ? "FIRE (held)" : "NO FIRE");

    fire.detail = fire.decision ? "dec=1" : "dec=0";
    if (fire.distance >= 0) fire.detail += fmt::format(" | dist {:.2f}m", fire.distance);
    fire.detail += fire.sent ? " | sent=YES(mode1)" : " | sent=no";
    if (!fire.decision) {
      if (targets.empty())
        fire.detail += " | no target";
      else if (!command.control)
        fire.detail += " | not shootable state";
      else if (!auto_fire)
        fire.detail += " | auto_fire=false";
      else if (blocked_by_distance)
        fire.detail += fmt::format(" | {:.2f}m > {:.2f}m", target_distance, max_shoot_distance);
      else
        fire.detail += " | tolerance/aim not met";
    } else if (!fire.sent) {
      fire.detail += " | gimbal cmd disabled";
    }
    // ==================================================

    if (gimbal_command_enabled) {
      // ========== 锁定转向期间传计算出的角速度，而非 0 ==========
      double yaw_vel = locked_turn ? locked_yaw_vel : 0.0;
      gimbal.send(command.control, command.shoot, command.yaw, yaw_vel, 0, command.pitch, 0, 0);
      // ==================================================
    }

#ifdef SRM_VISION_WITH_ROS2
    /// ROS2通信
    Eigen::Vector4d target_info = decider.get_target_info(armors, targets);

    ros2.publish(target_info);
#endif

    if (debug) {
      auto debug_img = img.clone();
      auto detection_source = front_target_detected && omni_target_detected ? "front+omni"
                              : front_target_detected                       ? "front"
                              : omni_target_detected                        ? "omni"
                                                                            : "none";
      auto detection_color = front_target_detected || omni_target_detected ? cv::Scalar{0, 255, 0}
                                                                           : cv::Scalar{0, 0, 255};
      tools::draw_text(
        debug_img, fmt::format("[{}] target: {}", current_state, detection_source), {10, 30},
        detection_color);

      // 开火状态：第一行大字，第二行明细（距离 / 是否真的发出 / 未开火原因）
      tools::draw_text(debug_img, fire.title, {10, 85}, fire_color(fire.display_fire), 1.6, 4);
      tools::draw_text(debug_img, fire.detail, {10, 118}, {205, 205, 205}, 0.7, 2);

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

        if (current_state != "switching") {
          auto aim_point = aimer.debug_aim_point;
          auto image_points = solver.reproject_armor(
            aim_point.xyza.head(3), aim_point.xyza[3], target.armor_type, target.name);
          tools::draw_points(
            debug_img, image_points,
            aim_point.valid ? cv::Scalar{0, 0, 255} : cv::Scalar{255, 0, 0});
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
        "sentry_detection_view",
        make_detection_view(selected_image, selected_armors, view_camera, fire));
    }

    // ========== 更新 prev_state（必须在循环末尾） ==========
    prev_state = current_state;

    if ((show_debug_window || show_detection_window) && cv::waitKey(1) == 'q') break;
  }

  if (show_debug_window) cv::destroyWindow("sentry_multithread_debug");
  if (show_detection_window) cv::destroyWindow("sentry_detection_view");

  return 0;
}
