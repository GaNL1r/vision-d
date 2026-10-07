#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "calibration/uvc_calibration.hpp"

namespace fs = std::filesystem;

const std::string keys =
  "{help h usage ?  |                              | 输出命令行参数说明}"
  "{config-path c   | configs/sentry.yaml          | 相机配置文件}"
  "{board-config b  | configs/uvc_calibration.yaml | 棋盘格配置文件}"
  "{camera          | left                         | UVC相机角色: left/right}"
  "{output-folder o |                              | 输出目录，默认按相机和时间创建}"
  "{preview-scale   | 0.75                         | 仅缩放预览，保存原始分辨率}"
  "{detect-board    | true                         | 预览检测棋盘格；保存时始终检测}";

int main(int argc, char * argv[])
{
  try {
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
      cli.printMessage();
      return 0;
    }
    auto config_path = cli.get<std::string>("config-path");
    auto board_path = cli.get<std::string>("board-config");
    auto role = cli.get<std::string>("camera");
    auto output = cli.get<std::string>("output-folder");
    auto scale = cli.get<double>("preview-scale");
    auto detect_board = cli.get<bool>("detect-board");
    if (!cli.check()) {
      cli.printErrors();
      return 1;
    }
    if (!std::isfinite(scale) || scale <= 0 || scale > 1) {
      throw std::runtime_error("preview-scale must be in (0, 1]");
    }
    auto board = calibration::load_chessboard(board_path);
    auto yaml = YAML::LoadFile(config_path);
    std::string device;
    for (const auto & camera : yaml["cameras"]) {
      if (camera["role"].as<std::string>() != role) continue;
      if (camera["type"].as<std::string>() != "uvc") {
        throw std::runtime_error("Selected camera is not UVC: " + role);
      }
      device = camera["device"].as<std::string>();
      break;
    }
    if (device.empty()) throw std::runtime_error("UVC camera role not found: " + role);
    if (device.rfind("/dev/", 0) != 0) device = "/dev/" + device;
    int width = yaml["image_width"].as<int>();
    int height = yaml["image_height"].as<int>();
    if (width <= 0 || height <= 0) throw std::runtime_error("Invalid capture resolution");
    if (!std::getenv("DISPLAY") && !std::getenv("WAYLAND_DISPLAY")) {
      throw std::runtime_error("Capture requires a display; offline calibrate_uvc does not");
    }
    if (output.empty()) {
      auto stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                     std::chrono::system_clock::now().time_since_epoch())
                     .count();
      output = fmt::format("assets/uvc_calibration/{}/{}", role, stamp);
    }
    if (fs::exists(output) && !fs::is_empty(output)) {
      throw std::runtime_error("Use a new or empty capture directory: " + output);
    }

    // Use the same V4L2/MJPG settings as io::USBCamera, without the gimbal or capture queue.
    cv::VideoCapture camera(device, cv::CAP_V4L2);
    if (!camera.isOpened()) throw std::runtime_error("Cannot open " + device);
    auto set_property = [&](int property, double value, const std::string & name) {
      if (!camera.set(property, value)) fmt::print(stderr, "Warning: failed to set {}\n", name);
    };
    set_property(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), "MJPG");
    set_property(cv::CAP_PROP_FRAME_WIDTH, width, "width");
    set_property(cv::CAP_PROP_FRAME_HEIGHT, height, "height");
    set_property(cv::CAP_PROP_FPS, yaml["usb_frame_rate"].as<double>(), "fps");
    set_property(cv::CAP_PROP_AUTO_EXPOSURE, 1, "manual exposure");
    set_property(cv::CAP_PROP_EXPOSURE, yaml["usb_exposure"].as<double>(), "exposure");
    set_property(cv::CAP_PROP_GAMMA, yaml["usb_gamma"].as<double>(), "gamma");
    set_property(cv::CAP_PROP_GAIN, yaml["usb_gain"].as<double>(), "gain");

    cv::Mat image;
    if (!camera.read(image) || image.empty()) throw std::runtime_error("Cannot read " + device);
    if (image.size() != cv::Size(width, height)) {
      throw std::runtime_error(fmt::format(
        "Requested {}x{}, received {}x{}; use the actual runtime capture mode", width, height,
        image.cols, image.rows));
    }
    fs::create_directories(output);
    YAML::Node metadata;
    metadata["camera_role"] = role;
    metadata["device"] = device;
    metadata["camera_config"] = fs::absolute(config_path).string();
    metadata["image_width"] = image.cols;
    metadata["image_height"] = image.rows;
    metadata["pattern_cols"] = board.pattern_size.width;
    metadata["pattern_rows"] = board.pattern_size.height;
    metadata["center_distance_mm"] = board.square_size_mm;
    metadata["reported_fps"] = camera.get(cv::CAP_PROP_FPS);
    metadata["reported_exposure"] = camera.get(cv::CAP_PROP_EXPOSURE);
    metadata["reported_gain"] = camera.get(cv::CAP_PROP_GAIN);
    metadata["reported_gamma"] = camera.get(cv::CAP_PROP_GAMMA);
    metadata["reported_focus"] = camera.get(cv::CAP_PROP_FOCUS);
    metadata["reported_autofocus"] = camera.get(cv::CAP_PROP_AUTOFOCUS);
    std::ofstream metadata_file(fs::path(output) / "capture.yaml");
    metadata_file << metadata << '\n';
    metadata_file.close();
    if (!metadata_file) throw std::runtime_error("Cannot write capture metadata");
    fmt::print(
      "{}: {}, {}x{}, reported {:.1f} fps\nChessboard: {}x{} inner corners, {} mm\n"
      "s: save detected board, q/Esc: quit. Output: {}\n",
      role, device, width, height, camera.get(cv::CAP_PROP_FPS), board.pattern_size.width,
      board.pattern_size.height, board.square_size_mm, output);

    int count = 0;
    const std::string window = "UVC calibration: s=save, q=quit";
    while (true) {
      if (image.size() != cv::Size(width, height)) throw std::runtime_error("Capture size changed");
      std::vector<cv::Point2f> corners;
      bool found = detect_board && calibration::find_corners(image, board, corners);
      cv::Mat drawing = image.clone();
      if (found) cv::drawChessboardCorners(drawing, board.pattern_size, corners, true);
      cv::resize(drawing, drawing, {}, scale, scale);
      cv::rectangle(drawing, {0, 0}, {drawing.cols, 42}, {0, 0, 0}, cv::FILLED);
      cv::putText(
        drawing,
        fmt::format(
          "{} | {} | saved {} | s:save q:quit", role,
          found ? "BOARD FOUND" : "NO BOARD / PREVIEW OFF", count),
        {8, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255, 255, 255}, 1, cv::LINE_AA);
      cv::imshow(window, drawing);
      auto key = cv::waitKey(1) & 0xff;
      if (key == 'q' || key == 27 || cv::getWindowProperty(window, cv::WND_PROP_VISIBLE) < 1) break;
      if (key == 's') {
        if (!detect_board) found = calibration::find_corners(image, board, corners);
        if (found) {
          auto path = fs::path(output) / fmt::format("{:04d}.png", count + 1);
          if (!cv::imwrite(path.string(), image))
            throw std::runtime_error("Cannot save " + path.string());
          fmt::print("Saved {}\n", path.string());
          ++count;
        } else {
          fmt::print("Not saved: complete chessboard not detected\n");
        }
      }
      if (!camera.read(image) || image.empty())
        throw std::runtime_error("UVC camera stopped delivering frames");
    }
    cv::destroyAllWindows();
    fmt::print("Saved {} images. Recommended: 20-30 varied views per camera.\n", count);
    return 0;
  } catch (const std::exception & error) {
    fmt::print(stderr, "capture_uvc: {}\n", error.what());
    return 1;
  }
}
