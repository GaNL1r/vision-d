#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <utility>

#include "calibration/uvc_calibration.hpp"

namespace fs = std::filesystem;

const std::string keys =
  "{help h usage ?  |                              | 输出命令行参数说明}"
  "{config-path c   | configs/uvc_calibration.yaml | 棋盘格配置文件}"
  "{model           | both                         | pinhole/fisheye/both，不自动选模型}"
  "{output-folder o |                              | 输出目录，默认输入目录下results}"
  "{previews        | true                         | 保存原图和去畸变图并排预览}"
  "{@input-folder   |                              | 单台相机标定图片目录}";

namespace
{
std::vector<double> values(const cv::Mat & matrix)
{
  return {matrix.begin<double>(), matrix.end<double>()};
}

void write_yaml(const fs::path & path, const YAML::Node & node)
{
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(17);
  emitter << node;
  std::ofstream stream(path);
  stream << emitter.c_str() << '\n';
  stream.close();
  if (!stream) throw std::runtime_error("Cannot write " + path.string());
}

void save_result(
  const fs::path & output, const calibration::Intrinsics & result, cv::Size image_size,
  const calibration::Chessboard & board, const std::vector<calibration::Observation> & observations,
  const YAML::Node & rejected, const YAML::Node & capture_metadata, bool previews)
{
  YAML::Node yaml;
  yaml["camera_model"] = result.model;
  yaml["image_width"] = image_size.width;
  yaml["image_height"] = image_size.height;
  yaml["camera_matrix"] = values(result.camera_matrix);
  yaml["camera_matrix"].SetStyle(YAML::EmitterStyle::Flow);
  yaml["distort_coeffs"] = values(result.distort_coeffs);
  yaml["distort_coeffs"].SetStyle(YAML::EmitterStyle::Flow);
  yaml["distortion_order"] = result.model == "fisheye" ? "k1,k2,k3,k4" : "k1,k2,p1,p2,k3";
  yaml["rms_reprojection_error_px"] = result.rms_px;
  yaml["valid_images"] = observations.size();
  yaml["pattern_cols"] = board.pattern_size.width;
  yaml["pattern_rows"] = board.pattern_size.height;
  yaml["center_distance_mm"] = board.square_size_mm;
  yaml["opencv_version"] = CV_VERSION;
  yaml["calibration_flags"] = result.flags;
  yaml["rejected_images"] = rejected;
  if (capture_metadata.IsMap()) yaml["capture"] = capture_metadata;
  for (size_t i = 0; i < observations.size(); ++i) {
    YAML::Node view;
    view["image"] = observations[i].image_path;
    view["rms_reprojection_error_px"] = result.per_view_rms_px[i];
    for (const auto & point : observations[i].corners) {
      YAML::Node xy;
      xy.push_back(point.x);
      xy.push_back(point.y);
      xy.SetStyle(YAML::EmitterStyle::Flow);
      view["corners_px"].push_back(xy);
    }
    yaml["views"].push_back(view);
  }
  // Save useful intrinsics even if preview generation fails for an unsuitable model.
  auto result_path = output / (result.model + ".yaml");
  write_yaml(result_path, yaml);
  if (!previews) return;
  cv::Mat map_x, map_y;
  auto new_matrix = calibration::make_undistortion_maps(result, image_size, map_x, map_y);
  yaml["preview_camera_matrix"] = values(new_matrix);
  yaml["preview_camera_matrix"].SetStyle(YAML::EmitterStyle::Flow);
  yaml["preview_description"] =
    "Left: original. Right: rectified pinhole image, same resolution; black borders are unmapped "
    "pixels. balance/alpha=1.";
  write_yaml(result_path, yaml);
  auto preview_folder = output / (result.model + "_previews");
  fs::create_directories(preview_folder);
  for (size_t i = 0; i < observations.size(); ++i) {
    auto original = cv::imread(observations[i].image_path);
    if (original.empty() || original.size() != image_size)
      throw std::runtime_error("Input image changed during calibration");
    cv::Mat undistorted;
    cv::remap(original, undistorted, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::Mat preview(image_size.height + 40, image_size.width * 2, CV_8UC3, cv::Scalar(0, 0, 0));
    original.copyTo(preview(cv::Rect(0, 40, image_size.width, image_size.height)));
    undistorted.copyTo(
      preview(cv::Rect(image_size.width, 40, image_size.width, image_size.height)));
    cv::putText(
      preview, "Original", {10, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.7, {255, 255, 255}, 1,
      cv::LINE_AA);
    cv::putText(
      preview, result.model + " undistorted", {image_size.width + 10, 28}, cv::FONT_HERSHEY_SIMPLEX,
      0.7, {255, 255, 255}, 1, cv::LINE_AA);
    auto path = preview_folder / fmt::format("{:04d}.png", i + 1);
    if (!cv::imwrite(path.string(), preview))
      throw std::runtime_error("Cannot write preview " + path.string());
  }
}
}  // namespace

int main(int argc, char * argv[])
{
  try {
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
      cli.printMessage();
      return 0;
    }
    auto config = cli.get<std::string>("config-path");
    auto input = cli.get<std::string>(0);
    auto output = cli.get<std::string>("output-folder");
    auto model = cli.get<std::string>("model");
    auto previews = cli.get<bool>("previews");
    if (!cli.check()) {
      cli.printErrors();
      return 1;
    }
    if (model != "both" && model != "pinhole" && model != "fisheye") {
      throw std::runtime_error("model must be pinhole, fisheye or both");
    }
    if (input.empty() || !fs::is_directory(input))
      throw std::runtime_error("Provide an image directory");
    if (output.empty()) output = (fs::path(input) / "results").string();
    if (fs::exists(output) && !fs::is_empty(output))
      throw std::runtime_error("Use a new or empty output directory: " + output);
    auto board = calibration::load_chessboard(config);
    YAML::Node metadata;
    if (fs::exists(fs::path(input) / "capture.yaml")) {
      metadata = YAML::LoadFile((fs::path(input) / "capture.yaml").string());
      if (
        metadata["pattern_cols"].as<int>() != board.pattern_size.width ||
        metadata["pattern_rows"].as<int>() != board.pattern_size.height ||
        std::abs(metadata["center_distance_mm"].as<double>() - board.square_size_mm) > 1e-9) {
        throw std::runtime_error("Board config differs from capture.yaml");
      }
    }
    std::vector<fs::path> paths;
    for (const auto & entry : fs::directory_iterator(input)) {
      if (!entry.is_regular_file()) continue;
      auto extension = entry.path().extension().string();
      std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char c) {
        return std::tolower(c);
      });
      if (
        extension == ".png" || extension == ".jpg" || extension == ".jpeg" || extension == ".bmp" ||
        extension == ".tif" || extension == ".tiff")
        paths.push_back(entry.path());
    }
    std::sort(paths.begin(), paths.end());
    if (paths.empty()) throw std::runtime_error("No supported images in " + input);
    cv::Size image_size;
    std::vector<calibration::Observation> observations;
    YAML::Node rejected(YAML::NodeType::Sequence);
    for (const auto & path : paths) {
      auto image = cv::imread(path.string());
      std::string reason;
      if (image.empty()) {
        reason = "unreadable image";
      } else {
        if (image_size.empty()) image_size = image.size();
        if (image.size() != image_size)
          throw std::runtime_error("Mixed image resolutions: " + path.string());
        if (
          metadata.IsMap() && (metadata["image_width"].as<int>() != image.cols ||
                               metadata["image_height"].as<int>() != image.rows))
          throw std::runtime_error("Image size differs from capture.yaml");
        std::vector<cv::Point2f> corners;
        if (calibration::find_corners(image, board, corners)) {
          observations.push_back({fs::absolute(path).string(), std::move(corners)});
          fmt::print("[accepted] {}\n", path.string());
          continue;
        }
        reason = "complete chessboard not detected";
      }
      YAML::Node record;
      record["image"] = fs::absolute(path).string();
      record["reason"] = reason;
      rejected.push_back(record);
      fmt::print("[rejected] {}: {}\n", path.string(), reason);
    }
    if (observations.size() < static_cast<size_t>(board.min_views)) {
      throw std::runtime_error(
        fmt::format("Only {} valid views; need at least {}", observations.size(), board.min_views));
    }
    fs::create_directories(output);
    std::vector<std::string> models = model == "both"
                                        ? std::vector<std::string>{"pinhole", "fisheye"}
                                        : std::vector<std::string>{model};
    bool failed = false;
    for (const auto & name : models) {
      try {
        auto result = calibration::calibrate_uvc(observations, image_size, board, name);
        fmt::print(
          "{}: {} views, training RMS {:.4f} px\n", name, observations.size(), result.rms_px);
        for (size_t i = 0; i < observations.size(); ++i) {
          fmt::print("  {:.4f} px  {}\n", result.per_view_rms_px[i], observations[i].image_path);
        }
        save_result(output, result, image_size, board, observations, rejected, metadata, previews);
      } catch (const std::exception & error) {
        fmt::print(
          stderr, "{} failed: {}\nUse varied, sharp board views covering the image.\n", name,
          error.what());
        failed = true;
      }
    }
    fmt::print(
      "Output: {}\nRMS is training error; compare edge straightness and independent images before "
      "choosing a model.\n",
      output);
    return failed ? 1 : 0;
  } catch (const std::exception & error) {
    fmt::print(stderr, "calibrate_uvc: {}\n", error.what());
    return 1;
  }
}
