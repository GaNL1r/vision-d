#include "uvc_calibration.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>

namespace calibration
{
Chessboard load_chessboard(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  Chessboard board{
    {yaml["pattern_cols"].as<int>(), yaml["pattern_rows"].as<int>()},
    yaml["center_distance_mm"].as<double>(),
    yaml["min_views"].as<int>(12)};
  if (
    board.pattern_size.width < 2 || board.pattern_size.height < 2 ||
    !std::isfinite(board.square_size_mm) || board.square_size_mm <= 0 || board.min_views < 6) {
    throw std::runtime_error(
      "Invalid chessboard: need >=2 inner corners per axis, spacing >0, min_views >=6");
  }
  return board;
}

std::vector<cv::Point3f> board_points(const Chessboard & board)
{
  std::vector<cv::Point3f> points;
  for (int row = 0; row < board.pattern_size.height; ++row) {
    for (int col = 0; col < board.pattern_size.width; ++col) {
      points.emplace_back(col * board.square_size_mm, row * board.square_size_mm, 0);
    }
  }
  return points;
}

bool find_corners(
  const cv::Mat & image, const Chessboard & board, std::vector<cv::Point2f> & corners)
{
  cv::Mat gray;
  if (image.channels() == 1) {
    gray = image;
  } else {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  }
  // SB directly returns subpixel corners; capture and offline calibration use the same detector.
  return cv::findChessboardCornersSB(
    gray, board.pattern_size, corners, cv::CALIB_CB_NORMALIZE_IMAGE);
}

Intrinsics calibrate_uvc(
  const std::vector<Observation> & observations, cv::Size image_size, const Chessboard & board,
  const std::string & model)
{
  if (model != "pinhole" && model != "fisheye") {
    throw std::invalid_argument("Camera model must be pinhole or fisheye");
  }
  if (
    image_size.width <= 0 || image_size.height <= 0 ||
    observations.size() < static_cast<size_t>(board.min_views)) {
    throw std::invalid_argument("Invalid image size or insufficient chessboard views");
  }
  std::vector<std::vector<cv::Point2f>> image_points;
  for (const auto & observation : observations) {
    if (observation.corners.size() != static_cast<size_t>(board.pattern_size.area())) {
      throw std::invalid_argument("Incomplete chessboard observation: " + observation.image_path);
    }
    for (const auto & point : observation.corners) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
        throw std::invalid_argument("Non-finite chessboard coordinates");
      }
    }
    image_points.push_back(observation.corners);
  }
  std::vector<std::vector<cv::Point3f>> object_points(observations.size(), board_points(board));
  Intrinsics result;
  result.model = model;
  result.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  auto criteria = cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-9);
  if (model == "fisheye") {
    result.flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_CHECK_COND |
                   cv::fisheye::CALIB_FIX_SKEW;
    result.rms_px = cv::fisheye::calibrate(
      object_points, image_points, image_size, result.camera_matrix, result.distort_coeffs,
      result.rvecs, result.tvecs, result.flags, criteria);
  } else {
    // Wide-angle pinhole model: estimate k3 as well as k1, k2, p1, p2.
    result.flags = 0;
    result.rms_px = cv::calibrateCamera(
      object_points, image_points, image_size, result.camera_matrix, result.distort_coeffs,
      result.rvecs, result.tvecs, result.flags, criteria);
  }
  if (
    !std::isfinite(result.rms_px) || !cv::checkRange(result.camera_matrix) ||
    !cv::checkRange(result.distort_coeffs) || result.camera_matrix.at<double>(0, 0) <= 0 ||
    result.camera_matrix.at<double>(1, 1) <= 0) {
    throw std::runtime_error("Calibration returned invalid intrinsics");
  }
  for (size_t i = 0; i < observations.size(); ++i) {
    std::vector<cv::Point2f> projected;
    if (model == "fisheye") {
      cv::fisheye::projectPoints(
        object_points[i], projected, result.rvecs[i], result.tvecs[i], result.camera_matrix,
        result.distort_coeffs);
    } else {
      cv::projectPoints(
        object_points[i], result.rvecs[i], result.tvecs[i], result.camera_matrix,
        result.distort_coeffs, projected);
    }
    double squared_error = cv::norm(image_points[i], projected, cv::NORM_L2SQR);
    result.per_view_rms_px.push_back(std::sqrt(squared_error / projected.size()));
  }
  return result;
}

cv::Mat make_undistortion_maps(
  const Intrinsics & result, cv::Size image_size, cv::Mat & map_x, cv::Mat & map_y)
{
  cv::Mat new_camera_matrix;
  if (result.model == "fisheye") {
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
      result.camera_matrix, result.distort_coeffs, image_size, cv::Mat::eye(3, 3, CV_64F),
      new_camera_matrix, 1.0);
    cv::fisheye::initUndistortRectifyMap(
      result.camera_matrix, result.distort_coeffs, cv::Mat::eye(3, 3, CV_64F), new_camera_matrix,
      image_size, CV_32FC1, map_x, map_y);
  } else {
    new_camera_matrix = cv::getOptimalNewCameraMatrix(
      result.camera_matrix, result.distort_coeffs, image_size, 1.0, image_size);
    cv::initUndistortRectifyMap(
      result.camera_matrix, result.distort_coeffs, cv::Mat(), new_camera_matrix, image_size,
      CV_32FC1, map_x, map_y);
  }
  if (
    !cv::checkRange(new_camera_matrix) || new_camera_matrix.at<double>(0, 0) <= 0 ||
    new_camera_matrix.at<double>(1, 1) <= 0 || !cv::checkRange(map_x) || !cv::checkRange(map_y)) {
    throw std::runtime_error(
      "Invalid undistortion maps; inspect calibration model and image coverage");
  }
  return new_camera_matrix;
}
}  // namespace calibration
