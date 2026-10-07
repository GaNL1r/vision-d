#include "calibration/uvc_calibration.hpp"

#include <fmt/core.h>

#include <cmath>
#include <stdexcept>

namespace
{
void expect(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}

void test_model(const std::string & model)
{
  calibration::Chessboard board{{11, 8}, 30, 12};
  cv::Size size(1280, 720);
  cv::Mat expected_k = (cv::Mat_<double>(3, 3) << 620, 0, 635, 0, 610, 365, 0, 0, 1);
  cv::Mat expected_d = model == "fisheye"
                         ? cv::Mat((cv::Mat_<double>(4, 1) << -0.03, 0.006, -0.001, 0.0002))
                         : cv::Mat((cv::Mat_<double>(5, 1) << -0.15, 0.04, 0.001, -0.002, 0.005));
  auto object_points = calibration::board_points(board);
  std::vector<calibration::Observation> observations;
  for (int i = 0; i < 24; ++i) {
    cv::Vec3d rotation(-0.4 + 0.16 * (i % 6), -0.35 + 0.23 * (i % 4), -0.2 + 0.1 * (i % 5));
    cv::Vec3d translation(-250 + 80 * (i % 4), -180 + 60 * (i % 3), 650 + 60 * (i % 5));
    std::vector<cv::Point2f> pixels;
    if (model == "fisheye") {
      cv::fisheye::projectPoints(
        object_points, pixels, rotation, translation, expected_k, expected_d);
    } else {
      cv::projectPoints(object_points, rotation, translation, expected_k, expected_d, pixels);
    }
    observations.push_back({fmt::format("synthetic_{}", i), pixels});
  }
  auto result = calibration::calibrate_uvc(observations, size, board, model);
  expect(result.rms_px < 0.001, model + ": unexpectedly large reprojection error");
  expect(
    cv::norm(result.camera_matrix, expected_k, cv::NORM_INF) < 0.05,
    model + ": did not recover known intrinsics");
  expect(
    cv::norm(result.distort_coeffs.reshape(1, 1), expected_d.reshape(1, 1), cv::NORM_INF) < 0.005,
    model + ": did not recover known distortion");
  expect(result.per_view_rms_px.size() == observations.size(), "Missing per-view errors");
  double squared_error = 0;
  for (auto error : result.per_view_rms_px) squared_error += error * error;
  expect(
    std::abs(std::sqrt(squared_error / observations.size()) - result.rms_px) < 0.0001,
    "Per-view errors disagree with global RMS");

  // An unseen pose checks the fitted projection, not just its training residuals.
  cv::Vec3d rotation(0.21, -0.18, 0.13), translation(-140, -85, 725);
  std::vector<cv::Point2f> expected_pixels, fitted_pixels;
  if (model == "fisheye") {
    cv::fisheye::projectPoints(
      object_points, expected_pixels, rotation, translation, expected_k, expected_d);
    cv::fisheye::projectPoints(
      object_points, fitted_pixels, rotation, translation, result.camera_matrix,
      result.distort_coeffs);
  } else {
    cv::projectPoints(
      object_points, rotation, translation, expected_k, expected_d, expected_pixels);
    cv::projectPoints(
      object_points, rotation, translation, result.camera_matrix, result.distort_coeffs,
      fitted_pixels);
  }
  expect(
    cv::norm(expected_pixels, fitted_pixels, cv::NORM_INF) < 0.01,
    "Unseen-pose projection inaccurate");
  cv::Mat map_x, map_y;
  calibration::make_undistortion_maps(result, size, map_x, map_y);
  expect(
    map_x.size() == size && map_y.size() == size && cv::checkRange(map_x) && cv::checkRange(map_y),
    "Invalid undistortion maps");
  fmt::print(
    "PASS {}: known intrinsics/distortion, unseen pose, maps, RMS {:.6f} px\n", model,
    result.rms_px);

  observations.resize(3);
  bool rejected = false;
  try {
    calibration::calibrate_uvc(observations, size, board, model);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  expect(rejected, "Insufficient views must be rejected");
}

void test_chessboard()
{
  // Deliberately differs from the industrial program's hard-coded 11x8 pattern.
  calibration::Chessboard board{{7, 5}, 25, 12};
  constexpr int square = 50;
  cv::Mat image(400, 500, CV_8UC1, cv::Scalar(255));
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 8; ++col) {
      if ((row + col) % 2 == 0)
        cv::rectangle(
          image, {50 + col * square, 50 + row * square, square, square}, cv::Scalar(0), cv::FILLED);
    }
  }
  std::vector<cv::Point2f> corners;
  expect(
    calibration::find_corners(image, board, corners), "Configured 7x5 chessboard was not detected");
  expect(corners.size() == 35, "Wrong corner count");
  auto points = calibration::board_points(board);
  expect(
    points.front() == cv::Point3f(0, 0, 0) && points.back() == cv::Point3f(150, 100, 0),
    "Board geometry must use configured spacing in mm");
  cv::Mat blank(image.size(), CV_8UC1, cv::Scalar(127));
  expect(!calibration::find_corners(blank, board, corners), "Blank image accepted as chessboard");
  fmt::print("PASS configurable chessboard and blank-image rejection\n");
}
}  // namespace

int main()
{
  try {
    test_chessboard();
    test_model("pinhole");
    test_model("fisheye");
    return 0;
  } catch (const std::exception & error) {
    fmt::print(stderr, "uvc_calibration_test: {}\n", error.what());
    return 1;
  }
}
