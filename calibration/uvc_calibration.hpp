#ifndef CALIBRATION__UVC_CALIBRATION_HPP
#define CALIBRATION__UVC_CALIBRATION_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace calibration
{
struct Chessboard
{
  cv::Size pattern_size;
  double square_size_mm;
  int min_views;
};

struct Observation
{
  std::string image_path;
  std::vector<cv::Point2f> corners;
};

struct Intrinsics
{
  std::string model;
  cv::Mat camera_matrix;
  cv::Mat distort_coeffs;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  std::vector<double> per_view_rms_px;
  double rms_px;
  int flags;
};

Chessboard load_chessboard(const std::string & config_path);
std::vector<cv::Point3f> board_points(const Chessboard & board);
bool find_corners(
  const cv::Mat & image, const Chessboard & board, std::vector<cv::Point2f> & corners);
Intrinsics calibrate_uvc(
  const std::vector<Observation> & observations, cv::Size image_size, const Chessboard & board,
  const std::string & model);
cv::Mat make_undistortion_maps(
  const Intrinsics & result, cv::Size image_size, cv::Mat & map_x, cv::Mat & map_y);
}  // namespace calibration

#endif  // CALIBRATION__UVC_CALIBRATION_HPP
