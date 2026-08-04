#ifndef OMNIPERCEPTION__PERCEPTRON_HPP
#define OMNIPERCEPTION__PERCEPTRON_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "decider.hpp"
#include "detection.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tools/thread_safe_queue.hpp"

namespace omniperception
{
struct RawDetectionFrame
{
  cv::Mat image;
  std::list<auto_aim::Armor> armors;
  std::chrono::steady_clock::time_point timestamp;
};

class Perceptron
{
public:
  Perceptron(
    const std::vector<io::USBCamera *> & cameras, const std::string & config_path,
    const std::string & debug_role = "");

  ~Perceptron();

  std::vector<DetectionResult> get_detection_queue();
  bool raw_target_detected(std::chrono::milliseconds hold_time) const;
  std::optional<RawDetectionFrame> raw_detection_frame() const;

  void parallel_infer(io::USBCamera * cam, std::shared_ptr<auto_aim::YOLO> yolo_parallel);

private:
  std::vector<std::thread> threads_;
  tools::ThreadSafeQueue<DetectionResult> detection_queue_;

  std::vector<std::shared_ptr<auto_aim::YOLO>> yolo_detectors_;

  Decider decider_;
  std::atomic<int64_t> last_raw_detection_ns_{0};
  std::string debug_role_;
  std::optional<RawDetectionFrame> raw_detection_frame_;
  bool stop_flag_;
  mutable std::mutex mutex_;
  mutable std::mutex debug_mutex_;
  std::condition_variable condition_;
};

}  // namespace omniperception
#endif
