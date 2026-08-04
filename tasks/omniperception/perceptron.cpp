#include "perceptron.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>
#include <thread>

#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

namespace omniperception
{
Perceptron::Perceptron(
  const std::vector<io::USBCamera *> & cameras, const std::string & config_path,
  const std::string & debug_role)
: detection_queue_(10), decider_(config_path), debug_role_(debug_role), stop_flag_(false)
{
  if (cameras.empty()) {
    throw std::invalid_argument("Perceptron requires at least one camera");
  }

  yolo_detectors_.reserve(cameras.size());
  threads_.reserve(cameras.size());
  for (auto * camera : cameras) {
    if (!camera) {
      throw std::invalid_argument("Perceptron camera pointer is null");
    }
    yolo_detectors_.push_back(std::make_shared<auto_aim::YOLO>(config_path, false));
  }

  for (size_t i = 0; i < cameras.size(); ++i) {
    auto * camera = cameras[i];
    auto detector = yolo_detectors_[i];
    threads_.emplace_back([this, camera, detector] { parallel_infer(camera, detector); });
  }

  tools::logger()->info("Perceptron initialized with {} cameras.", cameras.size());
}

Perceptron::~Perceptron()
{
  {
    std::unique_lock<std::mutex> lock(mutex_);
    stop_flag_ = true;  // 设置退出标志
  }
  condition_.notify_all();  // 唤醒所有等待的线程

  // 等待线程结束
  for (auto & t : threads_) {
    if (t.joinable()) {
      t.join();
    }
  }
  tools::logger()->info("Perceptron destructed.");
}

std::vector<DetectionResult> Perceptron::get_detection_queue()
{
  std::vector<DetectionResult> result;
  DetectionResult temp;

  // 注意：这里的 pop 不阻塞（假设队列为空时会报错或忽略）
  while (!detection_queue_.empty()) {
    detection_queue_.pop(temp);
    result.push_back(std::move(temp));
  }

  return result;
}

bool Perceptron::raw_target_detected(std::chrono::milliseconds hold_time) const
{
  auto last_detection_ns = last_raw_detection_ns_.load(std::memory_order_relaxed);
  if (last_detection_ns == 0) return false;

  auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::steady_clock::now().time_since_epoch())
                  .count();
  auto hold_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(hold_time).count();
  return now_ns - last_detection_ns <= hold_ns;
}

std::optional<RawDetectionFrame> Perceptron::raw_detection_frame() const
{
  std::lock_guard<std::mutex> lock(debug_mutex_);
  return raw_detection_frame_;
}

// 将并行推理逻辑移动到类成员函数
void Perceptron::parallel_infer(
  io::USBCamera * cam, std::shared_ptr<auto_aim::YOLO> yolo_parallel)
{
  if (!cam) {
    tools::logger()->error("Camera pointer is null!");
    return;
  }
  try {
    while (true) {
      cv::Mat usb_img;
      std::chrono::steady_clock::time_point ts;

      {
        std::unique_lock<std::mutex> lock(mutex_);
        if (stop_flag_) break;  // 检查是否需要退出
      }

      cam->read(usb_img, ts);
      if (usb_img.empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        continue;
      }

      auto armors = yolo_parallel->detect(usb_img);
      if (cam->device_name == debug_role_) {
        RawDetectionFrame frame{usb_img.clone(), armors, ts};
        std::lock_guard<std::mutex> lock(debug_mutex_);
        raw_detection_frame_ = std::move(frame);
      }
      if (!armors.empty()) {
        auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count();
        last_raw_detection_ns_.store(now_ns, std::memory_order_relaxed);
      }
      decider_.armor_filter(armors);
      decider_.set_priority(armors);
      armors.sort(
        [](const auto_aim::Armor & a, const auto_aim::Armor & b) {
          return a.priority < b.priority;
        });

      if (!armors.empty()) {
        auto delta_angle = decider_.delta_angle(armors, cam->device_name);

        DetectionResult dr;
        dr.armors = std::move(armors);
        dr.timestamp = ts;
        dr.delta_yaw = delta_angle[0] / 57.3;
        dr.delta_pitch = delta_angle[1] / 57.3;
        detection_queue_.push(dr);  // 推入线程安全队列
      }
    }
  } catch (const std::exception & e) {
    tools::logger()->error("Exception in parallel_infer: {}", e.what());
  }
}

}  // namespace omniperception
