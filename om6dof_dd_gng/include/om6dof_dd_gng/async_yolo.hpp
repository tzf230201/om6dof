#pragma once

// C++ port of realsense_ddgng/dd_gng_yolo.py's AsyncYolo: runs YOLOX off the
// depth/GNG loop so a ~100-300 ms OpenCV DNN forward pass never stalls the
// ~30 Hz capture thread. Same non-blocking submit()/snapshot() shape as the
// Python class: submit() starts a detached worker at most once per period_s
// and is a no-op otherwise; snapshot() returns whatever the last worker
// produced (stale between runs, which is expected and handled by the caller
// re-checking depth every frame against the last known box).

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <opencv2/opencv.hpp>

#include "om6dof_dd_gng/yolox_detector.hpp"

namespace om6dof_dd_gng
{

class AsyncYolo
{
public:
  AsyncYolo(
    std::shared_ptr<YoloXDetector> detector, double period_s,
    std::unordered_set<std::string> allowed_classes)
  : detector_(std::move(detector)),
    period_s_(std::max(0.05, period_s)),
    allowed_classes_(std::move(allowed_classes))
  {
  }

  ~AsyncYolo()
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void submit(const cv::Mat & bgr)
  {
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (busy_) {
        return;
      }
      const double elapsed = std::chrono::duration<double>(now - last_start_).count();
      if (elapsed < period_s_) {
        return;
      }
      busy_ = true;
      last_start_ = now;
    }
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread(&AsyncYolo::runWorker, this, bgr.clone());
  }

  std::vector<YoloDetection> snapshot() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return detections_;
  }

private:
  void runWorker(cv::Mat bgr)
  {
    std::vector<YoloDetection> detections = detector_->detect(bgr);
    if (!allowed_classes_.empty()) {
      std::vector<YoloDetection> filtered;
      filtered.reserve(detections.size());
      for (const auto & detection : detections) {
        if (allowed_classes_.count(detection.className()) > 0) {
          filtered.push_back(detection);
        }
      }
      detections = std::move(filtered);
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    detections_ = std::move(detections);
    busy_ = false;
  }

  std::shared_ptr<YoloXDetector> detector_;
  double period_s_;
  std::unordered_set<std::string> allowed_classes_;

  mutable std::mutex state_mutex_;
  bool busy_ = false;
  std::chrono::steady_clock::time_point last_start_{};
  std::vector<YoloDetection> detections_;
  std::thread worker_;
};

}  // namespace om6dof_dd_gng
