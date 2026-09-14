#pragma once

// C++ port of realsense_ddgng/dd_gng_yolo.py's AsyncYolo: runs YOLOX off the
// depth/GNG loop so a ~100-300 ms OpenCV DNN forward pass never stalls the
// ~30 Hz capture thread. Same non-blocking submit()/snapshot() shape as the
// Python class: submit() starts a detached worker at most once per period_s
// and is a no-op otherwise; snapshot() returns whatever the last worker
// produced (stale between runs, which is expected and handled by the caller
// re-checking depth every frame against the last known box).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include "om6dof_dd_gng/yolox_detector.hpp"

namespace om6dof_dd_gng
{

class AsyncYolo
{
public:
  enum class SubmitOutcome
  {
    Started,
    Busy,
    RateLimited,
  };

  struct Snapshot
  {
    std::vector<YoloDetection> detections;
    uint64_t input_frame_sequence = 0;
    uint64_t result_sequence = 0;
    uint64_t started_count = 0;
    uint64_t busy_skip_count = 0;
    uint64_t rate_limited_count = 0;
    uint64_t completed_count = 0;
    uint64_t failed_count = 0;
    double inference_ms = -1.0;
    double result_age_ms = -1.0;
    std::string last_error;
    bool busy = false;
  };

  AsyncYolo(
    std::shared_ptr<YoloDetector> detector, double period_s,
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

  SubmitOutcome submit(const cv::Mat & bgr, uint64_t input_frame_sequence)
  {
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (busy_) {
        ++busy_skip_count_;
        return SubmitOutcome::Busy;
      }
      const double elapsed = std::chrono::duration<double>(now - last_start_).count();
      if (elapsed < period_s_) {
        ++rate_limited_count_;
        return SubmitOutcome::RateLimited;
      }
      busy_ = true;
      last_start_ = now;
      ++started_count_;
    }
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread(&AsyncYolo::runWorker, this, bgr.clone(), input_frame_sequence);
    return SubmitOutcome::Started;
  }

  Snapshot snapshot() const
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    Snapshot output;
    output.detections = detections_;
    output.input_frame_sequence = result_input_frame_sequence_;
    output.result_sequence = result_sequence_;
    output.started_count = started_count_;
    output.busy_skip_count = busy_skip_count_;
    output.rate_limited_count = rate_limited_count_;
    output.completed_count = completed_count_;
    output.failed_count = failed_count_;
    output.inference_ms = inference_ms_;
    output.last_error = last_error_;
    output.busy = busy_;
    if (result_sequence_ > 0) {
      output.result_age_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - result_completed_).count();
    }
    return output;
  }

  // A selection is configuration for future submissions only. A worker that
  // has already started keeps its own snapshot, so changing the GUI target
  // never races the detector mid-inference or changes a completed result.
  void setAllowedClasses(std::unordered_set<std::string> allowed_classes)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    allowed_classes_ = std::move(allowed_classes);
  }

private:
  void runWorker(cv::Mat bgr, uint64_t input_frame_sequence)
  {
    const auto started = std::chrono::steady_clock::now();
    std::vector<YoloDetection> detections;
    std::string error;
    try {
      detections = detector_->detect(bgr);
    } catch (const std::exception & ex) {
      error = ex.what();
    } catch (...) {
      error = "unknown YOLO worker failure";
    }
    std::unordered_set<std::string> allowed_classes;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      allowed_classes = allowed_classes_;
    }
    if (!allowed_classes.empty()) {
      std::vector<YoloDetection> filtered;
      filtered.reserve(detections.size());
      for (const auto & detection : detections) {
        if (allowed_classes.count(detection.className()) > 0) {
          filtered.push_back(detection);
        }
      }
      detections = std::move(filtered);
    }
    const auto completed = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!error.empty()) {
      ++failed_count_;
      last_error_ = std::move(error);
      inference_ms_ = std::chrono::duration<double, std::milli>(completed - started).count();
      busy_ = false;
      return;
    }
    detections_ = std::move(detections);
    result_input_frame_sequence_ = input_frame_sequence;
    ++result_sequence_;
    ++completed_count_;
    last_error_.clear();
    inference_ms_ = std::chrono::duration<double, std::milli>(completed - started).count();
    result_completed_ = completed;
    busy_ = false;
  }

  std::shared_ptr<YoloDetector> detector_;
  double period_s_;
  std::unordered_set<std::string> allowed_classes_;

  mutable std::mutex state_mutex_;
  bool busy_ = false;
  std::chrono::steady_clock::time_point last_start_{};
  std::chrono::steady_clock::time_point result_completed_{};
  uint64_t result_input_frame_sequence_ = 0;
  uint64_t result_sequence_ = 0;
  uint64_t started_count_ = 0;
  uint64_t busy_skip_count_ = 0;
  uint64_t rate_limited_count_ = 0;
  uint64_t completed_count_ = 0;
  uint64_t failed_count_ = 0;
  double inference_ms_ = -1.0;
  std::string last_error_;
  std::vector<YoloDetection> detections_;
  std::thread worker_;
};

}  // namespace om6dof_dd_gng
