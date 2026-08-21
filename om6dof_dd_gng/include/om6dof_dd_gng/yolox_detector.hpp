#pragma once

// C++ port of om6dof_perception/om6dof_perception/yolox_detector.py's
// OpenCV-DNN YOLOX-S wrapper. Pre/post-processing mirrors that file exactly
// (letterbox padding, raw 0-255 RGB input with no mean/std normalisation,
// grid+stride decode, cv::dnn::NMSBoxes) because it is the already-verified
// pairing for models/yolox_s.onnx on this hardware; ONNX Runtime (what
// TopoVLA's own main.cpp uses) is not available on this Jetson.

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

namespace om6dof_dd_gng
{

constexpr std::array<const char *, 80> kCocoClasses = {
  "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
  "truck", "boat", "traffic light", "fire hydrant", "stop sign",
  "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
  "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
  "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
  "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
  "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
  "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
  "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
  "couch", "potted plant", "bed", "dining table", "toilet", "tv",
  "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
  "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
  "scissors", "teddy bear", "hair drier", "toothbrush",
};

struct YoloDetection
{
  // x, y, width, height in the *source* image (post letterbox-descale), pixels.
  float x = 0.0F;
  float y = 0.0F;
  float w = 0.0F;
  float h = 0.0F;
  float score = 0.0F;
  int class_id = -1;

  const char * className() const
  {
    return (class_id >= 0 && class_id < static_cast<int>(kCocoClasses.size()))
      ? kCocoClasses[static_cast<size_t>(class_id)] : "unknown";
  }
};

class YoloXDetector
{
public:
  static constexpr int kInputSize = 640;
  static constexpr std::array<int, 3> kStrides = {8, 16, 32};

  YoloXDetector(const std::string & model_path, float confidence, float nms_threshold)
  : confidence_(confidence), nms_threshold_(nms_threshold)
  {
    net_ = cv::dnn::readNet(model_path);
    buildAnchors();
  }

  std::vector<YoloDetection> detect(const cv::Mat & bgr)
  {
    float scale = 1.0F;
    cv::Mat blob = letterboxToBlob(bgr, scale);

    net_.setInput(blob);
    std::vector<cv::Mat> outs;
    net_.forward(outs, net_.getUnconnectedOutLayersNames());
    if (outs.empty()) {
      return {};
    }
    // Expected shape (1, 8400, 85): 4 box + 1 objectness + 80 class scores,
    // 8400 = 80*80 + 40*40 + 20*20 anchors for strides 8/16/32 at 640 input.
    const cv::Mat & out = outs[0];
    const int num_boxes = out.size[1];
    const int num_attrs = out.size[2];
    const int num_classes = num_attrs - 5;
    const float * data = reinterpret_cast<const float *>(out.data);

    std::vector<cv::Rect2d> boxes;
    std::vector<float> scores;
    std::vector<int> class_ids;
    boxes.reserve(64);
    scores.reserve(64);
    class_ids.reserve(64);

    for (int i = 0; i < num_boxes; ++i) {
      const float * row = data + static_cast<size_t>(i) * num_attrs;
      const float cx = (row[0] + anchor_grid_[static_cast<size_t>(i)].x) *
        anchor_stride_[static_cast<size_t>(i)];
      const float cy = (row[1] + anchor_grid_[static_cast<size_t>(i)].y) *
        anchor_stride_[static_cast<size_t>(i)];
      const float w = std::exp(row[2]) * anchor_stride_[static_cast<size_t>(i)];
      const float h = std::exp(row[3]) * anchor_stride_[static_cast<size_t>(i)];
      const float objectness = row[4];

      int best_class = -1;
      float best_score = 0.0F;
      for (int c = 0; c < num_classes; ++c) {
        const float score = objectness * row[5 + c];
        if (score > best_score) {
          best_score = score;
          best_class = c;
        }
      }
      if (best_score < confidence_) {
        continue;
      }
      const double x = cx - w / 2.0F;
      const double y = cy - h / 2.0F;
      boxes.emplace_back(x / scale, y / scale, w / scale, h / scale);
      scores.push_back(best_score);
      class_ids.push_back(best_class);
    }

    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, scores, confidence_, nms_threshold_, keep);

    const int width = bgr.cols;
    const int height = bgr.rows;
    std::vector<YoloDetection> results;
    results.reserve(keep.size());
    for (const int index : keep) {
      double x = boxes[static_cast<size_t>(index)].x;
      double y = boxes[static_cast<size_t>(index)].y;
      double w = boxes[static_cast<size_t>(index)].width;
      double h = boxes[static_cast<size_t>(index)].height;
      x = std::clamp(std::round(x), 0.0, static_cast<double>(width - 1));
      y = std::clamp(std::round(y), 0.0, static_cast<double>(height - 1));
      w = std::max(4.0, std::min(static_cast<double>(width) - x, std::round(w)));
      h = std::max(4.0, std::min(static_cast<double>(height) - y, std::round(h)));

      YoloDetection detection;
      detection.x = static_cast<float>(x);
      detection.y = static_cast<float>(y);
      detection.w = static_cast<float>(w);
      detection.h = static_cast<float>(h);
      detection.score = scores[static_cast<size_t>(index)];
      detection.class_id = class_ids[static_cast<size_t>(index)];
      results.push_back(detection);
    }
    return results;
  }

private:
  void buildAnchors()
  {
    anchor_grid_.clear();
    anchor_stride_.clear();
    for (const int stride : kStrides) {
      const int size = kInputSize / stride;
      for (int gy = 0; gy < size; ++gy) {
        for (int gx = 0; gx < size; ++gx) {
          anchor_grid_.push_back(cv::Point2f(static_cast<float>(gx), static_cast<float>(gy)));
          anchor_stride_.push_back(static_cast<float>(stride));
        }
      }
    }
  }

  // Matches yolox_detector.py's _letterbox exactly: resize preserving aspect
  // ratio to fit inside 640x640, pad with 114, BGR->RGB, NO /255 scaling
  // (this model was exported to expect raw 0-255 input).
  cv::Mat letterboxToBlob(const cv::Mat & bgr, float & scale_out)
  {
    const int height = bgr.rows;
    const int width = bgr.cols;
    const float scale = std::min(
      static_cast<float>(kInputSize) / height, static_cast<float>(kInputSize) / width);
    scale_out = scale;
    const int new_w = static_cast<int>(width * scale);
    const int new_h = static_cast<int>(height * scale);

    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);
    cv::Mat resized_rgb;
    cv::cvtColor(resized, resized_rgb, cv::COLOR_BGR2RGB);

    cv::Mat padded(kInputSize, kInputSize, CV_8UC3, cv::Scalar(114, 114, 114));
    resized_rgb.copyTo(padded(cv::Rect(0, 0, new_w, new_h)));

    cv::Mat padded_f;
    padded.convertTo(padded_f, CV_32F);
    // scalefactor=1.0, no mean subtraction, swapRB=false (already RGB above).
    return cv::dnn::blobFromImage(padded_f, 1.0, cv::Size(kInputSize, kInputSize), cv::Scalar(), false, false);
  }

  cv::dnn::Net net_;
  float confidence_;
  float nms_threshold_;
  std::vector<cv::Point2f> anchor_grid_;
  std::vector<float> anchor_stride_;
};

}  // namespace om6dof_dd_gng
