#pragma once

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include "om6dof_dd_gng/yolox_detector.hpp"

namespace om6dof_dd_gng
{

class TensorRtLogger final : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * message) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      last_message_ = message ? message : "TensorRT error";
    }
  }

  const std::string & lastMessage() const {return last_message_;}

private:
  std::string last_message_;
};

class TensorRtYoloXDetector final : public YoloDetector
{
public:
  static constexpr int kInputElements = 1 * 3 * 640 * 640;
  static constexpr int kOutputBoxes = 8400;
  static constexpr int kOutputAttributes = 85;
  static constexpr int kOutputElements = kOutputBoxes * kOutputAttributes;

  TensorRtYoloXDetector(
    const std::string & engine_path, float confidence, float nms_threshold)
  : confidence_(confidence), nms_threshold_(nms_threshold)
  {
    std::ifstream input(engine_path, std::ios::binary | std::ios::ate);
    if (!input) {
      throw std::runtime_error("Cannot open TensorRT engine: " + engine_path);
    }
    const std::streamsize size = input.tellg();
    if (size <= 0) {
      throw std::runtime_error("TensorRT engine is empty: " + engine_path);
    }
    input.seekg(0, std::ios::beg);
    std::vector<char> bytes(static_cast<size_t>(size));
    if (!input.read(bytes.data(), size)) {
      throw std::runtime_error("Cannot read TensorRT engine: " + engine_path);
    }

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_) {
      throw std::runtime_error("TensorRT runtime creation failed: " + logger_.lastMessage());
    }
    engine_.reset(runtime_->deserializeCudaEngine(bytes.data(), bytes.size()));
    if (!engine_) {
      throw std::runtime_error("TensorRT engine deserialization failed: " + logger_.lastMessage());
    }
    context_.reset(engine_->createExecutionContext());
    if (!context_) {
      throw std::runtime_error("TensorRT execution context creation failed");
    }
    discoverBindings();
    checkCuda(cudaStreamCreate(&stream_), "cudaStreamCreate");
    checkCuda(cudaMalloc(&device_input_, sizeof(float) * kInputElements), "cudaMalloc input");
    checkCuda(cudaMalloc(&device_output_, sizeof(float) * kOutputElements), "cudaMalloc output");
    output_.resize(kOutputElements);
    if (!context_->setTensorAddress(input_name_.c_str(), device_input_) ||
      !context_->setTensorAddress(output_name_.c_str(), device_output_))
    {
      throw std::runtime_error("TensorRT tensor address setup failed");
    }
    buildAnchors();
  }

  ~TensorRtYoloXDetector() override
  {
    if (device_output_) {cudaFree(device_output_);}
    if (device_input_) {cudaFree(device_input_);}
    if (stream_) {cudaStreamDestroy(stream_);}
  }

  const char * backendName() const override {return "tensorrt";}

  std::vector<YoloDetection> detect(const cv::Mat & bgr) override
  {
    if (bgr.empty()) {
      return {};
    }
    float scale = 1.0F;
    cv::Mat blob = letterboxToBlob(bgr, scale);
    if (!blob.isContinuous() || blob.total() != kInputElements || blob.type() != CV_32F) {
      throw std::runtime_error("Unexpected YOLOX input blob layout");
    }
    checkCuda(cudaMemcpyAsync(
        device_input_, blob.ptr<float>(), sizeof(float) * kInputElements,
        cudaMemcpyHostToDevice, stream_), "YOLOX host-to-device copy");
    if (!context_->enqueueV3(stream_)) {
      throw std::runtime_error("TensorRT YOLOX enqueue failed");
    }
    checkCuda(cudaMemcpyAsync(
        output_.data(), device_output_, sizeof(float) * kOutputElements,
        cudaMemcpyDeviceToHost, stream_), "YOLOX device-to-host copy");
    checkCuda(cudaStreamSynchronize(stream_), "YOLOX stream synchronize");
    return decode(output_.data(), scale, bgr.cols, bgr.rows);
  }

private:
  template<typename T>
  struct Delete
  {
    void operator()(T * value) const {delete value;}
  };

  static void checkCuda(cudaError_t status, const char * operation)
  {
    if (status != cudaSuccess) {
      throw std::runtime_error(std::string(operation) + ": " + cudaGetErrorString(status));
    }
  }

  static bool dimensionsEqual(const nvinfer1::Dims & dims, const std::vector<int> & expected)
  {
    if (dims.nbDims != static_cast<int>(expected.size())) {return false;}
    for (int i = 0; i < dims.nbDims; ++i) {
      if (dims.d[i] != expected[static_cast<size_t>(i)]) {return false;}
    }
    return true;
  }

  void discoverBindings()
  {
    for (int index = 0; index < engine_->getNbIOTensors(); ++index) {
      const char * name = engine_->getIOTensorName(index);
      if (!name || engine_->getTensorDataType(name) != nvinfer1::DataType::kFLOAT) {
        continue;
      }
      const auto dims = engine_->getTensorShape(name);
      if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT &&
        dimensionsEqual(dims, {1, 3, 640, 640}))
      {
        input_name_ = name;
      } else if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kOUTPUT &&
        dimensionsEqual(dims, {1, 8400, 85}))
      {
        output_name_ = name;
      }
    }
    if (input_name_.empty() || output_name_.empty()) {
      throw std::runtime_error("TensorRT engine must expose float32 [1,3,640,640] -> [1,8400,85]");
    }
  }

  static cv::Mat letterboxToBlob(const cv::Mat & bgr, float & scale_out)
  {
    const float scale = std::min(640.0F / bgr.rows, 640.0F / bgr.cols);
    scale_out = scale;
    const int new_width = static_cast<int>(bgr.cols * scale);
    const int new_height = static_cast<int>(bgr.rows * scale);
    cv::Mat resized;
    cv::resize(bgr, resized, cv::Size(new_width, new_height), 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(resized, resized, cv::COLOR_BGR2RGB);
    cv::Mat padded(640, 640, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(padded(cv::Rect(0, 0, new_width, new_height)));
    cv::Mat padded_float;
    padded.convertTo(padded_float, CV_32F);
    return cv::dnn::blobFromImage(
      padded_float, 1.0, cv::Size(640, 640), cv::Scalar(), false, false);
  }

  void buildAnchors()
  {
    for (const int stride : YoloXDetector::kStrides) {
      const int size = 640 / stride;
      for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
          anchor_grid_.emplace_back(static_cast<float>(x), static_cast<float>(y));
          anchor_stride_.push_back(static_cast<float>(stride));
        }
      }
    }
  }

  std::vector<YoloDetection> decode(
    const float * data, float scale, int image_width, int image_height) const
  {
    std::vector<cv::Rect2d> boxes;
    std::vector<float> scores;
    std::vector<int> class_ids;
    for (int i = 0; i < kOutputBoxes; ++i) {
      const float * row = data + static_cast<size_t>(i) * kOutputAttributes;
      const float cx = (row[0] + anchor_grid_[i].x) * anchor_stride_[i];
      const float cy = (row[1] + anchor_grid_[i].y) * anchor_stride_[i];
      const float width = std::exp(row[2]) * anchor_stride_[i];
      const float height = std::exp(row[3]) * anchor_stride_[i];
      int best_class = -1;
      float best_score = 0.0F;
      for (int class_id = 0; class_id < 80; ++class_id) {
        const float score = row[4] * row[5 + class_id];
        if (score > best_score) {best_score = score; best_class = class_id;}
      }
      if (best_score < confidence_) {continue;}
      boxes.emplace_back(
        (cx - width / 2.0F) / scale, (cy - height / 2.0F) / scale,
        width / scale, height / scale);
      scores.push_back(best_score);
      class_ids.push_back(best_class);
    }
    std::vector<int> keep;
    cv::dnn::NMSBoxes(boxes, scores, confidence_, nms_threshold_, keep);
    std::vector<YoloDetection> detections;
    detections.reserve(keep.size());
    for (const int index : keep) {
      double x = std::clamp(std::round(boxes[index].x), 0.0, static_cast<double>(image_width - 1));
      double y = std::clamp(std::round(boxes[index].y), 0.0, static_cast<double>(image_height - 1));
      double width = std::max(4.0, std::min(static_cast<double>(image_width) - x,
          std::round(boxes[index].width)));
      double height = std::max(4.0, std::min(static_cast<double>(image_height) - y,
          std::round(boxes[index].height)));
      detections.push_back({static_cast<float>(x), static_cast<float>(y),
        static_cast<float>(width), static_cast<float>(height), scores[index], class_ids[index]});
    }
    return detections;
  }

  TensorRtLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime, Delete<nvinfer1::IRuntime>> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine, Delete<nvinfer1::ICudaEngine>> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext, Delete<nvinfer1::IExecutionContext>> context_;
  cudaStream_t stream_ = nullptr;
  void * device_input_ = nullptr;
  void * device_output_ = nullptr;
  std::string input_name_;
  std::string output_name_;
  float confidence_;
  float nms_threshold_;
  std::vector<float> output_;
  std::vector<cv::Point2f> anchor_grid_;
  std::vector<float> anchor_stride_;
};

}  // namespace om6dof_dd_gng
