#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>

#include "om6dof_dd_gng/tensorrt_yolox_detector.hpp"

namespace
{
std::string expandHome(const std::string & path)
{
  if (path.rfind("~/", 0) != 0) {
    return path;
  }
  const char * home = std::getenv("HOME");
  return home ? std::string(home) + path.substr(1) : path;
}

cv::Scalar classColor(const int class_id)
{
  const unsigned int seed = static_cast<unsigned int>(class_id * 2654435761U);
  return cv::Scalar(64 + (seed & 0x7fU), 64 + ((seed >> 8) & 0x7fU), 64 + ((seed >> 16) & 0x7fU));
}
}  // namespace

class YoloXViewerNode final : public rclcpp::Node
{
public:
  YoloXViewerNode()
  : Node("yolox_viewer")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/om6dof_topo_gng_v2/rgb/image/compressed");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/om6dof_yolox/debug_image/compressed");
    detections_topic_ = declare_parameter<std::string>(
      "detections_topic", "/om6dof_yolox/detections");
    engine_path_ = expandHome(declare_parameter<std::string>(
      "yolo_engine_path", "~/.cache/om6dof_perception/yolox_s_fp16.engine"));
    confidence_ = static_cast<float>(declare_parameter<double>("confidence", 0.25));
    nms_threshold_ = static_cast<float>(declare_parameter<double>("nms_threshold", 0.45));
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 90);
    display_window_ = declare_parameter<bool>("display_window", false);

    overlay_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(output_topic_, 2);
    detections_publisher_ = create_publisher<std_msgs::msg::String>(detections_topic_, 2);
    input_subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&YoloXViewerNode::inputCallback, this, std::placeholders::_1));

    worker_ = std::thread(&YoloXViewerNode::workerLoop, this);
    RCLCPP_INFO(
      get_logger(), "YOLOX C++ viewer: %s -> %s (all post-NMS boxes)",
      input_topic_.c_str(), output_topic_.c_str());
  }

  ~YoloXViewerNode() override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_one();
    if (worker_.joinable()) {
      worker_.join();
    }
    if (display_window_ready_) {
      cv::destroyWindow(window_name_);
    }
  }

private:
  struct Frame
  {
    std_msgs::msg::Header header;
    cv::Mat bgr;
  };

  void inputCallback(const sensor_msgs::msg::CompressedImage::SharedPtr message)
  {
    const cv::Mat encoded(1, static_cast<int>(message->data.size()), CV_8UC1,
      const_cast<uint8_t *>(message->data.data()));
    cv::Mat bgr = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (bgr.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Cannot decode input RGB JPEG");
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_frame_ = Frame{message->header, std::move(bgr)};
      frame_ready_ = true;
    }
    condition_.notify_one();
  }

  void workerLoop()
  {
    try {
      detector_ = std::make_unique<om6dof_dd_gng::TensorRtYoloXDetector>(
        engine_path_, confidence_, nms_threshold_);
      RCLCPP_INFO(get_logger(), "TensorRT engine loaded: %s", engine_path_.c_str());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "YOLOX C++ viewer could not load engine: %s", error.what());
      return;
    }

    while (rclcpp::ok()) {
      Frame frame;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this] {return stopping_ || frame_ready_;});
        if (stopping_) {
          return;
        }
        frame = std::move(latest_frame_);
        frame_ready_ = false;
      }

      const auto started = std::chrono::steady_clock::now();
      std::vector<om6dof_dd_gng::YoloDetection> detections;
      try {
        detections = detector_->detect(frame.bgr);
      } catch (const std::exception & error) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "YOLOX inference failed: %s", error.what());
        continue;
      }
      const double inference_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
      const double output_fps = updateOutputFps();
      publishOverlay(frame, detections, inference_ms, output_fps);
      publishDetections(frame.header, detections, inference_ms);
    }
  }

  void publishOverlay(
    const Frame & frame, const std::vector<om6dof_dd_gng::YoloDetection> & detections,
    const double inference_ms, const double output_fps)
  {
    cv::Mat overlay = frame.bgr.clone();
    for (const auto & detection : detections) {
      const cv::Scalar color = classColor(detection.class_id);
      const cv::Rect box(
        static_cast<int>(detection.x), static_cast<int>(detection.y),
        static_cast<int>(detection.w), static_cast<int>(detection.h));
      cv::rectangle(overlay, box, color, 2, cv::LINE_AA);
      char text[160];
      std::snprintf(text, sizeof(text), "%s %.0f%%", detection.className(), detection.score * 100.0F);
      int baseline = 0;
      const cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.55, 1, &baseline);
      const int text_y = std::max(text_size.height + 4, box.y);
      cv::rectangle(overlay, cv::Rect(box.x, text_y - text_size.height - 5,
        text_size.width + 6, text_size.height + 6), color, cv::FILLED);
      cv::putText(overlay, text, cv::Point(box.x + 3, text_y - 3),
        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }
    char hud[128];
    std::snprintf(hud, sizeof(hud), "YOLOX C++ | %.1f FPS | %zu boxes | %.1f ms",
      output_fps, detections.size(), inference_ms);
    cv::putText(overlay, hud, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX,
      0.7, cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(overlay, hud, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX,
      0.7, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    std::vector<uint8_t> encoded;
    if (!cv::imencode(".jpg", overlay, encoded, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_})) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Cannot encode YOLOX overlay JPEG");
      return;
    }
    sensor_msgs::msg::CompressedImage output;
    output.header = frame.header;
    output.format = "jpeg";
    output.data = std::move(encoded);
    overlay_publisher_->publish(std::move(output));
    showWindow(overlay);
  }

  void showWindow(const cv::Mat & overlay)
  {
    if (!display_window_) {
      return;
    }
    if (!display_window_ready_) {
      if (std::getenv("DISPLAY") == nullptr) {
        RCLCPP_ERROR(get_logger(), "display_window:=true requires a local AGX desktop DISPLAY");
        display_window_ = false;
        return;
      }
      try {
        cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_name_, 960, 720);
        display_window_ready_ = true;
      } catch (const cv::Exception & error) {
        RCLCPP_ERROR(get_logger(), "Cannot create YOLOX display window: %s", error.what());
        display_window_ = false;
        return;
      }
    }
    try {
      cv::imshow(window_name_, overlay);
      cv::waitKey(1);
    } catch (const cv::Exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
        "Cannot update YOLOX display window: %s", error.what());
    }
  }

  void publishDetections(
    const std_msgs::msg::Header & header,
    const std::vector<om6dof_dd_gng::YoloDetection> & detections, const double inference_ms)
  {
    std::ostringstream json;
    json << "{\"stamp_sec\":" << header.stamp.sec << ",\"stamp_nanosec\":" << header.stamp.nanosec
         << ",\"inference_ms\":" << inference_ms << ",\"detections\":[";
    for (size_t i = 0; i < detections.size(); ++i) {
      const auto & d = detections[i];
      if (i) {json << ',';}
      json << "{\"class_id\":" << d.class_id << ",\"class\":\"" << d.className()
           << "\",\"score\":" << d.score << ",\"x\":" << d.x << ",\"y\":" << d.y
           << ",\"w\":" << d.w << ",\"h\":" << d.h << '}';
    }
    json << "]}";
    std_msgs::msg::String output;
    output.data = json.str();
    detections_publisher_->publish(std::move(output));
  }

  double updateOutputFps()
  {
    const auto now = std::chrono::steady_clock::now();
    ++frames_in_fps_window_;
    const double elapsed = std::chrono::duration<double>(now - fps_window_started_).count();
    if (elapsed >= 1.0) {
      output_fps_ = static_cast<double>(frames_in_fps_window_) / elapsed;
      fps_window_started_ = now;
      frames_in_fps_window_ = 0;
    }
    return output_fps_;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string detections_topic_;
  std::string engine_path_;
  float confidence_ = 0.25F;
  float nms_threshold_ = 0.45F;
  int jpeg_quality_ = 90;
  bool display_window_ = false;
  bool display_window_ready_ = false;
  const std::string window_name_ = "YOLOX C++ — RealSense RGB";
  std::unique_ptr<om6dof_dd_gng::TensorRtYoloXDetector> detector_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr input_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr overlay_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detections_publisher_;
  std::mutex mutex_;
  std::condition_variable condition_;
  Frame latest_frame_;
  bool frame_ready_ = false;
  bool stopping_ = false;
  std::thread worker_;
  std::chrono::steady_clock::time_point fps_window_started_ = std::chrono::steady_clock::now();
  size_t frames_in_fps_window_ = 0;
  double output_fps_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YoloXViewerNode>());
  rclcpp::shutdown();
  return 0;
}
