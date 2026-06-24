#include <chrono>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class CaptureCalibSample : public rclcpp::Node {
public:
  CaptureCalibSample() : Node("capture_calib_sample") {
    lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/livox/lidar");
    image_topic_ = declare_parameter<std::string>(
        "image_topic", "/camera/color/image_raw");
    pcd_file_ = declare_parameter<std::string>("pcd_file", "/tmp/livox_calib_sample.pcd");
    image_file_ = declare_parameter<std::string>("image_file", "/tmp/livox_calib_sample.png");
    duration_ = declare_parameter<double>("duration", 1.0);
    image_timeout_ = declare_parameter<double>("image_timeout", 2.0);

    if (duration_ <= 0.0) {
      throw std::invalid_argument("duration must be greater than zero");
    }

    auto qos = rclcpp::SensorDataQoS();
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, qos,
        std::bind(&CaptureCalibSample::lidarCallback, this, std::placeholders::_1));
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, qos,
        std::bind(&CaptureCalibSample::imageCallback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&CaptureCalibSample::checkCompletion, this));

    RCLCPP_INFO(get_logger(),
                "Waiting for lidar %s and camera %s; output: %s, %s",
                lidar_topic_.c_str(), image_topic_.c_str(), pcd_file_.c_str(),
                image_file_.c_str());
  }

private:
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message) {
    const auto now = std::chrono::steady_clock::now();
    if (!started_) {
      started_ = true;
      start_time_ = now;
      RCLCPP_INFO(get_logger(), "First lidar frame received; collecting for %.3f s", duration_);
    }
    if (collection_finished_) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> frame;
    pcl::fromROSMsg(*message, frame);
    cloud_ += frame;
    ++lidar_frames_;

    if (std::chrono::duration<double>(now - start_time_).count() >= duration_) {
      collection_finished_ = true;
      finish_time_ = now;
      RCLCPP_INFO(get_logger(), "Point-cloud window complete: %zu frames, %zu points",
                  lidar_frames_, cloud_.size());
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr message) {
    if (!started_ || collection_finished_) {
      return;
    }
    try {
      image_ = cv_bridge::toCvCopy(message, "bgr8")->image;
      image_received_ = true;
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_WARN(get_logger(), "Unable to convert image: %s", error.what());
    }
  }

  void checkCompletion() {
    if (!started_ || saved_) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (!collection_finished_ &&
        std::chrono::duration<double>(now - start_time_).count() >= duration_) {
      collection_finished_ = true;
      finish_time_ = now;
    }
    if (!collection_finished_) {
      return;
    }
    const bool image_timed_out =
        std::chrono::duration<double>(now - finish_time_).count() >= image_timeout_;
    if (!image_received_ && !image_timed_out) {
      return;
    }
    saveAndExit();
  }

  void saveAndExit() {
    saved_ = true;
    cloud_.width = cloud_.size();
    cloud_.height = 1;
    cloud_.is_dense = false;

    bool success = !cloud_.empty() &&
        pcl::io::savePCDFileBinary(pcd_file_, cloud_) == 0;
    if (!image_received_) {
      RCLCPP_ERROR(get_logger(), "No camera image received within the capture window");
      success = false;
    } else if (!cv::imwrite(image_file_, image_)) {
      RCLCPP_ERROR(get_logger(), "Unable to save image to %s", image_file_.c_str());
      success = false;
    }

    if (success) {
      RCLCPP_INFO(get_logger(), "Saved %zu points to %s", cloud_.size(), pcd_file_.c_str());
      RCLCPP_INFO(get_logger(), "Saved image to %s", image_file_.c_str());
    } else if (cloud_.empty()) {
      RCLCPP_ERROR(get_logger(), "No lidar points were received");
    }
    rclcpp::shutdown();
  }

  std::string lidar_topic_;
  std::string image_topic_;
  std::string pcd_file_;
  std::string image_file_;
  double duration_{1.0};
  double image_timeout_{2.0};
  bool started_{false};
  bool collection_finished_{false};
  bool image_received_{false};
  bool saved_{false};
  size_t lidar_frames_{0};
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point finish_time_;
  pcl::PointCloud<pcl::PointXYZI> cloud_;
  cv::Mat image_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CaptureCalibSample>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("capture_calib_sample"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  return 0;
}
