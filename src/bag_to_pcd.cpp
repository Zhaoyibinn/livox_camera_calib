#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>

template<typename MessageT>
MessageT deserialize(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & bag_message) {
  rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
  rclcpp::Serialization<MessageT> serializer;
  MessageT message;
  serializer.deserialize_message(&serialized, &message);
  return message;
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bag_to_pcd");
  node->declare_parameter("bag_file", "");
  node->declare_parameter("pcd_file", "");
  node->declare_parameter("lidar_topic", "/livox/lidar");
  node->declare_parameter("is_custom_msg", false);

  const auto bag_file = node->get_parameter("bag_file").as_string();
  const auto pcd_file = node->get_parameter("pcd_file").as_string();
  const auto lidar_topic = node->get_parameter("lidar_topic").as_string();
  const auto is_custom_msg = node->get_parameter("is_custom_msg").as_bool();
  if (bag_file.empty() || pcd_file.empty()) {
    RCLCPP_ERROR(node->get_logger(), "bag_file and pcd_file must be set");
    rclcpp::shutdown();
    return 1;
  }

  rosbag2_cpp::Reader reader(
      std::make_unique<rosbag2_cpp::readers::SequentialReader>());
  try {
    reader.open({bag_file, "sqlite3"}, {"cdr", "cdr"});
    rosbag2_storage::StorageFilter filter;
    filter.topics = {lidar_topic};
    reader.set_filter(filter);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Unable to open bag: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  pcl::PointCloud<pcl::PointXYZI> output_cloud;
  try {
    while (reader.has_next()) {
      const auto bag_message = reader.read_next();
      if (is_custom_msg) {
        const auto message = deserialize<livox_ros_driver2::msg::CustomMsg>(bag_message);
        for (const auto & point : message.points) {
          pcl::PointXYZI pcl_point;
          pcl_point.x = point.x;
          pcl_point.y = point.y;
          pcl_point.z = point.z;
          pcl_point.intensity = point.reflectivity;
          output_cloud.push_back(pcl_point);
        }
      } else {
        const auto message = deserialize<sensor_msgs::msg::PointCloud2>(bag_message);
        pcl::PCLPointCloud2 pcl_cloud;
        pcl_conversions::toPCL(message, pcl_cloud);
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromPCLPointCloud2(pcl_cloud, cloud);
        output_cloud += cloud;
      }
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Unable to read bag: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  output_cloud.is_dense = false;
  output_cloud.width = output_cloud.size();
  output_cloud.height = 1;
  if (pcl::io::savePCDFileBinary(pcd_file, output_cloud) != 0) {
    RCLCPP_ERROR(node->get_logger(), "Unable to save PCD: %s", pcd_file.c_str());
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Saved %zu points to %s",
              output_cloud.size(), pcd_file.c_str());
  rclcpp::shutdown();
  return 0;
}
