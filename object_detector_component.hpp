#pragma once

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace aist_intern2023
{
class ObjectDetector : public rclcpp::Node
{
private:
  double z_range_min_;
  double z_range_max_;

  pcl::PointCloud<pcl::PointXYZI> cloud_buffer_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_cloud_sub_;

public:
  ObjectDetector(const rclcpp::NodeOptions & options) : ObjectDetector("", options) {}
  ObjectDetector(
    const std::string & name_space = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("object_detector_node", name_space, options)
  {
    using namespace std::chrono_literals;
    z_range_min_ = static_cast<double>(this->declare_parameter("detector.z_range_min", -0.2));
    z_range_max_ = static_cast<double>(this->declare_parameter("detector.z_range_max", 3.0));

    input_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", rclcpp::QoS(1).best_effort(),
      std::bind(&ObjectDetector::input_cloud_cb, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(100ms, std::bind(&ObjectDetector::main_process, this));
  }

  void main_process() {
    
  }

  void input_cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_pcl_cloud =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromROSMsg(*msg, *input_pcl_cloud);
    cloud_buffer_ += *input_pcl_cloud;
  }
};
}  // namespace aist_intern2023
