#pragma once

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "point_pillars.h"

class PointPillarsROS : public rclcpp::Node
{
private:
  const int NUM_POINT_FEATURE_ = 5;
  const int OUTPUT_NUM_BOX_FEATURE_ = 7;
  // const float TRAINED_SENSOR_HEIGHT_ =

  double z_range_min_;
  double z_range_max_;

  bool baselink_support_;
  bool use_tracking_;
  float score_threshold_;
  float nms_overlap_threshold_;
  bool use_onnx_;
  std::string pfe_onnx_file_;
  std::string backbone_file_;
  std::string pp_config_;

  std::unique_ptr<PointPillars> point_pillars_ptr_;
  pcl::PointCloud<pcl::PointXYZI> cloud_buffer_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr input_cloud_sub_;

public:
  PointPillarsROS(const rclcpp::NodeOptions & options) : PointPillarsROS("", options) {}
  PointPillarsROS(
    const std::string & name_space = "",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("object_detector_node", name_space, options)
  {
    using namespace std::chrono_literals;
    z_range_min_ = static_cast<double>(this->declare_parameter("detector.z_range_min", -0.2));
    z_range_max_ = static_cast<double>(this->declare_parameter("detector.z_range_max", 3.0));

    use_tracking_ = static_cast<bool>(this->declare_parameter("detector.use_tracking", false));
    score_threshold_ =
      static_cast<float>(this->declare_parameter("detector.score_threshold", 0.5f));
    nms_overlap_threshold_ =
      static_cast<float>(this->declare_parameter("detector.nms_overlap_threshold", 0.5f));
    use_onnx_ = static_cast<bool>(this->declare_parameter("detector.use_onnx", true));
    pfe_onnx_file_ =
      static_cast<std::string>(this->declare_parameter("detector.pfe_onnx_file", ""));
    backbone_file_ =
      static_cast<std::string>(this->declare_parameter("detector.backbone_file", ""));
    pp_config_ = static_cast<std::string>(this->declare_parameter("pp_config", ""));

    point_pillars_ptr_ = std::make_unique<PointPillars>(
      score_threshold_, nms_overlap_threshold_, use_onnx_, pfe_onnx_file_, backbone_file_,
      pp_config_);

    input_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", rclcpp::QoS(1).best_effort(),
      std::bind(&PointPillarsROS::input_cloud_cb, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(100ms, std::bind(&PointPillarsROS::main_process, this));
  }

  void main_process()
  {
    float * points_array = new float[cloud_buffer_.size() * NUM_POINT_FEATURE_];
    cloud_to_array(cloud_buffer_, points_array, 0.4);  // tekitou offset variable

    std::vector<float> detection_result;
    std::vector<int> label_result;
    std::vector<float> scores_result;
    point_pillars_ptr_->doInference(
      points_array, cloud_buffer_.size(), &detection_result, &label_result, &scores_result);

    delete[] points_array;

    cloud_buffer_.clear();
  }

  void input_cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_pcl_cloud =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromROSMsg(*msg, *input_pcl_cloud);
    cloud_buffer_ += *input_pcl_cloud;
  }

  void cloud_to_array(
    const pcl::PointCloud<pcl::PointXYZI> & input, float * output, const float z_offset)
  {
    for (size_t i = 0; i < input.size(); ++i) {
      output[i * NUM_POINT_FEATURE_ + 0] = input[i].x;
      output[i * NUM_POINT_FEATURE_ + 1] = input[i].y;
      output[i * NUM_POINT_FEATURE_ + 2] = input[i].z + z_offset;
      output[i * NUM_POINT_FEATURE_ + 3] = static_cast<float>(input[i].intensity);

      for (size_t j = 4; j < NUM_POINT_FEATURE_; ++j) {
        output[i * NUM_POINT_FEATURE_ + j] = 0;
      }
    }
  }
};