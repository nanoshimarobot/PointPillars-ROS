#pragma once

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <jsk_recognition_msgs/msg/bounding_box.hpp>
#include <jsk_recognition_msgs/msg/bounding_box_array.hpp>
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

  rclcpp::Publisher<jsk_recognition_msgs::msg::BoundingBoxArray>::SharedPtr bb_pub_;

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
    pp_config_ = static_cast<std::string>(this->declare_parameter("detector.pp_config", ""));

    point_pillars_ptr_ = std::make_unique<PointPillars>(
      score_threshold_, nms_overlap_threshold_, use_onnx_, pfe_onnx_file_, backbone_file_,
      pp_config_);

    bb_pub_ = this->create_publisher<jsk_recognition_msgs::msg::BoundingBoxArray>(
      "dbg/bounding_box", rclcpp::QoS(1));
    input_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", rclcpp::QoS(1).best_effort(),
      std::bind(&PointPillarsROS::input_cloud_cb, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(300ms, std::bind(&PointPillarsROS::main_process, this));
  }

  void main_process()
  {
    // jsk_recognition_msgs::msg::BoundingBoxArray bb_array;
    // jsk_recognition_msgs::msg::BoundingBox bb_msg;
    // bb_msg.header.frame_id = "livox_frame";
    // bb_msg.header.stamp = this->get_clock()->now();
    // bb_msg.pose.orientation.w = 1.0;

    // for (size_t i = 0; i < 3; ++i) {
    //   bb_msg.pose.position.x = i;
    //   bb_msg.pose.position.y = i;
    //   bb_msg.pose.position.z = i;
    //   bb_msg.dimensions.x = 1.0 * i;
    //   bb_msg.dimensions.y = 1.0 * i;
    //   bb_msg.dimensions.z = 1.0 * i;
    //   bb_array.boxes.push_back(bb_msg);
    // }

    // bb_pub_->publish(bb_array);
    return;
    if (cloud_buffer_.size() <= 10) {
      return;
    }

    float * points_array = new float[cloud_buffer_.size() * NUM_POINT_FEATURE_];
    cloud_to_array(cloud_buffer_, points_array, 0.4);  // tekitou offset variable

    std::vector<float> detection_result;
    std::vector<int> label_result;
    std::vector<float> scores_result;
    point_pillars_ptr_->doInference(
      points_array, cloud_buffer_.size(), &detection_result, &label_result, &scores_result);

    RCLCPP_INFO(this->get_logger(), "======================");
    for (auto & label : label_result) {
      RCLCPP_INFO(this->get_logger(), "label : %d", label);
    }
    RCLCPP_INFO(this->get_logger(), "======================");

    size_t result_size = detection_result.size() / OUTPUT_NUM_BOX_FEATURE_;
    jsk_recognition_msgs::msg::BoundingBoxArray bb_array;
    jsk_recognition_msgs::msg::BoundingBox result_bb;
    result_bb.header.frame_id = "livox_frame";
    result_bb.header.stamp = this->get_clock()->now();
    for (size_t i = 0; i < result_size; ++i) {
      result_bb.pose.position.x = detection_result[i * OUTPUT_NUM_BOX_FEATURE_ + 0];
      result_bb.pose.position.y = detection_result[i * OUTPUT_NUM_BOX_FEATURE_ + 1];
      result_bb.pose.position.z = detection_result[i * OUTPUT_NUM_BOX_FEATURE_ + 2];
      // float yaw = detection_result[i * OUTPUT_NUM_BOX_FEATURE_ + 6];
      // yaw += M_PI / 2;
      // yaw = std::atan2(std::sin(yaw), std::cos(yaw));
      // geometry_msgs::msg::Quaternion q = tf::createQuaternionMsgFromYaw(-yaw);
      // result_bb.pose.orientation = q;

      result_bb.dimensions.x = detection_result[i * OUTPUT_NUM_BOX_FEATURE_ + 4];
      result_bb.dimensions.y = detection_result[i * OUTPUT_NUM_BOX_FEATURE_ + 3];
      result_bb.dimensions.z = detection_result[i * OUTPUT_NUM_BOX_FEATURE_ + 5];

      bb_array.boxes.push_back(result_bb);
    }

    bb_pub_->publish(bb_array);

    delete[] points_array;

    cloud_buffer_.clear();
  }

  void input_cloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_pcl_cloud =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromROSMsg(*msg, *input_pcl_cloud);

    if (input_pcl_cloud->size() < 10) {
      return;
    }

    // for (size_t i = 0; i < input_pcl_cloud->size(); ++i) {
    //   // input_pcl_cloud->at(i).z += 1.0;
    // }

    float * points_array = new float[input_pcl_cloud->size() * NUM_POINT_FEATURE_];
    cloud_to_array(*input_pcl_cloud, points_array, 0.0);  // tekitou offset variable

    std::vector<float> detection_result;
    std::vector<int> label_result;
    std::vector<float> scores_result;
    point_pillars_ptr_->doInference(
      points_array, cloud_buffer_.size(), &detection_result, &label_result, &scores_result);

    RCLCPP_INFO(this->get_logger(), "======================");
    for (auto & label : label_result) {
      RCLCPP_INFO(this->get_logger(), "label : %d", label);
    }
    RCLCPP_INFO(this->get_logger(), "======================");

    size_t result_size = detection_result.size() / OUTPUT_NUM_BOX_FEATURE_;
    jsk_recognition_msgs::msg::BoundingBoxArray bb_array;
    jsk_recognition_msgs::msg::BoundingBox result_bb;
    result_bb.header.frame_id = "base_link";
    result_bb.header.stamp = this->get_clock()->now();
    for (size_t i = 0; i < result_size; ++i) {
      result_bb.pose.position.x = detection_result[i * OUTPUT_NUM_BOX_FEATURE_ + 0];
      result_bb.pose.position.y = detection_result[i * OUTPUT_NUM_BOX_FEATURE_ + 1];
      result_bb.pose.position.z = detection_result[i * OUTPUT_NUM_BOX_FEATURE_ + 2];
      // float yaw = detection_result[i * OUTPUT_NUM_BOX_FEATURE_ + 6];
      // yaw += M_PI / 2;
      // yaw = std::atan2(std::sin(yaw), std::cos(yaw));
      // geometry_msgs::msg::Quaternion q = tf::createQuaternionMsgFromYaw(-yaw);
      // result_bb.pose.orientation = q;

      result_bb.dimensions.x = detection_result[i * OUTPUT_NUM_BOX_FEATURE_ + 4];
      result_bb.dimensions.y = detection_result[i * OUTPUT_NUM_BOX_FEATURE_ + 3];
      result_bb.dimensions.z = detection_result[i * OUTPUT_NUM_BOX_FEATURE_ + 5];

      bb_array.boxes.push_back(result_bb);
    }

    bb_pub_->publish(bb_array);

    delete[] points_array;
    // cloud_buffer_ += *input_pcl_cloud;
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