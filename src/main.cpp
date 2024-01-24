#include <memory>
#include <point_pillars_ros/point_pillars_ros_component.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointPillarsROS>());
  rclcpp::shutdown();
  return 0;
}