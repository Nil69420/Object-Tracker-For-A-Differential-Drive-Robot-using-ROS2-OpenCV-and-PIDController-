#include <memory>

#include "/src/tracking/include/tracking/ObjectDetector.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_detector = std::make_shared<tracking::ObjectDetector>();

  rclcpp::spin(node_detector);

  rclcpp::shutdown();
  return 0;
}