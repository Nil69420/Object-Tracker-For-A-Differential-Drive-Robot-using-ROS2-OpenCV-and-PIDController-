#include <memory>

#include "src/tracking/include/tracking/ObjectDetector.hpp"
#include "src/tracking/include/tracking/HeadController.hpp"

#include "/home/nil/Desktop/bookros2_ws/src/tracking_msgs/msg/PanTiltCommand.msg"

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node_detector = std::make_shared<tracking::ObjectDetector>();
  auto node_head_controller = std::make_shared<tracking::HeadController>();
  auto node_tracker = rclcpp::Node::make_shared("tracker");

  const int IMG_WIDTH = 640;
  const int IMG_HEIGHT = 480;

  auto command_pub = node_tracker->create_publisher<tracking_msgs::msg::PanTiltCommand>(
    "/command", 100);
  auto detection_sub = node_tracker->create_subscription<vision_msgs::msg::Detection2D>(
    "/detection", rclcpp::SensorDataQoS(),
    [command_pub](vision_msgs::msg::Detection2D::SharedPtr msg) {
      tracking_msgs::msg::PanTiltCommand command;
      command.pan = (msg->bbox.center.position.x / IMG_WIDTH) * 2.0 - 1.0;
      command.tilt = (msg->bbox.center.position.y / IMG_HEIGHT) * 2.0 - 1.0;
      command_pub->publish(command);
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_detector);
  executor.add_node(node_head_controller->get_node_base_interface());
  executor.add_node(node_tracker);

  node_head_controller->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}