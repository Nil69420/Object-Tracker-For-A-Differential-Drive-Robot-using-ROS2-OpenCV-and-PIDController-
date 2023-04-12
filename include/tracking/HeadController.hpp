#ifndef TRACKING__HEADCONTROLLER_HPP_
#define TRACKING__HEADCONTROLLER_HPP_

#include <memory>

#include "src/tracking_msgs/msg/pan_tilt_command.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "src/tracking/include/tracking/PIDController.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tracking
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HeadController : public rclcpp_lifecycle::LifecycleNode
{
public:
  HeadController();

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);

  void control_sycle();

  void joint_state_callback(control_msgs::msg::JointTrajectoryControllerState::UniquePtr msg);
  void command_callback(tracking_msgs::msg::PanTiltCommand::UniquePtr msg);

private:
  rclcpp::Subscription<tracking_msgs::msg::PanTiltCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_sub_;
  rclcpp_lifecycle::LifecyclePublisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  control_msgs::msg::JointTrajectoryControllerState::UniquePtr last_state_;
  tracking_msgs::msg::PanTiltCommand::UniquePtr last_command_;
  rclcpp::Time last_command_ts_;

  PIDController pan_pid_, tilt_pid_;
};

}  

#endif
