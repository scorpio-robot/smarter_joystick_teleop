// Copyright 2026 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SMARTER_JOYSTICK_TELEOP__SMARTER_JOYSTICK_TELEOP_HPP_
#define SMARTER_JOYSTICK_TELEOP__SMARTER_JOYSTICK_TELEOP_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "smarter_joystick_teleop/state_machine.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace smarter_joystick_teleop
{

class SmarterJoystickTeleopNode : public rclcpp::Node
{
public:
  explicit SmarterJoystickTeleopNode(const rclcpp::NodeOptions & options);

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg);
  void watchdogTimerCallback();

  void applyTransitionEffects(const TransitionEffects & effects, const rclcpp::Time & stamp);
  void publishZeroVelocity(const rclcpp::Time & stamp);
  void publishDirectTeleopCommand(const sensor_msgs::msg::Joy & joy, const rclcpp::Time & stamp);
  void maybeSendAssistedGoal(const sensor_msgs::msg::Joy & joy, const rclcpp::Time & stamp);

  bool lookupRobotPose(geometry_msgs::msg::PoseStamped & robot_pose);
  void sendNavigateGoal(const geometry_msgs::msg::PoseStamped & goal_pose);
  void cancelNavigationGoal();

  double getAxis(const sensor_msgs::msg::Joy & joy, int axis) const;
  int getButton(const sensor_msgs::msg::Joy & joy, int button) const;
  static double applyDeadzone(double value, double deadzone);
  static bool isZeroTime(const builtin_interfaces::msg::Time & stamp);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  bool enable_stamped_cmd_vel_{true};
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  TeleopStateMachine state_machine_;

  bool has_received_joy_{false};
  int previous_estop_button_state_{0};
  int previous_submode_button_state_{0};
  rclcpp::Time last_joy_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_assisted_goal_time_{0, 0, RCL_ROS_TIME};

  std::shared_ptr<GoalHandleNavigateToPose> active_goal_handle_;

  std::string joy_topic_;
  std::string cmd_vel_topic_;
  std::string nav_action_name_;
  std::string global_frame_;
  std::string base_frame_;
  std::string cmd_vel_frame_id_;

  int axis_linear_{1};
  int axis_angular_{3};
  int estop_toggle_button_{5};
  int submode_toggle_button_{4};

  double max_linear_speed_{1.0};
  double max_angular_speed_{1.2};
  double linear_deadzone_{0.05};
  double angular_deadzone_{0.05};

  double assisted_max_distance_{2.0};
  double assisted_max_heading_{0.8};
  double assisted_distance_deadzone_{0.15};
  double assisted_heading_deadzone_{0.15};
  double assisted_goal_period_sec_{0.25};

  double joy_timeout_sec_{1.0};
  double watchdog_period_sec_{0.1};
};

}  // namespace smarter_joystick_teleop

#endif  // SMARTER_JOYSTICK_TELEOP__SMARTER_JOYSTICK_TELEOP_HPP_
