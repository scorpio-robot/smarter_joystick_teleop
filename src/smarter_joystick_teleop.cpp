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

#include "smarter_joystick_teleop/smarter_joystick_teleop.hpp"

#include <cmath>
#include <functional>

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace smarter_joystick_teleop
{

SmarterJoystickTeleopNode::SmarterJoystickTeleopNode(const rclcpp::NodeOptions & options)
: Node("smarter_joystick_teleop", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");
  cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  nav_action_name_ = this->declare_parameter<std::string>("nav_action_name", "navigate_to_pose");
  global_frame_ = this->declare_parameter<std::string>("global_frame", "map");
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  cmd_vel_frame_id_ = this->declare_parameter<std::string>("cmd_vel_frame_id", "base_link");
  enable_stamped_cmd_vel_ = this->declare_parameter<bool>("enable_stamped_cmd_vel", true);

  axis_linear_ = this->declare_parameter<int>("axis_linear", 1);
  axis_angular_ = this->declare_parameter<int>("axis_angular", 3);
  estop_toggle_button_ = this->declare_parameter<int>("estop_toggle_button", 5);
  submode_toggle_button_ = this->declare_parameter<int>("submode_toggle_button", 4);

  max_linear_speed_ = this->declare_parameter<double>("max_linear_speed", 1.0);
  max_angular_speed_ = this->declare_parameter<double>("max_angular_speed", 1.2);
  linear_deadzone_ = this->declare_parameter<double>("linear_deadzone", 0.05);
  angular_deadzone_ = this->declare_parameter<double>("angular_deadzone", 0.05);

  assisted_max_distance_ = this->declare_parameter<double>("assisted_max_distance", 2.0);
  assisted_max_heading_ = this->declare_parameter<double>("assisted_max_heading", 0.8);
  assisted_distance_deadzone_ = this->declare_parameter<double>("assisted_distance_deadzone", 0.15);
  assisted_heading_deadzone_ = this->declare_parameter<double>("assisted_heading_deadzone", 0.15);
  assisted_goal_period_sec_ = this->declare_parameter<double>("assisted_goal_period_sec", 0.25);

  joy_timeout_sec_ = this->declare_parameter<double>("joy_timeout_sec", 1.0);
  watchdog_period_sec_ = this->declare_parameter<double>("watchdog_period_sec", 0.1);

  if (estop_toggle_button_ == submode_toggle_button_) {
    RCLCPP_WARN(
      this->get_logger(),
      "estop_toggle_button and submode_toggle_button are identical (%d). "
      "Only emergency toggle is guaranteed in this configuration.",
      estop_toggle_button_);
  }

  if (enable_stamped_cmd_vel_) {
    cmd_vel_stamped_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>(cmd_vel_topic_, 10);
  } else {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  }
  nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, nav_action_name_);

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    joy_topic_, 20,
    std::bind(&SmarterJoystickTeleopNode::joyCallback, this, std::placeholders::_1));

  const auto watchdog_period = std::chrono::duration<double>(watchdog_period_sec_);
  watchdog_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(watchdog_period),
    std::bind(&SmarterJoystickTeleopNode::watchdogTimerCallback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Started in mode %s. estop_toggle_button=%d, "
    "submode_toggle_button=%d, joy_timeout=%.2fs",
    state_machine_.modeString().c_str(), estop_toggle_button_, submode_toggle_button_,
    joy_timeout_sec_);
}

void SmarterJoystickTeleopNode::joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  const rclcpp::Time stamp =
    isZeroTime(msg->header.stamp) ? this->now() : rclcpp::Time(msg->header.stamp);

  has_received_joy_ = true;
  last_joy_time_ = stamp;

  const int estop_button_state = getButton(*msg, estop_toggle_button_);
  const int submode_button_state = getButton(*msg, submode_toggle_button_);

  const bool estop_rising_edge = estop_button_state > 0 && previous_estop_button_state_ == 0;
  const bool submode_rising_edge = submode_button_state > 0 && previous_submode_button_state_ == 0;

  previous_estop_button_state_ = estop_button_state;
  previous_submode_button_state_ = submode_button_state;

  if (estop_rising_edge) {
    const auto effects = state_machine_.onEmergencyTogglePressed();
    applyTransitionEffects(effects, stamp);
  }

  if (submode_rising_edge) {
    const auto effects = state_machine_.onFreeSubmodeTogglePressed();
    applyTransitionEffects(effects, stamp);
  }

  if (state_machine_.isEmergencyStop()) {
    return;
  }

  if (state_machine_.freeMode() == FreeMode::K_DIRECT_TELEOP) {
    publishDirectTeleopCommand(*msg, stamp);
  } else {
    maybeSendAssistedGoal(*msg, stamp);
  }
}

void SmarterJoystickTeleopNode::watchdogTimerCallback()
{
  if (!has_received_joy_) {
    return;
  }

  const auto now = this->now();
  const double dt = (now - last_joy_time_).seconds();
  if (dt <= joy_timeout_sec_) {
    return;
  }

  const auto effects = state_machine_.onJoystickTimeout();
  if (effects.mode_changed) {
    RCLCPP_WARN(
      this->get_logger(),
      "Joystick timeout detected (%.3fs > %.3fs), switching to "
      "emergency stop",
      dt, joy_timeout_sec_);
  }
  applyTransitionEffects(effects, now);
}

void SmarterJoystickTeleopNode::applyTransitionEffects(
  const TransitionEffects & effects, const rclcpp::Time & stamp)
{
  if (effects.mode_changed) {
    RCLCPP_INFO(this->get_logger(), "Mode changed to %s", state_machine_.modeString().c_str());
  }

  if (effects.cancel_nav2_goal) {
    cancelNavigationGoal();
  }

  if (effects.publish_zero_velocity) {
    publishZeroVelocity(stamp);
  }
}

void SmarterJoystickTeleopNode::publishZeroVelocity(const rclcpp::Time & stamp)
{
  if (enable_stamped_cmd_vel_ && cmd_vel_stamped_pub_) {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = stamp;
    cmd.header.frame_id = cmd_vel_frame_id_;
    cmd_vel_stamped_pub_->publish(cmd);
  } else if (cmd_vel_pub_) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
  }
}

void SmarterJoystickTeleopNode::publishDirectTeleopCommand(
  const sensor_msgs::msg::Joy & joy, const rclcpp::Time & stamp)
{
  const double linear_axis = applyDeadzone(getAxis(joy, axis_linear_), linear_deadzone_);
  const double angular_axis = applyDeadzone(getAxis(joy, axis_angular_), angular_deadzone_);

  if (enable_stamped_cmd_vel_ && cmd_vel_stamped_pub_) {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = stamp;
    cmd.header.frame_id = cmd_vel_frame_id_;
    cmd.twist.linear.x = linear_axis * max_linear_speed_;
    cmd.twist.angular.z = angular_axis * max_angular_speed_;
    cmd_vel_stamped_pub_->publish(cmd);
  } else if (cmd_vel_pub_) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_axis * max_linear_speed_;
    cmd.angular.z = angular_axis * max_angular_speed_;
    cmd_vel_pub_->publish(cmd);
  }
}

void SmarterJoystickTeleopNode::maybeSendAssistedGoal(
  const sensor_msgs::msg::Joy & joy, const rclcpp::Time & stamp)
{
  const double distance_axis =
    applyDeadzone(getAxis(joy, axis_linear_), assisted_distance_deadzone_);
  const double heading_axis =
    applyDeadzone(getAxis(joy, axis_angular_), assisted_heading_deadzone_);

  const auto assisted_effects = state_machine_.onAssistedDistanceAxis(distance_axis);
  if (assisted_effects.cancel_nav2_goal) {
    cancelNavigationGoal();
  }
  if (!assisted_effects.allow_goal_update) {
    return;
  }

  if (last_assisted_goal_time_.nanoseconds() > 0) {
    const double dt = (stamp - last_assisted_goal_time_).seconds();
    if (dt < assisted_goal_period_sec_) {
      return;
    }
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!lookupRobotPose(robot_pose)) {
    return;
  }

  const double heading = heading_axis * assisted_max_heading_;
  const double distance = distance_axis * assisted_max_distance_;

  const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
  const double goal_yaw = robot_yaw + heading;

  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = stamp;
  goal.header.frame_id = global_frame_;
  goal.pose.position.x = robot_pose.pose.position.x + distance * std::cos(goal_yaw);
  goal.pose.position.y = robot_pose.pose.position.y + distance * std::sin(goal_yaw);
  goal.pose.position.z = robot_pose.pose.position.z;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, goal_yaw);
  goal.pose.orientation = tf2::toMsg(q);

  sendNavigateGoal(goal);
  last_assisted_goal_time_ = stamp;
}

bool SmarterJoystickTeleopNode::lookupRobotPose(geometry_msgs::msg::PoseStamped & robot_pose)
{
  try {
    const auto tf = tf_buffer_.lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);

    robot_pose.header = tf.header;
    robot_pose.pose.position.x = tf.transform.translation.x;
    robot_pose.pose.position.y = tf.transform.translation.y;
    robot_pose.pose.position.z = tf.transform.translation.z;
    robot_pose.pose.orientation = tf.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Cannot lookup %s -> %s transform: %s",
      global_frame_.c_str(), base_frame_.c_str(), ex.what());
    return false;
  }
}

void SmarterJoystickTeleopNode::sendNavigateGoal(const geometry_msgs::msg::PoseStamped & goal_pose)
{
  if (!nav_to_pose_client_->wait_for_action_server(std::chrono::milliseconds(1))) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "NavigateToPose action server %s is not ready",
      nav_action_name_.c_str());
    return;
  }

  NavigateToPose::Goal goal;
  goal.pose = goal_pose;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback = [this](std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
    if (!goal_handle) {
      RCLCPP_WARN(this->get_logger(), "NavigateToPose goal rejected");
      return;
    }
    active_goal_handle_ = goal_handle;
  };

  options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult & result) {
    (void)result;
    active_goal_handle_.reset();
  };

  nav_to_pose_client_->async_send_goal(goal, options);
}

void SmarterJoystickTeleopNode::cancelNavigationGoal()
{
  if (!nav_to_pose_client_->wait_for_action_server(std::chrono::milliseconds(1))) {
    return;
  }

  if (active_goal_handle_) {
    nav_to_pose_client_->async_cancel_goal(active_goal_handle_);
    return;
  }

  nav_to_pose_client_->async_cancel_all_goals();
}

double SmarterJoystickTeleopNode::getAxis(const sensor_msgs::msg::Joy & joy, int axis) const
{
  if (axis < 0 || static_cast<size_t>(axis) >= joy.axes.size()) {
    return 0.0;
  }
  return static_cast<double>(joy.axes[static_cast<size_t>(axis)]);
}

int SmarterJoystickTeleopNode::getButton(const sensor_msgs::msg::Joy & joy, int button) const
{
  if (button < 0 || static_cast<size_t>(button) >= joy.buttons.size()) {
    return 0;
  }
  return joy.buttons[static_cast<size_t>(button)];
}

double SmarterJoystickTeleopNode::applyDeadzone(double value, double deadzone)
{
  if (std::abs(value) < deadzone) {
    return 0.0;
  }
  return value;
}

bool SmarterJoystickTeleopNode::isZeroTime(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec == 0 && stamp.nanosec == 0;
}

}  // namespace smarter_joystick_teleop

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(smarter_joystick_teleop::SmarterJoystickTeleopNode)
