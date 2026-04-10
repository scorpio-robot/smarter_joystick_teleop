# Smarter Joystick Teleop

[![ROS 2](https://img.shields.io/badge/ROS2-Jazzy-blue?logo=ros)](https://docs.ros.org/en/jazzy/index.html)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)
[![Build and Test](https://github.com/scorpio-robot/smarter_joystick_teleop/actions/workflows/build_and_test.yml/badge.svg)](https://github.com/scorpio-robot/smarter_joystick_teleop/actions/workflows/build_and_test.yml)

`smarter_joystick_teleop` is a C++ ROS 2 node that adds a mode-aware joystick control layer on top of Nav2.

It supports:

- emergency stop mode
- direct teleop mode
- Nav2 assisted driving mode

## Mode Design

### Emergency Stop Mode

Initial state is emergency stop mode.

Right shoulder button (`buttons[5]`) rising edge toggles emergency stop and free mode.

When emergency stop is entered:

- cancel current Nav2 navigation action
- publish zero on `cmd_vel` (uses `TwistStamped` by default; set `enable_stamped_cmd_vel` to `false` to publish `Twist`)

### Free Mode

Inside free mode, left shoulder button (`buttons[4]`) rising edge toggles submodes.

Submode transitions:

- direct -> assisted: publish zero velocity first
- assisted -> direct: cancel current Nav2 action first

### Direct Teleop Mode

- `axes[1]` controls linear velocity
- `axes[3]` controls angular velocity
- output topic: `cmd_vel` (publishes `TwistStamped` by default; set `enable_stamped_cmd_vel` to `false` to publish `Twist`)

### Assisted Driving Mode

- `axes[1]` controls goal distance
- `axes[3]` controls goal heading offset
- goals are sent via `nav2_msgs/action/NavigateToPose` (default action name: `navigate_to_pose`)
- when `axes[1]` returns to zero, current Nav2 action is canceled

### Joystick Disconnect Rule

- if joystick data has never been received since startup: do not treat as disconnect
- after first joystick message is received, if no new message arrives for `joy_timeout_sec` (default `1.0s`), switch to emergency stop mode

## Default Joy Mapping

This package defaults match [config/smarter_joystick_teleop.yaml](config/smarter_joystick_teleop.yaml):

- `axes[0]`: left stick left/right
- `axes[1]`: left stick forward/backward
- `axes[3]`: right stick left/right
- `axes[4]`: right stick forward/backward
- `buttons[4]`: left shoulder
- `buttons[5]`: right shoulder

## Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `joy_topic` | string | `/joy` | Joystick topic |
| `cmd_vel_topic` | string | `/cmd_vel` | TwistStamped output topic |
| `nav_action_name` | string | `navigate_to_pose` | Nav2 action server name |
| `global_frame` | string | `map` | Goal frame for assisted mode |
| `base_frame` | string | `base_link` | Robot frame used for TF lookup |
| `cmd_vel_frame_id` | string | `base_link` | Frame id in published TwistStamped |
| `axis_linear` | int | `1` | Linear control axis |
| `axis_angular` | int | `3` | Angular control axis |
| `estop_toggle_button` | int | `5` | Toggle emergency/free mode |
| `submode_toggle_button` | int | `4` | Toggle free submode |
| `max_linear_speed` | double | `1.0` | Direct mode max linear speed |
| `max_angular_speed` | double | `1.2` | Direct mode max angular speed |
| `linear_deadzone` | double | `0.05` | Linear axis deadzone |
| `angular_deadzone` | double | `0.05` | Angular axis deadzone |
| `assisted_max_distance` | double | `2.0` | Assisted max goal distance |
| `assisted_max_heading` | double | `0.8` | Assisted max heading offset (rad) |
| `assisted_distance_deadzone` | double | `0.15` | Assisted distance deadzone |
| `assisted_heading_deadzone` | double | `0.15` | Assisted heading deadzone |
| `assisted_goal_period_sec` | double | `0.25` | Minimum interval between assisted goals |
| `joy_timeout_sec` | double | `1.0` | Timeout to trigger emergency stop |
| `watchdog_period_sec` | double | `0.1` | Watchdog timer period |
| `enable_stamped_cmd_vel` | bool | `true` | Publish `geometry_msgs/msg/TwistStamped` when true, otherwise publish `geometry_msgs/msg/Twist` |

## Build

```bash
cd ~/ros_ws
colcon build --symlink-install --packages-up-to smarter_joystick_teleop
```

## Run

```bash
ros2 launch smarter_joystick_teleop smarter_joystick_teleop_launch.py
```
