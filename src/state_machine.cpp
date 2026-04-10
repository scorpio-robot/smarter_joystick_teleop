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

#include "smarter_joystick_teleop/state_machine.hpp"

#include <cmath>

namespace smarter_joystick_teleop
{

TeleopStateMachine::TeleopStateMachine()
: major_mode_(MajorMode::K_EMERGENCY_STOP),
  free_mode_(FreeMode::K_DIRECT_TELEOP),
  assisted_session_active_(false)
{
}

TransitionEffects TeleopStateMachine::onEmergencyTogglePressed()
{
  TransitionEffects effects;

  if (major_mode_ == MajorMode::K_EMERGENCY_STOP) {
    major_mode_ = MajorMode::K_FREE;
    free_mode_ = FreeMode::K_DIRECT_TELEOP;
    assisted_session_active_ = false;
    effects.mode_changed = true;
    return effects;
  }

  major_mode_ = MajorMode::K_EMERGENCY_STOP;
  assisted_session_active_ = false;
  effects.mode_changed = true;
  effects.cancel_nav2_goal = true;
  effects.publish_zero_velocity = true;
  return effects;
}

TransitionEffects TeleopStateMachine::onFreeSubmodeTogglePressed()
{
  TransitionEffects effects;

  if (major_mode_ != MajorMode::K_FREE) {
    return effects;
  }

  if (free_mode_ == FreeMode::K_DIRECT_TELEOP) {
    free_mode_ = FreeMode::K_ASSISTED_DRIVING;
    assisted_session_active_ = false;
    effects.mode_changed = true;
    effects.publish_zero_velocity = true;
    return effects;
  }

  free_mode_ = FreeMode::K_DIRECT_TELEOP;
  assisted_session_active_ = false;
  effects.mode_changed = true;
  effects.cancel_nav2_goal = true;
  return effects;
}

TransitionEffects TeleopStateMachine::onJoystickTimeout()
{
  TransitionEffects effects;
  if (major_mode_ == MajorMode::K_EMERGENCY_STOP) {
    return effects;
  }

  major_mode_ = MajorMode::K_EMERGENCY_STOP;
  assisted_session_active_ = false;
  effects.mode_changed = true;
  effects.cancel_nav2_goal = true;
  effects.publish_zero_velocity = true;
  return effects;
}

AssistedCommandEffects TeleopStateMachine::onAssistedDistanceAxis(const double distance_axis)
{
  AssistedCommandEffects effects;

  if (major_mode_ != MajorMode::K_FREE || free_mode_ != FreeMode::K_ASSISTED_DRIVING) {
    assisted_session_active_ = false;
    return effects;
  }

  const bool currently_active = std::abs(distance_axis) >= 1e-6;
  effects.allow_goal_update = currently_active;

  if (!currently_active && assisted_session_active_) {
    effects.cancel_nav2_goal = true;
  }

  assisted_session_active_ = currently_active;
  return effects;
}

MajorMode TeleopStateMachine::majorMode() const { return major_mode_; }

FreeMode TeleopStateMachine::freeMode() const { return free_mode_; }

bool TeleopStateMachine::isEmergencyStop() const
{
  return major_mode_ == MajorMode::K_EMERGENCY_STOP;
}

std::string TeleopStateMachine::modeString() const
{
  if (major_mode_ == MajorMode::K_EMERGENCY_STOP) {
    return "EMERGENCY_STOP";
  }

  if (free_mode_ == FreeMode::K_DIRECT_TELEOP) {
    return "FREE_DIRECT";
  }

  return "FREE_ASSISTED";
}

}  // namespace smarter_joystick_teleop
