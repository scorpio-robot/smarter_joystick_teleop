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

#ifndef SMARTER_JOYSTICK_TELEOP__STATE_MACHINE_HPP_
#define SMARTER_JOYSTICK_TELEOP__STATE_MACHINE_HPP_

#include <cstdint>
#include <string>

namespace smarter_joystick_teleop
{

enum class MajorMode : std::uint8_t {
  K_EMERGENCY_STOP = 0,
  K_FREE = 1,
};

enum class FreeMode : std::uint8_t {
  K_DIRECT_TELEOP = 0,
  K_ASSISTED_DRIVING = 1,
};

struct TransitionEffects
{
  bool mode_changed{false};
  bool publish_zero_velocity{false};
  bool cancel_nav2_goal{false};
};

struct AssistedCommandEffects
{
  bool allow_goal_update{false};
  bool cancel_nav2_goal{false};
};

class TeleopStateMachine
{
public:
  TeleopStateMachine();

  TransitionEffects onEmergencyTogglePressed();
  TransitionEffects onFreeSubmodeTogglePressed();
  TransitionEffects onJoystickTimeout();
  AssistedCommandEffects onAssistedDistanceAxis(double distance_axis);

  MajorMode majorMode() const;
  FreeMode freeMode() const;
  bool isEmergencyStop() const;
  std::string modeString() const;

private:
  MajorMode major_mode_;
  FreeMode free_mode_;
  bool assisted_session_active_;
};

}  // namespace smarter_joystick_teleop

#endif  // SMARTER_JOYSTICK_TELEOP__STATE_MACHINE_HPP_
