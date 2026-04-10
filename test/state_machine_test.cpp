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

#include <gtest/gtest.h>

namespace smarter_joystick_teleop
{

TEST(TeleopStateMachineTest, startsInEmergencyStop)
{
  TeleopStateMachine machine;
  EXPECT_TRUE(machine.isEmergencyStop());
  EXPECT_EQ(machine.majorMode(), MajorMode::K_EMERGENCY_STOP);
  EXPECT_EQ(machine.freeMode(), FreeMode::K_DIRECT_TELEOP);
}

TEST(TeleopStateMachineTest, emergencyToggleLeavesEmergencyStop)
{
  TeleopStateMachine machine;

  const auto effects = machine.onEmergencyTogglePressed();

  EXPECT_FALSE(machine.isEmergencyStop());
  EXPECT_EQ(machine.majorMode(), MajorMode::K_FREE);
  EXPECT_EQ(machine.freeMode(), FreeMode::K_DIRECT_TELEOP);
  EXPECT_TRUE(effects.mode_changed);
  EXPECT_FALSE(effects.publish_zero_velocity);
  EXPECT_FALSE(effects.cancel_nav2_goal);
}

TEST(TeleopStateMachineTest, directToAssistedPublishesZeroVelocity)
{
  TeleopStateMachine machine;
  (void)machine.onEmergencyTogglePressed();

  const auto effects = machine.onFreeSubmodeTogglePressed();

  EXPECT_EQ(machine.freeMode(), FreeMode::K_ASSISTED_DRIVING);
  EXPECT_TRUE(effects.mode_changed);
  EXPECT_TRUE(effects.publish_zero_velocity);
  EXPECT_FALSE(effects.cancel_nav2_goal);
}

TEST(TeleopStateMachineTest, assistedToDirectCancelsNavigationGoal)
{
  TeleopStateMachine machine;
  (void)machine.onEmergencyTogglePressed();
  (void)machine.onFreeSubmodeTogglePressed();

  const auto effects = machine.onFreeSubmodeTogglePressed();

  EXPECT_EQ(machine.freeMode(), FreeMode::K_DIRECT_TELEOP);
  EXPECT_TRUE(effects.mode_changed);
  EXPECT_FALSE(effects.publish_zero_velocity);
  EXPECT_TRUE(effects.cancel_nav2_goal);
}

TEST(TeleopStateMachineTest, joystickTimeoutSwitchesToEmergencyStop)
{
  TeleopStateMachine machine;
  (void)machine.onEmergencyTogglePressed();

  const auto effects = machine.onJoystickTimeout();

  EXPECT_TRUE(machine.isEmergencyStop());
  EXPECT_TRUE(effects.mode_changed);
  EXPECT_TRUE(effects.publish_zero_velocity);
  EXPECT_TRUE(effects.cancel_nav2_goal);
}

TEST(TeleopStateMachineTest, timeoutInEmergencyStopHasNoEffects)
{
  TeleopStateMachine machine;

  const auto effects = machine.onJoystickTimeout();

  EXPECT_TRUE(machine.isEmergencyStop());
  EXPECT_FALSE(effects.mode_changed);
  EXPECT_FALSE(effects.publish_zero_velocity);
  EXPECT_FALSE(effects.cancel_nav2_goal);
}

TEST(TeleopStateMachineTest, assistedDistanceAxisCancelOnRelease)
{
  TeleopStateMachine machine;
  (void)machine.onEmergencyTogglePressed();
  (void)machine.onFreeSubmodeTogglePressed();

  const auto active_effects = machine.onAssistedDistanceAxis(0.8);
  EXPECT_TRUE(active_effects.allow_goal_update);
  EXPECT_FALSE(active_effects.cancel_nav2_goal);

  const auto release_effects = machine.onAssistedDistanceAxis(0.0);
  EXPECT_FALSE(release_effects.allow_goal_update);
  EXPECT_TRUE(release_effects.cancel_nav2_goal);

  const auto hold_zero_effects = machine.onAssistedDistanceAxis(0.0);
  EXPECT_FALSE(hold_zero_effects.allow_goal_update);
  EXPECT_FALSE(hold_zero_effects.cancel_nav2_goal);
}

TEST(TeleopStateMachineTest, assistedDistanceAxisIgnoredOutsideAssistedMode)
{
  TeleopStateMachine machine;

  const auto emergency_effects = machine.onAssistedDistanceAxis(0.8);
  EXPECT_FALSE(emergency_effects.allow_goal_update);
  EXPECT_FALSE(emergency_effects.cancel_nav2_goal);

  (void)machine.onEmergencyTogglePressed();
  const auto direct_effects = machine.onAssistedDistanceAxis(0.8);
  EXPECT_FALSE(direct_effects.allow_goal_update);
  EXPECT_FALSE(direct_effects.cancel_nav2_goal);
}

}  // namespace smarter_joystick_teleop
