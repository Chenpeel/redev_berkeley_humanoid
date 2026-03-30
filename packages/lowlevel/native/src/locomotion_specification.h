// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

#include "consts.h"
#include "locomotion_parity.h"

constexpr const char *DEFAULT_LEFT_LEG_BUS = "can0";
constexpr const char *DEFAULT_RIGHT_LEG_BUS = "can1";

struct JointTransportAddress {
  std::string bus_name;
  uint8_t device_id = 0;
  const char *joint_name = "";
};

struct LocomotionRobotSpecification {
  std::array<JointTransportAddress, N_JOINTS> joint_addresses{};
  std::array<std::pair<size_t, size_t>, N_JOINTS / 2> mirrored_joint_pairs{};
  JointArray joint_axis_directions{};
  JointArray initialization_positions{};
  JointArray standing_positions{};
  JointArray calibration_reference_positions{};
};

inline LocomotionRobotSpecification build_leg_locomotion_robot_specification(
    const std::string &left_leg_bus = DEFAULT_LEFT_LEG_BUS,
    const std::string &right_leg_bus = DEFAULT_RIGHT_LEG_BUS) {
  LocomotionRobotSpecification specification;

  // Preserve the current native runtime actuator mapping.
  specification.joint_addresses = {{
      {left_leg_bus, 5, "left_hip_roll_joint"},
      {left_leg_bus, 3, "left_hip_yaw_joint"},
      {left_leg_bus, 1, "left_hip_pitch_joint"},
      {left_leg_bus, 7, "left_knee_pitch_joint"},
      {left_leg_bus, 11, "left_ankle_pitch_joint"},
      {left_leg_bus, 13, "left_ankle_roll_joint"},
      {right_leg_bus, 6, "right_hip_roll_joint"},
      {right_leg_bus, 4, "right_hip_yaw_joint"},
      {right_leg_bus, 2, "right_hip_pitch_joint"},
      {right_leg_bus, 8, "right_knee_pitch_joint"},
      {right_leg_bus, 12, "right_ankle_pitch_joint"},
      {right_leg_bus, 14, "right_ankle_roll_joint"},
  }};

  specification.mirrored_joint_pairs = {{
      {0, 6},
      {1, 7},
      {2, 8},
      {3, 9},
      {4, 10},
      {5, 11},
  }};

  specification.joint_axis_directions = {{
      -1.0f, 1.0f, -1.0f,
      -1.0f,
      -1.0f, 1.0f,
      -1.0f, 1.0f, 1.0f,
      1.0f,
      1.0f, 1.0f,
  }};

  specification.initialization_positions = kDefaultPolicyEntryPositions;
  specification.standing_positions = kDefaultStandingPositions;
  specification.calibration_reference_positions = kDefaultStandingPositions;
  return specification;
}
