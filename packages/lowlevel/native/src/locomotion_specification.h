// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "consts.h"
#include "locomotion_parity.h"

constexpr const char *DEFAULT_LEFT_LEG_BUS = "can0";
constexpr const char *DEFAULT_RIGHT_LEG_BUS = "can1";
constexpr const char *DEFAULT_HARDWARE_CONFIGURATION_PATH = "configs/hardware/robot_configuration.json";

enum class LocomotionSpecificationSource {
  LegacyNative = 0,
  HardwareConfiguration = 1,
};

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

struct SpecificationAddressMismatch {
  size_t index = 0;
  const char *joint_name = "";
  uint8_t expected_device_id = 0;
  uint8_t actual_device_id = 0;
};

inline const char *locomotion_specification_source_name(LocomotionSpecificationSource source) {
  switch (source) {
    case LocomotionSpecificationSource::LegacyNative:
      return "legacy";
    case LocomotionSpecificationSource::HardwareConfiguration:
      return "hardware-config";
  }
  return "unknown";
}

inline void populate_common_leg_locomotion_specification(LocomotionRobotSpecification *specification) {
  specification->mirrored_joint_pairs = {{
      {0, 6},
      {1, 7},
      {2, 8},
      {3, 9},
      {4, 10},
      {5, 11},
  }};

  specification->joint_axis_directions = {{
      -1.0f, 1.0f, -1.0f,
      -1.0f,
      -1.0f, 1.0f,
      -1.0f, 1.0f, 1.0f,
      1.0f,
      1.0f, 1.0f,
  }};

  specification->initialization_positions = kDefaultPolicyEntryPositions;
  specification->standing_positions = kDefaultStandingPositions;
  specification->calibration_reference_positions = kDefaultStandingPositions;
}

inline LocomotionRobotSpecification build_legacy_leg_locomotion_robot_specification(
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

  populate_common_leg_locomotion_specification(&specification);
  return specification;
}

inline LocomotionRobotSpecification build_leg_locomotion_robot_specification(
    const std::string &left_leg_bus = DEFAULT_LEFT_LEG_BUS,
    const std::string &right_leg_bus = DEFAULT_RIGHT_LEG_BUS) {
  return build_legacy_leg_locomotion_robot_specification(left_leg_bus, right_leg_bus);
}

inline LocomotionRobotSpecification build_hardware_config_leg_locomotion_robot_specification(
    const YAML::Node &robot_configuration,
    const std::string &left_leg_bus = DEFAULT_LEFT_LEG_BUS,
    const std::string &right_leg_bus = DEFAULT_RIGHT_LEG_BUS) {
  static constexpr std::array<const char *, N_JOINTS> kJointNames = {{
      "left_hip_roll_joint",
      "left_hip_yaw_joint",
      "left_hip_pitch_joint",
      "left_knee_pitch_joint",
      "left_ankle_pitch_joint",
      "left_ankle_roll_joint",
      "right_hip_roll_joint",
      "right_hip_yaw_joint",
      "right_hip_pitch_joint",
      "right_knee_pitch_joint",
      "right_ankle_pitch_joint",
      "right_ankle_roll_joint",
  }};

  LocomotionRobotSpecification specification;
  for (size_t index = 0; index < kJointNames.size(); ++index) {
    const char *joint_name = kJointNames[index];
    const YAML::Node joint_config = robot_configuration[joint_name];
    if (!joint_config || !joint_config["device_id"]) {
      throw std::runtime_error(std::string("Missing device_id for joint in hardware config: ") + joint_name);
    }

    specification.joint_addresses[index] = JointTransportAddress{
        index < 6 ? left_leg_bus : right_leg_bus,
        static_cast<uint8_t>(joint_config["device_id"].as<int>()),
        joint_name,
    };
  }

  populate_common_leg_locomotion_specification(&specification);
  return specification;
}

inline std::vector<SpecificationAddressMismatch> collect_device_id_mismatches(
    const LocomotionRobotSpecification &expected,
    const LocomotionRobotSpecification &actual) {
  std::vector<SpecificationAddressMismatch> mismatches;
  for (size_t index = 0; index < expected.joint_addresses.size(); ++index) {
    if (expected.joint_addresses[index].device_id == actual.joint_addresses[index].device_id) {
      continue;
    }
    mismatches.push_back(SpecificationAddressMismatch{
        index,
        expected.joint_addresses[index].joint_name,
        expected.joint_addresses[index].device_id,
        actual.joint_addresses[index].device_id,
    });
  }
  return mismatches;
}
