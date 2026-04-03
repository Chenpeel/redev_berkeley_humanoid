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

enum class CalibrationLimitSelector {
  Min = 0,
  Max = 1,
};

static constexpr std::array<const char *, N_JOINTS> kLegLocomotionJointNames = {{
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
  std::array<CalibrationLimitSelector, N_JOINTS> calibration_limit_selectors{};
};

struct SpecificationAddressMismatch {
  size_t index = 0;
  const char *joint_name = "";
  uint8_t expected_device_id = 0;
  uint8_t actual_device_id = 0;
};

struct HardwareConfigurationFieldIssue {
  size_t index = 0;
  const char *joint_name = "";
  const char *field_path = "";
};

struct HardwareConfigurationAudit {
  size_t expected_joint_count = N_JOINTS;
  std::vector<const char *> missing_joints;
  std::vector<HardwareConfigurationFieldIssue> missing_required_fields;
  std::vector<SpecificationAddressMismatch> device_id_mismatches;

  bool is_complete() const {
    return missing_joints.empty() && missing_required_fields.empty();
  }

  bool matches_expected_spec() const {
    return is_complete() && device_id_mismatches.empty();
  }
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

inline const char *calibration_limit_selector_name(CalibrationLimitSelector selector) {
  switch (selector) {
    case CalibrationLimitSelector::Min:
      return "min";
    case CalibrationLimitSelector::Max:
      return "max";
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
  specification->calibration_limit_selectors = {{
      CalibrationLimitSelector::Min,
      CalibrationLimitSelector::Max,
      CalibrationLimitSelector::Min,
      CalibrationLimitSelector::Min,
      CalibrationLimitSelector::Max,
      CalibrationLimitSelector::Min,
      CalibrationLimitSelector::Min,
      CalibrationLimitSelector::Min,
      CalibrationLimitSelector::Max,
      CalibrationLimitSelector::Min,
      CalibrationLimitSelector::Min,
      CalibrationLimitSelector::Max,
  }};
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
  LocomotionRobotSpecification specification;
  for (size_t index = 0; index < kLegLocomotionJointNames.size(); ++index) {
    const char *joint_name = kLegLocomotionJointNames[index];
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

inline HardwareConfigurationAudit audit_leg_hardware_configuration(
    const YAML::Node &robot_configuration,
    const LocomotionRobotSpecification *expected_specification = nullptr) {
  struct JointConfigurationFieldRequirement {
    const char *group_name;
    const char *field_name;
    const char *field_path;
  };
  static constexpr std::array<JointConfigurationFieldRequirement, 4> kRequiredFields = {{
      {"position_controller", "gear_ratio", "position_controller.gear_ratio"},
      {"motor", "phase_order", "motor.phase_order"},
      {"motor", "max_calibration_current", "motor.max_calibration_current"},
      {"encoder", "position_offset", "encoder.position_offset"},
  }};

  HardwareConfigurationAudit audit;
  for (size_t index = 0; index < kLegLocomotionJointNames.size(); ++index) {
    const char *joint_name = kLegLocomotionJointNames[index];
    const YAML::Node joint_config = robot_configuration[joint_name];
    if (!joint_config) {
      audit.missing_joints.push_back(joint_name);
      continue;
    }

    const YAML::Node device_id_node = joint_config["device_id"];
    if (!device_id_node) {
      audit.missing_required_fields.push_back({index, joint_name, "device_id"});
    } else if (expected_specification != nullptr) {
      const uint8_t actual_device_id = static_cast<uint8_t>(device_id_node.as<int>());
      const uint8_t expected_device_id =
          expected_specification->joint_addresses[index].device_id;
      if (expected_device_id != actual_device_id) {
        audit.device_id_mismatches.push_back({
            index,
            joint_name,
            expected_device_id,
            actual_device_id,
        });
      }
    }

    for (const JointConfigurationFieldRequirement &required_field : kRequiredFields) {
      const YAML::Node group_node = joint_config[required_field.group_name];
      if (!group_node || !group_node[required_field.field_name]) {
        audit.missing_required_fields.push_back({
            index,
            joint_name,
            required_field.field_path,
        });
      }
    }
  }

  return audit;
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
