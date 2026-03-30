// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

#include <cstdio>

#include "locomotion_specification.h"

namespace {

bool nearly_equal(float lhs, float rhs, float tolerance = 1e-6f)
{
  return lhs >= rhs - tolerance && lhs <= rhs + tolerance;
}

bool expect_true(bool condition, const char *message)
{
  if (!condition)
  {
    std::fprintf(stderr, "[FAIL] %s\n", message);
    return false;
  }
  return true;
}

bool test_leg_specification_supports_custom_bus_names()
{
  const LocomotionRobotSpecification specification =
      build_leg_locomotion_robot_specification("can2", "can3");

  bool ok = true;
  for (size_t index = 0; index < 6; ++index)
  {
    ok &= expect_true(
        specification.joint_addresses[index].bus_name == "can2",
        "left leg joints should use the custom left CAN bus");
  }
  for (size_t index = 6; index < 12; ++index)
  {
    ok &= expect_true(
        specification.joint_addresses[index].bus_name == "can3",
        "right leg joints should use the custom right CAN bus");
  }
  return ok;
}

bool test_leg_specification_preserves_current_native_device_mapping()
{
  const LocomotionRobotSpecification specification = build_leg_locomotion_robot_specification();

  return expect_true(specification.joint_addresses[0].device_id == 5, "left hip roll device id should stay 5") &&
      expect_true(specification.joint_addresses[1].device_id == 3, "left hip yaw device id should stay 3") &&
      expect_true(specification.joint_addresses[2].device_id == 1, "left hip pitch device id should stay 1") &&
      expect_true(specification.joint_addresses[6].device_id == 6, "right hip roll device id should stay 6") &&
      expect_true(specification.joint_addresses[7].device_id == 4, "right hip yaw device id should stay 4") &&
      expect_true(specification.joint_addresses[8].device_id == 2, "right hip pitch device id should stay 2");
}

bool test_leg_specification_exposes_expected_pose_templates()
{
  const LocomotionRobotSpecification specification = build_leg_locomotion_robot_specification();

  return expect_true(
             nearly_equal(specification.initialization_positions[2], -0.2f),
             "initialization pose should expose the default policy-entry hip pitch") &&
      expect_true(
          nearly_equal(specification.standing_positions[3], 0.2f),
          "standing pose should expose the standing knee pitch") &&
      expect_true(
          nearly_equal(
              specification.calibration_reference_positions[4],
              specification.standing_positions[4]),
          "calibration reference pose should match standing pose for the ankle pitch");
}

bool test_leg_specification_exposes_mirrored_joint_pairs()
{
  const LocomotionRobotSpecification specification = build_leg_locomotion_robot_specification();

  return expect_true(
             specification.mirrored_joint_pairs[0].first == 0 &&
                 specification.mirrored_joint_pairs[0].second == 6,
             "hip roll pair should mirror indices 0 and 6") &&
      expect_true(
          specification.mirrored_joint_pairs[5].first == 5 &&
              specification.mirrored_joint_pairs[5].second == 11,
          "ankle roll pair should mirror indices 5 and 11");
}

bool test_hardware_config_specification_uses_configuration_device_ids()
{
  const YAML::Node robot_configuration = YAML::Load(R"(
left_hip_roll_joint: {device_id: 1}
left_hip_yaw_joint: {device_id: 3}
left_hip_pitch_joint: {device_id: 5}
left_knee_pitch_joint: {device_id: 7}
left_ankle_pitch_joint: {device_id: 11}
left_ankle_roll_joint: {device_id: 13}
right_hip_roll_joint: {device_id: 2}
right_hip_yaw_joint: {device_id: 4}
right_hip_pitch_joint: {device_id: 6}
right_knee_pitch_joint: {device_id: 8}
right_ankle_pitch_joint: {device_id: 12}
right_ankle_roll_joint: {device_id: 14}
)");

  const LocomotionRobotSpecification specification =
      build_hardware_config_leg_locomotion_robot_specification(robot_configuration, "can2", "can3");

  return expect_true(specification.joint_addresses[0].bus_name == "can2", "hardware-config spec should keep left bus override") &&
      expect_true(specification.joint_addresses[6].bus_name == "can3", "hardware-config spec should keep right bus override") &&
      expect_true(specification.joint_addresses[0].device_id == 1, "hardware-config spec should use config device id for left hip roll") &&
      expect_true(specification.joint_addresses[2].device_id == 5, "hardware-config spec should use config device id for left hip pitch") &&
      expect_true(specification.joint_addresses[6].device_id == 2, "hardware-config spec should use config device id for right hip roll");
}

bool test_collect_device_id_mismatches_reports_legacy_vs_hardware_config_difference()
{
  const LocomotionRobotSpecification legacy_specification =
      build_legacy_leg_locomotion_robot_specification();
  const YAML::Node robot_configuration = YAML::Load(R"(
left_hip_roll_joint: {device_id: 1}
left_hip_yaw_joint: {device_id: 3}
left_hip_pitch_joint: {device_id: 5}
left_knee_pitch_joint: {device_id: 7}
left_ankle_pitch_joint: {device_id: 11}
left_ankle_roll_joint: {device_id: 13}
right_hip_roll_joint: {device_id: 2}
right_hip_yaw_joint: {device_id: 4}
right_hip_pitch_joint: {device_id: 6}
right_knee_pitch_joint: {device_id: 8}
right_ankle_pitch_joint: {device_id: 12}
right_ankle_roll_joint: {device_id: 14}
)");
  const LocomotionRobotSpecification hardware_specification =
      build_hardware_config_leg_locomotion_robot_specification(robot_configuration);

  const std::vector<SpecificationAddressMismatch> mismatches =
      collect_device_id_mismatches(legacy_specification, hardware_specification);

  return expect_true(mismatches.size() == 4, "legacy and hardware-config specs should differ on four device ids") &&
      expect_true(std::string(mismatches[0].joint_name) == "left_hip_roll_joint", "first mismatch should be left hip roll") &&
      expect_true(mismatches[0].expected_device_id == 5, "legacy left hip roll should stay 5") &&
      expect_true(mismatches[0].actual_device_id == 1, "hardware-config left hip roll should be 1");
}

}  // namespace

int main()
{
  struct TestCase
  {
    const char *name;
    bool (*run)();
  };

  const TestCase tests[] = {
      {"leg_specification_supports_custom_bus_names", test_leg_specification_supports_custom_bus_names},
      {"leg_specification_preserves_current_native_device_mapping", test_leg_specification_preserves_current_native_device_mapping},
      {"leg_specification_exposes_expected_pose_templates", test_leg_specification_exposes_expected_pose_templates},
      {"leg_specification_exposes_mirrored_joint_pairs", test_leg_specification_exposes_mirrored_joint_pairs},
      {"hardware_config_specification_uses_configuration_device_ids", test_hardware_config_specification_uses_configuration_device_ids},
      {"collect_device_id_mismatches_reports_legacy_vs_hardware_config_difference", test_collect_device_id_mismatches_reports_legacy_vs_hardware_config_difference},
  };

  bool ok = true;
  for (const TestCase &test : tests)
  {
    ok &= test.run();
  }

  if (!ok)
  {
    return 1;
  }

  std::puts("test-locomotion-specification passed");
  return 0;
}
