// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

#include <cmath>
#include <cstdio>

#include "locomotion_parity.h"

namespace {

bool nearly_equal(float lhs, float rhs, float tolerance = 1e-6f)
{
  return std::fabs(lhs - rhs) <= tolerance;
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

bool expect_joint_array(const JointArray &lhs, const JointArray &rhs, const char *message)
{
  for (size_t index = 0; index < lhs.size(); ++index)
  {
    if (!nearly_equal(lhs[index], rhs[index]))
    {
      std::fprintf(
          stderr,
          "[FAIL] %s at index %zu lhs=%.6f rhs=%.6f\n",
          message,
          index,
          lhs[index],
          rhs[index]);
      return false;
    }
  }
  return true;
}

bool test_pose_alignment_bias_round_trip()
{
  JointArray hardware_positions{};
  JointArray bias{};
  hardware_positions[0] = 0.10f;
  hardware_positions[1] = -0.30f;
  bias[0] = 0.05f;
  bias[1] = -0.02f;

  const JointArray policy_positions = apply_pose_alignment_bias(hardware_positions, bias);
  const JointArray recovered_hardware = remove_pose_alignment_bias(policy_positions, bias);

  return expect_joint_array(recovered_hardware, hardware_positions, "pose alignment bias should round-trip");
}

bool test_pose_delta_summary_reports_worst_joint()
{
  JointArray target = kDefaultStandingPositions;
  JointArray measured = kDefaultStandingPositions;
  measured[4] += 15.0f * static_cast<float>(M_PI) / 180.0f;

  const PoseDeltaSummary summary = compute_pose_delta_summary(target, measured);

  return expect_true(summary.worst_index == 4, "worst joint index should match largest delta") &&
      expect_true(summary.max_abs_delta_deg > 14.9f, "max_abs_delta_deg should be computed in degrees");
}

bool test_initialization_request_selects_standing_target()
{
  const InitializationDecision decision = resolve_initialization_decision(
      STATE_IDLE,
      STATE_RL_INIT,
      0.0f,
      InitializationTarget::kPolicyEntry,
      kDefaultStandingPositions,
      kDefaultPolicyEntryPositions,
      kDefaultStandingPositions,
      kPolicyEntryGateMaxAbsDeltaDeg);

  return expect_true(decision.target == InitializationTarget::kStanding, "RL init should use standing target") &&
      expect_joint_array(decision.target_positions, kDefaultStandingPositions, "standing target should match default");
}

bool test_policy_request_from_standing_restarts_initialization()
{
  const InitializationDecision decision = resolve_initialization_decision(
      STATE_RL_INIT,
      STATE_RL_RUNNING,
      1.0f,
      InitializationTarget::kStanding,
      kDefaultStandingPositions,
      kDefaultPolicyEntryPositions,
      kDefaultStandingPositions,
      kPolicyEntryGateMaxAbsDeltaDeg);

  return expect_true(decision.restart_initialization, "policy request from standing hold should restart init") &&
      expect_true(decision.target == InitializationTarget::kPolicyEntry, "restart should target policy entry pose");
}

bool test_policy_gate_blocks_large_delta()
{
  JointArray measured = kDefaultStandingPositions;
  measured[2] += 20.0f * static_cast<float>(M_PI) / 180.0f;

  const InitializationDecision decision = resolve_initialization_decision(
      STATE_IDLE,
      STATE_RL_RUNNING,
      0.0f,
      InitializationTarget::kPolicyEntry,
      kDefaultStandingPositions,
      kDefaultPolicyEntryPositions,
      measured,
      kPolicyEntryGateMaxAbsDeltaDeg);

  return expect_true(decision.policy_gate_blocked, "policy gate should block large standing deltas") &&
      expect_true(decision.effective_requested_state == STATE_IDLE, "blocked request should fall back to idle");
}

bool test_cycle_idle_to_initializing_captures_measurement()
{
  JointArray measured{};
  measured[0] = 0.3f;
  measured[1] = -0.2f;

  const LocomotionCycleResult result = advance_locomotion_cycle(
      LocomotionCycleContext{
          STATE_IDLE,
          STATE_RL_RUNNING,
          0.3f,
          kDefaultInitializationStep,
          JointArray{},
          measured,
          JointArray{},
          kDefaultPolicyEntryPositions,
          false,
      });

  return expect_true(result.state == STATE_RL_INIT, "idle policy request should enter initializing") &&
      expect_true(result.enter_position_mode, "idle -> init should request position mode") &&
      expect_joint_array(result.starting_positions, measured, "starting positions should capture measured values");
}

bool test_cycle_policy_control_to_idle_requests_damping()
{
  JointArray actions{};
  actions[0] = 0.7f;
  actions[1] = -0.1f;

  const LocomotionCycleResult result = advance_locomotion_cycle(
      LocomotionCycleContext{
          STATE_RL_RUNNING,
          STATE_IDLE,
          1.0f,
          kDefaultInitializationStep,
          JointArray{},
          JointArray{},
          actions,
          kDefaultPolicyEntryPositions,
          false,
      });

  return expect_true(result.state == STATE_IDLE, "policy control should return to idle on request") &&
      expect_true(result.enter_damping_mode, "policy control -> idle should request damping") &&
      expect_joint_array(result.joint_position_target, actions, "idle transition should preserve final actions");
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
      {"pose_alignment_bias_round_trip", test_pose_alignment_bias_round_trip},
      {"pose_delta_summary_reports_worst_joint", test_pose_delta_summary_reports_worst_joint},
      {"initialization_request_selects_standing_target", test_initialization_request_selects_standing_target},
      {"policy_request_from_standing_restarts_initialization", test_policy_request_from_standing_restarts_initialization},
      {"policy_gate_blocks_large_delta", test_policy_gate_blocks_large_delta},
      {"cycle_idle_to_initializing_captures_measurement", test_cycle_idle_to_initializing_captures_measurement},
      {"cycle_policy_control_to_idle_requests_damping", test_cycle_policy_control_to_idle_requests_damping},
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

  std::puts("test-locomotion-parity passed");
  return 0;
}
