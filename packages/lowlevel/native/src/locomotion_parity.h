// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

#pragma once

#include <array>
#include <cmath>
#include <cstddef>

#include "consts.h"
#include "control_state.h"

using JointArray = std::array<float, N_JOINTS>;

enum class InitializationTarget {
  kStanding = 0,
  kPolicyEntry = 1,
};

constexpr JointArray kDefaultPolicyEntryPositions = {
    0.0f, 0.0f, -0.2f, 0.4f, -0.3f, 0.0f,
    0.0f, 0.0f, -0.2f, 0.4f, -0.3f, 0.0f,
};

constexpr JointArray kDefaultStandingPositions = {
    0.0f, 0.0f, -0.1f, 0.2f, -0.1f, 0.0f,
    0.0f, 0.0f, -0.1f, 0.2f, -0.1f, 0.0f,
};

constexpr float kPolicyEntryGateMaxAbsDeltaDeg = 10.0f;
constexpr int kPolicyEntryZeroCommandSteps = 25;
constexpr float kDefaultInitializationStep = 1.0f / 200.0f;

struct PoseDeltaSummary {
  size_t worst_index = 0;
  float max_abs_delta_deg = 0.0f;
};

inline JointArray linear_interpolate(
    const JointArray &start,
    const JointArray &end,
    float percentage) {
  const float clamped = std::fmin(std::fmax(percentage, 0.0f), 1.0f);
  JointArray output{};
  for (size_t index = 0; index < output.size(); ++index) {
    output[index] = start[index] * (1.0f - clamped) + end[index] * clamped;
  }
  return output;
}

inline JointArray apply_pose_alignment_bias(
    const JointArray &hardware_positions,
    const JointArray &pose_alignment_bias) {
  JointArray output{};
  for (size_t index = 0; index < output.size(); ++index) {
    output[index] = hardware_positions[index] + pose_alignment_bias[index];
  }
  return output;
}

inline JointArray remove_pose_alignment_bias(
    const JointArray &policy_positions,
    const JointArray &pose_alignment_bias) {
  JointArray output{};
  for (size_t index = 0; index < output.size(); ++index) {
    output[index] = policy_positions[index] - pose_alignment_bias[index];
  }
  return output;
}

inline PoseDeltaSummary compute_pose_delta_summary(
    const JointArray &target_positions,
    const JointArray &measured_positions) {
  PoseDeltaSummary summary;
  for (size_t index = 0; index < target_positions.size(); ++index) {
    const float delta_deg = std::fabs((target_positions[index] - measured_positions[index]) * 180.0f / M_PI);
    if (delta_deg > summary.max_abs_delta_deg) {
      summary.max_abs_delta_deg = delta_deg;
      summary.worst_index = index;
    }
  }
  return summary;
}

struct InitializationDecision {
  ControllerState effective_requested_state = STATE_IDLE;
  JointArray target_positions = kDefaultPolicyEntryPositions;
  InitializationTarget target = InitializationTarget::kPolicyEntry;
  bool restart_initialization = false;
  bool policy_gate_blocked = false;
  PoseDeltaSummary gate_summary{};
};

inline InitializationDecision resolve_initialization_decision(
    ControllerState state,
    ControllerState requested_state,
    float initialization_progress,
    InitializationTarget active_target,
    const JointArray &standing_positions,
    const JointArray &policy_entry_positions,
    const JointArray &measured_positions,
    float gate_limit_deg) {
  InitializationDecision decision;
  decision.effective_requested_state = requested_state;
  decision.target = active_target;
  decision.target_positions =
      active_target == InitializationTarget::kStanding ? standing_positions : policy_entry_positions;

  const bool needs_policy_gate =
      requested_state == STATE_RL_RUNNING &&
      (state == STATE_IDLE ||
       (state == STATE_RL_INIT &&
        initialization_progress >= 1.0f &&
        active_target == InitializationTarget::kStanding));

  if (needs_policy_gate) {
    decision.gate_summary = compute_pose_delta_summary(standing_positions, measured_positions);
    if (decision.gate_summary.max_abs_delta_deg > gate_limit_deg) {
      decision.policy_gate_blocked = true;
      decision.effective_requested_state = state == STATE_IDLE ? STATE_IDLE : STATE_RL_INIT;
      return decision;
    }
  }

  if (decision.effective_requested_state == STATE_RL_INIT) {
    decision.target = InitializationTarget::kStanding;
    decision.target_positions = standing_positions;
    return decision;
  }

  if (decision.effective_requested_state == STATE_RL_RUNNING) {
    decision.target = InitializationTarget::kPolicyEntry;
    decision.target_positions = policy_entry_positions;
    decision.restart_initialization =
        state == STATE_RL_INIT &&
        initialization_progress >= 1.0f &&
        active_target == InitializationTarget::kStanding;
    return decision;
  }

  return decision;
}

struct LocomotionCycleContext {
  ControllerState state = STATE_IDLE;
  ControllerState requested_state = STATE_IDLE;
  float initialization_progress = 0.0f;
  float initialization_step = kDefaultInitializationStep;
  JointArray starting_positions{};
  JointArray measured_positions{};
  JointArray policy_actions{};
  JointArray initialization_positions{};
  bool restart_initialization = false;
};

struct LocomotionCycleResult {
  ControllerState state = STATE_IDLE;
  float initialization_progress = 0.0f;
  JointArray starting_positions{};
  JointArray joint_position_target{};
  bool enter_position_mode = false;
  bool enter_damping_mode = false;
};

inline bool should_publish_policy_observations(ControllerState state) {
  return state == STATE_IDLE || state == STATE_RL_INIT || state == STATE_RL_RUNNING;
}

inline LocomotionCycleResult advance_locomotion_cycle(const LocomotionCycleContext &context) {
  if (context.restart_initialization) {
    return LocomotionCycleResult{
        STATE_RL_INIT,
        0.0f,
        context.measured_positions,
        context.measured_positions,
        false,
        false,
    };
  }

  if (context.state == STATE_IDLE) {
    if (context.requested_state == STATE_RL_INIT || context.requested_state == STATE_RL_RUNNING) {
      return LocomotionCycleResult{
          STATE_RL_INIT,
          0.0f,
          context.measured_positions,
          context.measured_positions,
          true,
          false,
      };
    }

    return LocomotionCycleResult{
        STATE_IDLE,
        context.initialization_progress,
        context.starting_positions,
        context.measured_positions,
        false,
        false,
    };
  }

  if (context.state == STATE_RL_INIT) {
    if (context.initialization_progress < 1.0f) {
      const float next_progress = std::fmin(context.initialization_progress + context.initialization_step, 1.0f);
      return LocomotionCycleResult{
          STATE_RL_INIT,
          next_progress,
          context.starting_positions,
          linear_interpolate(
              context.starting_positions,
              context.initialization_positions,
              next_progress),
          false,
          false,
      };
    }

    if (context.requested_state == STATE_RL_RUNNING) {
      return LocomotionCycleResult{
          STATE_RL_RUNNING,
          context.initialization_progress,
          context.starting_positions,
          context.initialization_positions,
          false,
          false,
      };
    }

    if (context.requested_state == STATE_IDLE) {
      return LocomotionCycleResult{
          STATE_IDLE,
          context.initialization_progress,
          context.starting_positions,
          context.initialization_positions,
          false,
          true,
      };
    }

    return LocomotionCycleResult{
        STATE_RL_INIT,
        context.initialization_progress,
        context.starting_positions,
        context.initialization_positions,
        false,
        false,
    };
  }

  if (context.state == STATE_RL_RUNNING) {
    if (context.requested_state == STATE_IDLE) {
      return LocomotionCycleResult{
          STATE_IDLE,
          context.initialization_progress,
          context.starting_positions,
          context.policy_actions,
          false,
          true,
      };
    }

    return LocomotionCycleResult{
        STATE_RL_RUNNING,
        context.initialization_progress,
        context.starting_positions,
        context.policy_actions,
        false,
        false,
    };
  }

  return LocomotionCycleResult{
      STATE_IDLE,
      context.initialization_progress,
      context.starting_positions,
      context.measured_positions,
      false,
      false,
  };
}
