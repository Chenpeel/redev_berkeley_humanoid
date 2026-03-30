// Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

#pragma once


#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <stdio.h>
#include <yaml-cpp/yaml.h>

#include "consts.h"
#include "control_state.h"
#include "loop_function.h"
#include "locomotion_parity.h"
#include "locomotion_specification.h"
#include "motor_controller.h"
#include "imu.h"
#include "udp.h"
#include "socketcan.h"




#define N_LOWLEVEL_STATES       (4+3+12+12+1+3)
#define N_LOWLEVEL_COMMANDS     12

static inline float deg2rad(float deg) {
    return deg * M_PI / 180.0f;
}


/**
 * Joint order:
 *  0: left_hip_roll
 *  1: left_hip_yaw
 *  2: left_hip_pitch
 *  3: left_knee_pitch
 *  4: left_ankle_pitch
 *  5: left_ankle_roll
 *  6: right_hip_roll
 *  7: right_hip_yaw
 *  8: right_hip_pitch
 *  9: right_knee_pitch
 *  10: right_ankle_pitch
 *  11: right_ankle_roll
 */
enum JointOrder {
  LEG_LEFT_HIP_ROLL_JOINT         = 0,
  LEG_LEFT_HIP_YAW_JOINT          = 1,
  LEG_LEFT_HIP_PITCH_JOINT        = 2,
  LEG_LEFT_KNEE_PITCH_JOINT       = 3,
  LEG_LEFT_ANKLE_PITCH_JOINT      = 4,
  LEG_LEFT_ANKLE_ROLL_JOINT       = 5,
  LEG_RIGHT_HIP_ROLL_JOINT        = 6,
  LEG_RIGHT_HIP_YAW_JOINT         = 7,
  LEG_RIGHT_HIP_PITCH_JOINT       = 8,
  LEG_RIGHT_KNEE_PITCH_JOINT      = 9,
  LEG_RIGHT_ANKLE_PITCH_JOINT     = 10,
  LEG_RIGHT_ANKLE_ROLL_JOINT      = 11,
};
// enum JointOrder {
//   ARM_LEFT_SHOULDER_PITCH_JOINT   = 0,
//   ARM_LEFT_SHOULDER_ROLL_JOINT    = 1,
//   ARM_LEFT_SHOULDER_YAW_JOINT     = 2,
//   ARM_LEFT_ELBOW_PITCH_JOINT      = 3,
//   ARM_LEFT_ELBOW_ROLL_JOINT       = 4,
//   ARM_RIGHT_SHOULDER_PITCH_JOINT  = 5,
//   ARM_RIGHT_SHOULDER_ROLL_JOINT   = 6,
//   ARM_RIGHT_SHOULDER_YAW_JOINT    = 7,
//   ARM_RIGHT_ELBOW_PITCH_JOINT     = 8,
//   ARM_RIGHT_ELBOW_ROLL_JOINT      = 9,
//   LEG_LEFT_HIP_ROLL_JOINT         = 10,
//   LEG_LEFT_HIP_YAW_JOINT          = 11,
//   LEG_LEFT_HIP_PITCH_JOINT        = 12,
//   LEG_LEFT_KNEE_PITCH_JOINT       = 13,
//   LEG_LEFT_ANKLE_PITCH_JOINT      = 14,
//   LEG_LEFT_ANKLE_ROLL_JOINT       = 15,
//   LEG_RIGHT_HIP_ROLL_JOINT        = 16,
//   LEG_RIGHT_HIP_YAW_JOINT         = 17,
//   LEG_RIGHT_HIP_PITCH_JOINT       = 18,
//   LEG_RIGHT_KNEE_PITCH_JOINT      = 19,
//   LEG_RIGHT_ANKLE_PITCH_JOINT     = 20,
//   LEG_RIGHT_ANKLE_ROLL_JOINT      = 21,
// };


class RealHumanoid {
  public:
    float position_target[N_JOINTS];
    float position_measured[N_JOINTS];
    float hardware_position_measured[N_JOINTS];
    float velocity_measured[N_JOINTS];

    float starting_positions[N_JOINTS] = {0};

    float position_offsets[N_JOINTS] = {0};

    float joint_kp[N_JOINTS] = {0};
    float joint_kd[N_JOINTS] = {0};
    float torque_limit[N_JOINTS] = {0};
    JointArray active_initialization_positions_ = kDefaultPolicyEntryPositions;
    JointArray pose_alignment_bias_{};
    InitializationTarget active_initialization_target_ = InitializationTarget::kPolicyEntry;
    JointArray policy_entry_positions_ = kDefaultPolicyEntryPositions;

    RealHumanoid(
        const IMUConfiguration &imu_configuration,
        std::string left_leg_bus_name = DEFAULT_LEFT_LEG_BUS,
        std::string right_leg_bus_name = DEFAULT_RIGHT_LEG_BUS,
        LocomotionSpecificationSource specification_source = LocomotionSpecificationSource::LegacyNative,
        std::string hardware_configuration_path = DEFAULT_HARDWARE_CONFIGURATION_PATH);
    ~RealHumanoid();

    /**
     * Stop the humanoid low-level control.
     *
     * This method will join all the threads. On first execution, it will set motor to damping
     * mode; on second execution, it will set motor to idle mode and exits the main loop.
     */
    void stop();

    /**
     * Start the humanoid low-level control.
     *
     * This method will initialize the low-level components of the robot, create the udp
     * communication threads, and start the control loop.
     */
    void run();

    void run_calibration();

  private:
    /* Low-level components */
    uint8_t stopped = 0;
    ControllerState state = STATE_IDLE;
    ControllerState next_state = STATE_IDLE;
    bool calibration_file_found_ = false;
    bool pose_alignment_file_found_ = false;
    bool calibration_audit_printed_ = false;
    bool policy_request_blocked_ = false;

    float config_control_dt_ = 0.;
    float config_policy_dt_ = 0.;
    float policy_entry_gate_max_abs_delta_deg_ = kPolicyEntryGateMaxAbsDeltaDeg;
    int policy_entry_zero_command_steps_ = kPolicyEntryZeroCommandSteps;
    int policy_entry_zero_command_steps_remaining_ = 0;

    float stick_command_velocity_x_ = 0.0;
    float stick_command_velocity_y_ = 0.0;
    float stick_command_velocity_yaw_ = 0.0;

    std::string config_udp_host_addr_;

    /* devices */
    IMUConfiguration imu_configuration_;
    std::string left_leg_bus_name_;
    std::string right_leg_bus_name_;
    LocomotionSpecificationSource specification_source_ = LocomotionSpecificationSource::LegacyNative;
    std::string hardware_configuration_path_ = DEFAULT_HARDWARE_CONFIGURATION_PATH;
    LocomotionRobotSpecification specification_;
    IMU *imu;
    SocketCan left_arm_bus;
    SocketCan right_arm_bus;
    SocketCan left_leg_bus;
    SocketCan right_leg_bus;

    std::array<std::shared_ptr<MotorController>, N_JOINTS> joint_ptrs;


    uint8_t control_loop_count = 0;

    float init_percentage = 0.0;

    /* Threading */
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_udp_recv;
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_joystick;
    std::shared_ptr<LoopFunc> loop_imu;

    /* UDP stuff */
    UDP udp;
    float lowlevel_commands[N_LOWLEVEL_COMMANDS];
    float lowlevel_states[N_LOWLEVEL_STATES];

    UDP udp_joystick;

    std::mutex control_input_mutex_;

    /* Policy stuff */
    // torch::Tensor policy_observations;
    // torch::Tensor policy_actions;


    void initialize();

    /**
     * The control loop that communicates with the hardware.
     *
     * This loop will run at 250 Hz.
     *
     * On each loop iteration, it will perform the following:
     *  1. collects the action terms from action array set by either UDP communication or the policy.
     *  2. triggers an IMU update.
     *  3. receives the data from IMU.
     *  4. sends the target positions to joints and reads the meaured positions, with some delay between each joint.
     *  5. populates the observation array.
     */
    void control_loop();

    void keyboard_loop();

    void joystick_loop();
    
    void imu_loop();

    void process_actions();

    void process_observations();

    void policy_forward();

    void udp_recv();

    void update_imu();

    void update_joints();

    void print_calibration_audit() const;
};
