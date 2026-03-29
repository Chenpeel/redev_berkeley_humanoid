from __future__ import annotations

import time
from pathlib import Path

import numpy as np

from .calibration import CalibrationStore
from .command_source import GamepadCommandSource, LocomotionCommand
from .control_state import LocomotionControlState
from .imu import SerialImu
from .joint_transport import JointInterface, LocomotionActuatorArray
from .locomotion_cycle import LocomotionCycleContext, advance_locomotion_cycle
from .locomotion_diagnostics import (
    LocomotionDiagnosticSnapshot,
    build_locomotion_diagnostic_snapshot,
    format_imu_debug_line,
)
from .locomotion_specification import (
    LocomotionRobotSpecification,
    build_default_locomotion_robot_specification,
)
from .pose_alignment import (
    PoseAlignmentStore,
    apply_pose_alignment_bias,
    remove_pose_alignment_bias,
)


class LocomotionRobot:
    """组合传感器、命令源和执行器通信的 locomotion 运行时。"""

    _POLICY_ENTRY_GATE_MAX_ABS_DELTA_DEG = 10.0
    _POLICY_ENTRY_ZERO_COMMAND_STEPS = 25

    def __init__(
        self,
        specification: LocomotionRobotSpecification | None = None,
        calibration_store: CalibrationStore | None = None,
        pose_alignment_store: PoseAlignmentStore | None = None,
        *,
        enable_imu: bool = True,
        enable_command_source: bool = True,
        dry_run: bool = False,
        imu_device: str = "/dev/ttyUSB0",
        imu_baudrate: int | None = None,
        imu_read_timeout: float = 0.01,
        imu_wait_timeout: float = 2.0,
        require_imu_ready: bool = True,
        policy_entry_zero_command_steps: int = _POLICY_ENTRY_ZERO_COMMAND_STEPS,
    ) -> None:
        self.specification = specification or build_default_locomotion_robot_specification()
        self.calibration_store = calibration_store or CalibrationStore()
        self.pose_alignment_store = pose_alignment_store or PoseAlignmentStore()
        self.dry_run = dry_run
        self.imu_wait_timeout = float(imu_wait_timeout)
        self.require_imu_ready = require_imu_ready
        self.policy_entry_zero_command_steps = max(0, int(policy_entry_zero_command_steps))

        position_offsets = self.calibration_store.load_position_offsets(
            self.specification.joint_count)
        self.actuators = LocomotionActuatorArray(
            self.specification,
            position_offsets=position_offsets,
        )
        self.pose_alignment_bias = self.pose_alignment_store.load_pose_alignment_bias(
            self.specification.joint_count
        )

        effective_imu_baudrate = self.specification.imu_baudrate if imu_baudrate is None else int(imu_baudrate)
        self.imu = SerialImu(
            port=imu_device,
            baudrate=effective_imu_baudrate,
            read_timeout=imu_read_timeout,
        ) if enable_imu else None
        if self.imu is not None:
            self.imu.run_forever()

        self.command_source = GamepadCommandSource() if enable_command_source else None
        try:
            if self.command_source is not None:
                self.command_source.start()
        except Exception:
            self.shutdown()
            raise

        self.state = LocomotionControlState.IDLE
        self.requested_state = LocomotionControlState.INVALID
        self.initialization_progress = 0.0
        self.initialization_step = 0.01
        self.starting_positions = np.zeros(
            (self.specification.joint_count,), dtype=np.float32)
        self.active_initialization_positions = self.specification.initialization_positions.copy()
        self.active_initialization_label = "policy_entry"
        self.policy_joint_position_target = np.zeros(
            (self.specification.joint_count,), dtype=np.float32)
        self._policy_request_blocked = False
        self._policy_entry_zero_command_steps_remaining = 0
        self._raw_requested_state = LocomotionControlState.INVALID
        self.lowlevel_states = np.zeros(
            (self.specification.observation_size,), dtype=np.float32)

    @property
    def joint_interfaces(self) -> tuple[JointInterface, ...]:
        return self.actuators.joint_interfaces

    @property
    def joint_axis_directions(self) -> np.ndarray:
        return self.actuators.joint_axis_directions

    @property
    def position_offsets(self) -> np.ndarray:
        return self.actuators.position_offsets

    @property
    def joint_position_measured(self) -> np.ndarray:
        return self.actuators.joint_position_measured

    @property
    def policy_joint_position_measured(self) -> np.ndarray:
        return apply_pose_alignment_bias(
            self.actuators.joint_position_measured,
            self.pose_alignment_bias,
        )

    def enter_damping_mode(self) -> None:
        if self.dry_run:
            return
        self.actuators.configure_damping_mode()

    def shutdown(self) -> None:
        if self.imu is not None:
            self.imu.stop()
        if self.command_source is not None:
            self.command_source.stop()

        if not self.dry_run:
            self.actuators.set_idle_mode()
        self.actuators.shutdown()

    def stop(self) -> None:
        if self.imu is not None:
            self.imu.stop()
        if self.command_source is not None:
            self.command_source.stop()

        if self.dry_run:
            self.shutdown()
            return

        self.actuators.set_damping_mode()
        print("Entered damping mode. Press Ctrl+C again to exit.\n")

        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Exiting damping mode.")

        self.shutdown()

    def _get_command(self) -> LocomotionCommand:
        if self.command_source is None:
            return LocomotionCommand.zero()
        return self.command_source.snapshot()

    def _update_requested_state(self, requested_state: LocomotionControlState) -> None:
        if requested_state != LocomotionControlState.INVALID:
            self.requested_state = requested_state

    def _raw_joint_positions_from(self, joint_positions: np.ndarray) -> np.ndarray:
        hardware_positions = self._hardware_joint_positions_from(joint_positions)
        return (hardware_positions + self.position_offsets) * self.joint_axis_directions

    def _policy_joint_positions_from(self, hardware_joint_positions: np.ndarray) -> np.ndarray:
        return apply_pose_alignment_bias(hardware_joint_positions, self.pose_alignment_bias)

    def _hardware_joint_positions_from(self, policy_joint_positions: np.ndarray) -> np.ndarray:
        return remove_pose_alignment_bias(policy_joint_positions, self.pose_alignment_bias)

    def _compute_pose_delta(
        self,
        target_positions: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, str, float]:
        target = np.asarray(target_positions, dtype=np.float32)
        measured = self.policy_joint_position_measured
        delta = target - measured
        raw_delta = self._raw_joint_positions_from(target) - self._raw_joint_positions_from(measured)
        if self.specification.joint_count == 0:
            return delta, raw_delta, "", 0.0

        worst_index = int(np.argmax(np.abs(delta)))
        worst_joint_name = self.specification.joint_names[worst_index]
        max_abs_delta_deg = float(np.max(np.abs(np.rad2deg(delta))))
        return delta, raw_delta, worst_joint_name, max_abs_delta_deg

    def _calibration_file_state(self) -> str:
        calibration_path = getattr(self.calibration_store, "calibration_path", None)
        if calibration_path is None:
            return "unknown"
        return "found" if Path(calibration_path).exists() else "missing"

    def _pose_alignment_file_state(self) -> str:
        pose_alignment_path = getattr(self.pose_alignment_store, "pose_alignment_path", None)
        if pose_alignment_path is None:
            return "unknown"
        return "found" if Path(pose_alignment_path).exists() else "missing"

    def _print_calibration_audit(self) -> None:
        _, _, standing_joint_name, standing_max_abs_delta_deg = self._compute_pose_delta(
            self.specification.standing_positions,
        )
        _, _, initialization_joint_name, initialization_max_abs_delta_deg = self._compute_pose_delta(
            self.specification.initialization_positions,
        )
        max_abs_bias_deg = (
            float(np.max(np.abs(np.rad2deg(self.pose_alignment_bias))))
            if self.pose_alignment_bias.size
            else 0.0
        )
        print("Calibration audit:")
        print(f"  file: {self._calibration_file_state()}")
        print(
            "  pose alignment:",
            f"file={self._pose_alignment_file_state()}",
            f"max_abs_bias_deg={max_abs_bias_deg:.2f}",
        )
        print(
            "  standing pose:",
            f"max_abs_delta_deg={standing_max_abs_delta_deg:.2f}",
            f"worst_joint={standing_joint_name}",
        )
        print(
            "  initialization pose:",
            f"max_abs_delta_deg={initialization_max_abs_delta_deg:.2f}",
            f"worst_joint={initialization_joint_name}",
        )

    def _needs_policy_entry_gate(self) -> bool:
        if self.requested_state != LocomotionControlState.POLICY_CONTROL:
            return False
        if self.state == LocomotionControlState.IDLE:
            return True
        return (
            self.state == LocomotionControlState.INITIALIZING
            and self.initialization_progress >= 1.0
            and self.active_initialization_label == "standing"
        )

    def _policy_entry_gate_passes(self) -> bool:
        _, _, standing_joint_name, standing_max_abs_delta_deg = self._compute_pose_delta(
            self.specification.standing_positions,
        )
        if standing_max_abs_delta_deg <= self._POLICY_ENTRY_GATE_MAX_ABS_DELTA_DEG:
            self._policy_request_blocked = False
            return True

        if not self._policy_request_blocked:
            print(
                "Policy gate blocked:",
                f"standing pose max_abs_delta_deg={standing_max_abs_delta_deg:.2f}",
                f"worst_joint={standing_joint_name}",
                f"limit={self._POLICY_ENTRY_GATE_MAX_ABS_DELTA_DEG:.2f}",
            )
            print("Press A+LB to enter the ready/stand pose or recalibrate before requesting policy control.")
        self._policy_request_blocked = True
        self.requested_state = LocomotionControlState.INVALID
        return False

    def _resolve_initialization_cycle_inputs(
        self,
    ) -> tuple[LocomotionControlState, np.ndarray, bool]:
        requested_state = self.requested_state
        restart_initialization = False

        if self._needs_policy_entry_gate() and not self._policy_entry_gate_passes():
            requested_state = LocomotionControlState.INVALID

        if requested_state == LocomotionControlState.INITIALIZING:
            target_positions = np.asarray(self.specification.standing_positions, dtype=np.float32)
            target_label = "standing"
        elif requested_state == LocomotionControlState.POLICY_CONTROL:
            target_positions = np.asarray(self.specification.initialization_positions, dtype=np.float32)
            target_label = "policy_entry"
            restart_initialization = (
                self.state == LocomotionControlState.INITIALIZING
                and self.initialization_progress >= 1.0
                and self.active_initialization_label == "standing"
            )
        else:
            target_positions = np.asarray(self.active_initialization_positions, dtype=np.float32)
            target_label = self.active_initialization_label

        self.active_initialization_positions[:] = target_positions
        self.active_initialization_label = target_label
        return requested_state, target_positions.copy(), restart_initialization

    def _wait_for_imu_ready(self) -> None:
        if self.imu is None or not self.require_imu_ready:
            return
        if self.imu.wait_until_ready(timeout=self.imu_wait_timeout):
            return

        snapshot = self.imu.snapshot()
        raise RuntimeError(
            "IMU did not become ready before locomotion reset. "
            "Expected both quaternion and angular velocity frames from the HiWonder IMU. "
            f"timeout={self.imu_wait_timeout:g}s "
            f"quaternion_ready={snapshot.quaternion_ready} "
            f"angular_velocity_ready={snapshot.angular_velocity_ready} "
            f"last_timestamp={snapshot.timestamp:.6f}"
        )

    def _arm_policy_entry_zero_command_window(self) -> None:
        if self.policy_entry_zero_command_steps <= 0:
            return
        self._policy_entry_zero_command_steps_remaining = self.policy_entry_zero_command_steps

    def get_observations(self) -> np.ndarray:
        imu_quaternion = self.lowlevel_states[0:4]
        imu_angular_velocity = self.lowlevel_states[4:7]
        joint_positions = self.lowlevel_states[7:7 +
                                               self.specification.joint_count]
        joint_velocities = self.lowlevel_states[
            7 + self.specification.joint_count:7 + self.specification.joint_count * 2
        ]
        mode = self.lowlevel_states[7 + self.specification.joint_count *
                                    2:7 + self.specification.joint_count * 2 + 1]
        velocity_commands = self.lowlevel_states[7 +
                                                 self.specification.joint_count * 2 + 1:]

        if self.imu is not None:
            imu_snapshot = self.imu.snapshot()
            imu_quaternion[:] = imu_snapshot.quaternion_wxyz
            imu_angular_velocity[:] = np.deg2rad(imu_snapshot.angular_velocity_deg_s)
        else:
            imu_quaternion[:] = np.array(
                [1.0, 0.0, 0.0, 0.0], dtype=np.float32)
            imu_angular_velocity[:] = 0.0

        joint_positions[:] = self.policy_joint_position_measured[:]
        joint_velocities[:] = self.actuators.joint_velocity_measured[:]

        command = self._get_command()
        self._raw_requested_state = command.requested_state
        if self._raw_requested_state != LocomotionControlState.POLICY_CONTROL:
            self._policy_request_blocked = False
        self._update_requested_state(command.requested_state)
        mode[0] = int(self.requested_state)
        if self._policy_entry_zero_command_steps_remaining > 0:
            velocity_commands[:] = 0.0
            self._policy_entry_zero_command_steps_remaining -= 1
        else:
            velocity_commands[0] = command.velocity_x
            velocity_commands[1] = command.velocity_y
            velocity_commands[2] = command.velocity_yaw

        return self.lowlevel_states

    def reset(self) -> np.ndarray:
        self._wait_for_imu_ready()
        self.actuators.refresh_measurements()
        self._print_calibration_audit()
        return self.get_observations()

    def step(self, actions: np.ndarray) -> np.ndarray:
        if self.dry_run or not self.actuators.measurements_ready:
            self.actuators.refresh_measurements()

        previous_state = self.state
        measured_policy_positions = self.policy_joint_position_measured
        requested_state, initialization_positions, restart_initialization = (
            self._resolve_initialization_cycle_inputs()
        )

        result = advance_locomotion_cycle(
            LocomotionCycleContext(
                state=self.state,
                requested_state=requested_state,
                initialization_progress=self.initialization_progress,
                initialization_step=self.initialization_step,
                starting_positions=self.starting_positions,
                measured_positions=measured_policy_positions,
                policy_actions=actions,
                initialization_positions=initialization_positions,
                restart_initialization=restart_initialization,
            )
        )

        for message in result.messages:
            print(message)

        self.state = result.state
        self.initialization_progress = result.initialization_progress
        self.starting_positions[:] = result.starting_positions
        self.policy_joint_position_target[:] = result.joint_position_target
        self.actuators.joint_position_target[:] = self._hardware_joint_positions_from(
            result.joint_position_target
        )
        if (
            previous_state != LocomotionControlState.POLICY_CONTROL
            and result.state == LocomotionControlState.POLICY_CONTROL
        ):
            self._arm_policy_entry_zero_command_window()

        if not self.dry_run:
            if result.enter_position_mode:
                self.actuators.set_position_mode()
            if result.enter_damping_mode:
                self.actuators.set_damping_mode()
            self.actuators.synchronize()

        return self.get_observations()

    def create_diagnostic_snapshot(
        self,
        observations: np.ndarray,
        actions: np.ndarray,
    ) -> LocomotionDiagnosticSnapshot:
        joint_count = int(self.specification.joint_count)
        command_start = 7 + joint_count * 2 + 1
        command_velocity = observations[command_start: command_start + 3]
        return build_locomotion_diagnostic_snapshot(
            specification=self.specification,
            state=self.state,
            requested_state=self.requested_state,
            command_velocity=command_velocity,
            actions=actions,
            joint_position_target=self.policy_joint_position_target,
            joint_position_measured=self.policy_joint_position_measured,
            position_offsets=self.position_offsets,
            joint_axis_directions=self.joint_axis_directions,
            dry_run=self.dry_run,
            pose_alignment_bias=self.pose_alignment_bias,
        )

    def create_imu_debug_line(self) -> str | None:
        if self.imu is None:
            return None
        imu_snapshot = self.imu.snapshot()
        return format_imu_debug_line(
            quaternion_wxyz=imu_snapshot.quaternion_wxyz,
            angular_velocity_deg_s=imu_snapshot.angular_velocity_deg_s,
        )

    def read_joint_positions(self) -> np.ndarray:
        return self.actuators.read_positions()

    def check_connection(self) -> None:
        self.actuators.check_connection()
