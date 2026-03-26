from __future__ import annotations

import time

import numpy as np

from .calibration import CalibrationStore
from .command_source import GamepadCommandSource, LocomotionCommand
from .control_state import LocomotionControlState
from .imu import SerialImu
from .joint_transport import JointInterface, LocomotionActuatorArray
from .locomotion_specification import (
    LocomotionRobotSpecification,
    build_default_locomotion_robot_specification,
)


def linear_interpolate(start: np.ndarray, end: np.ndarray, percentage: float) -> np.ndarray:
    percentage = min(max(percentage, 0.0), 1.0)
    return start * (1.0 - percentage) + end * percentage


class LocomotionRobot:
    """组合传感器、命令源和执行器通信的 locomotion 运行时。"""

    def __init__(
        self,
        specification: LocomotionRobotSpecification | None = None,
        calibration_store: CalibrationStore | None = None,
        *,
        enable_imu: bool = True,
        enable_command_source: bool = True,
    ) -> None:
        self.specification = specification or build_default_locomotion_robot_specification()
        self.calibration_store = calibration_store or CalibrationStore()

        position_offsets = self.calibration_store.load_position_offsets(
            self.specification.joint_count)
        self.actuators = LocomotionActuatorArray(
            self.specification,
            position_offsets=position_offsets,
        )

        self.imu = SerialImu(
            baudrate=self.specification.imu_baudrate) if enable_imu else None
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
        self.starting_positions = np.zeros(
            (self.specification.joint_count,), dtype=np.float32)
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

    def enter_damping_mode(self) -> None:
        self.actuators.configure_damping_mode()

    def shutdown(self) -> None:
        if self.imu is not None:
            self.imu.stop()
        if self.command_source is not None:
            self.command_source.stop()

        self.actuators.set_idle_mode()
        self.actuators.shutdown()

    def stop(self) -> None:
        if self.imu is not None:
            self.imu.stop()
        if self.command_source is not None:
            self.command_source.stop()

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
            imu_quaternion[:] = self.imu.quaternion[:]
            print(f"IMU raw quaternion: {self.imu.quaternion[:]}")
            imu_angular_velocity[:] = np.deg2rad(self.imu.angular_velocity[:])
            print(
                f"IMU quaternion: {imu_quaternion}, angular velocity (rad/s): {imu_angular_velocity}")
        else:
            imu_quaternion[:] = np.array(
                [1.0, 0.0, 0.0, 0.0], dtype=np.float32)
            imu_angular_velocity[:] = 0.0

        joint_positions[:] = self.actuators.joint_position_measured[:]
        joint_velocities[:] = self.actuators.joint_velocity_measured[:]

        command = self._get_command()
        mode[0] = int(command.requested_state)
        velocity_commands[0] = command.velocity_x
        velocity_commands[1] = command.velocity_y
        velocity_commands[2] = command.velocity_yaw
        self.requested_state = command.requested_state

        return self.lowlevel_states

    def reset(self) -> np.ndarray:
        return self.get_observations()

    def step(self, actions: np.ndarray) -> np.ndarray:
        match self.state:
            case LocomotionControlState.IDLE:
                self.actuators.joint_position_target[:
                                                     ] = self.actuators.joint_position_measured[:]

                if self.requested_state == LocomotionControlState.INITIALIZING:
                    print("Switching to initialization mode")
                    self.state = self.requested_state
                    self.actuators.set_position_mode()
                    self.starting_positions[:] = self.actuators.joint_position_target[:]
                    self.initialization_progress = 0.0

            case LocomotionControlState.INITIALIZING:
                print(f"init: {self.initialization_progress:.2f}")
                if self.initialization_progress < 1.0:
                    print(
                        f"Initializing... {self.initialization_progress:.2f}")
                    self.initialization_progress = min(
                        self.initialization_progress + 0.01, 1.0)
                    self.actuators.joint_position_target[:] = linear_interpolate(
                        self.starting_positions,
                        self.specification.initialization_positions,
                        self.initialization_progress,
                    )
                elif self.requested_state == LocomotionControlState.POLICY_CONTROL:
                    print("Switching to policy control mode")
                    self.state = self.requested_state
                elif self.requested_state == LocomotionControlState.IDLE:
                    print("Switching to idle mode")
                    self.state = self.requested_state
                    self.actuators.set_damping_mode()

            case LocomotionControlState.POLICY_CONTROL:
                self.actuators.joint_position_target[:] = actions

                if self.requested_state == LocomotionControlState.IDLE:
                    print("Switching to idle mode")
                    self.state = self.requested_state
                    self.actuators.set_damping_mode()

            case _:
                self.state = LocomotionControlState.IDLE

        self.actuators.synchronize()
        return self.get_observations()

    def read_joint_positions(self) -> np.ndarray:
        return self.actuators.read_positions()

    def check_connection(self) -> None:
        self.actuators.check_connection()
