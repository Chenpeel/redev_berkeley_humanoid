from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

import berkeley_humanoid_lite_lowlevel.recoil as recoil

from .locomotion_specification import LocomotionRobotSpecification


@dataclass(frozen=True)
class JointInterface:
    bus: recoil.Bus
    device_id: int
    joint_name: str


class LocomotionActuatorArray:
    """管理 locomotion 机器人的执行器通信和坐标变换。"""

    def __init__(
        self,
        specification: LocomotionRobotSpecification,
        position_offsets: np.ndarray | None = None,
    ) -> None:
        self.specification = specification
        self._transports = {
            address.bus_name: recoil.Bus(address.bus_name)
            for address in self.specification.joint_addresses
        }
        self.joint_interfaces = tuple(
            JointInterface(
                bus=self._transports[address.bus_name],
                device_id=address.device_id,
                joint_name=address.joint_name,
            )
            for address in self.specification.joint_addresses
        )

        self.joint_axis_directions = self.specification.joint_axis_directions.copy()
        self.position_offsets = np.zeros((self.specification.joint_count,), dtype=np.float32)
        if position_offsets is not None:
            validated_offsets = np.asarray(position_offsets, dtype=np.float32)
            if validated_offsets.shape != (self.specification.joint_count,):
                raise ValueError("position_offsets 的长度必须与关节数一致。")
            self.position_offsets[:] = validated_offsets

        self.joint_position_target = np.zeros((self.specification.joint_count,), dtype=np.float32)
        self.joint_position_measured = np.zeros((self.specification.joint_count,), dtype=np.float32)
        self.joint_velocity_measured = np.zeros((self.specification.joint_count,), dtype=np.float32)

        self.joint_kp = np.zeros((self.specification.joint_count,), dtype=np.float32)
        self.joint_kd = np.zeros((self.specification.joint_count,), dtype=np.float32)
        self.torque_limit = np.zeros((self.specification.joint_count,), dtype=np.float32)

    def configure_damping_mode(
        self,
        position_kp: float = 20.0,
        position_kd: float = 2.0,
        torque_limit: float = 4.0,
    ) -> None:
        self.joint_kp[:] = position_kp
        self.joint_kd[:] = position_kd
        self.torque_limit[:] = torque_limit

        for index, joint in enumerate(self.joint_interfaces):
            print(f"Initializing joint {joint.joint_name}:")
            print(
                "  "
                f"kp: {self.joint_kp[index]}, "
                f"kd: {self.joint_kd[index]}, "
                f"torque limit: {self.torque_limit[index]}"
            )

            joint.bus.set_mode(joint.device_id, recoil.Mode.IDLE)
            time.sleep(0.001)
            joint.bus.write_position_kp(joint.device_id, self.joint_kp[index])
            time.sleep(0.001)
            joint.bus.write_position_kd(joint.device_id, self.joint_kd[index])
            time.sleep(0.001)
            joint.bus.write_torque_limit(joint.device_id, self.torque_limit[index])
            time.sleep(0.001)
            joint.bus.feed(joint.device_id)
            joint.bus.set_mode(joint.device_id, recoil.Mode.DAMPING)

        print("Motors enabled")

    def set_position_mode(self) -> None:
        for joint in self.joint_interfaces:
            joint.bus.feed(joint.device_id)
            joint.bus.set_mode(joint.device_id, recoil.Mode.POSITION)

    def set_damping_mode(self) -> None:
        for joint in self.joint_interfaces:
            joint.bus.set_mode(joint.device_id, recoil.Mode.DAMPING)

    def set_idle_mode(self) -> None:
        for joint in self.joint_interfaces:
            joint.bus.set_mode(joint.device_id, recoil.Mode.IDLE)

    def read_positions(self) -> np.ndarray:
        positions = np.zeros((self.specification.joint_count,), dtype=np.float32)
        for index, joint in enumerate(self.joint_interfaces):
            position_measured = joint.bus.read_position_measured(joint.device_id)
            positions[index] = (
                position_measured * self.joint_axis_directions[index]
            ) - self.position_offsets[index]
        return positions

    def _update_joint_pair(self, left_index: int, right_index: int) -> None:
        position_target_left = (
            self.joint_position_target[left_index] + self.position_offsets[left_index]
        ) * self.joint_axis_directions[left_index]
        position_target_right = (
            self.joint_position_target[right_index] + self.position_offsets[right_index]
        ) * self.joint_axis_directions[right_index]

        left_joint = self.joint_interfaces[left_index]
        right_joint = self.joint_interfaces[right_index]

        left_joint.bus.transmit_pdo_2(
            left_joint.device_id,
            position_target=position_target_left,
            velocity_target=0.0,
        )
        right_joint.bus.transmit_pdo_2(
            right_joint.device_id,
            position_target=position_target_right,
            velocity_target=0.0,
        )

        left_position_measured, left_velocity_measured = left_joint.bus.receive_pdo_2(left_joint.device_id)
        right_position_measured, right_velocity_measured = right_joint.bus.receive_pdo_2(right_joint.device_id)

        # 执行器和策略坐标系不同，这里统一做方向与零位变换。
        if left_position_measured is not None:
            self.joint_position_measured[left_index] = (
                left_position_measured * self.joint_axis_directions[left_index]
            ) - self.position_offsets[left_index]
        if left_velocity_measured is not None:
            self.joint_velocity_measured[left_index] = (
                left_velocity_measured * self.joint_axis_directions[left_index]
            )
        if right_position_measured is not None:
            self.joint_position_measured[right_index] = (
                right_position_measured * self.joint_axis_directions[right_index]
            ) - self.position_offsets[right_index]
        if right_velocity_measured is not None:
            self.joint_velocity_measured[right_index] = (
                right_velocity_measured * self.joint_axis_directions[right_index]
            )

    def synchronize(self) -> None:
        for left_index, right_index in self.specification.mirrored_joint_pairs:
            self._update_joint_pair(left_index, right_index)

    def check_connection(self) -> None:
        for joint in self.joint_interfaces:
            print(f"Pinging {joint.joint_name} ... ", end="\t")
            result = joint.bus.ping(joint.device_id)
            print("OK" if result else "ERROR")
            time.sleep(0.1)

    def shutdown(self) -> None:
        for bus in self._transports.values():
            bus.stop()
