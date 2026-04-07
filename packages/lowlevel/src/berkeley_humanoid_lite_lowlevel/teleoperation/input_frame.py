from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np

SUPPORTED_FAKE_TELEOP_PATTERNS = (
    "static_pose", "single_arm_sweep", "dual_arm_circle")
FakeTeleoperationPattern = Literal["static_pose",
                                   "single_arm_sweep", "dual_arm_circle"]
SweepArm = Literal["auto", "left", "right"]

_LEFT_REST_TRANSLATION = np.array([0.2, 0.2, 0.6], dtype=np.float64)
_RIGHT_REST_TRANSLATION = np.array([0.2, -0.2, 0.6], dtype=np.float64)


@dataclass(slots=True)
class TeleopHandInput:
    pose: np.ndarray
    button_pressed: bool
    trigger: float

    def __post_init__(self) -> None:
        pose = np.asarray(self.pose, dtype=np.float64)
        if pose.shape != (4, 4):
            raise ValueError(
                "Teleoperation pose must be a 4x4 homogeneous transform.")

        self.pose = pose.copy()
        self.button_pressed = bool(self.button_pressed)
        self.trigger = float(np.clip(self.trigger, 0.0, 1.0))

    def to_bridge_data(self) -> dict[str, object]:
        return {
            "pose": self.pose.tolist(),
            "button_pressed": self.button_pressed,
            "trigger": self.trigger,
        }


@dataclass(slots=True)
class TeleopInputFrame:
    left: TeleopHandInput
    right: TeleopHandInput

    def to_bridge_data(self) -> dict[str, dict[str, object]]:
        return {
            "left": self.left.to_bridge_data(),
            "right": self.right.to_bridge_data(),
        }


def _build_pose(translation: np.ndarray) -> np.ndarray:
    pose = np.eye(4, dtype=np.float64)
    pose[0:3, 3] = np.asarray(translation, dtype=np.float64)
    return pose


def _resolve_sweep_arm(
    *,
    sweep_arm: SweepArm,
    left_enable: bool,
    right_enable: bool,
) -> Literal["left", "right"]:
    if sweep_arm in {"left", "right"}:
        return sweep_arm
    if left_enable and not right_enable:
        return "left"
    if right_enable and not left_enable:
        return "right"
    return "left"


def _build_gripper_targets(timestamp: float, *, with_gripper: bool) -> tuple[float, float]:
    if not with_gripper:
        return (0.0, 0.0)

    left_trigger = 0.5 + 0.5 * np.sin(timestamp)
    right_trigger = 0.5 + 0.5 * np.cos(timestamp)
    return (float(left_trigger), float(right_trigger))


def build_fake_input_frame(
    timestamp: float,
    *,
    pattern: FakeTeleoperationPattern = "dual_arm_circle",
    left_enable: bool = True,
    right_enable: bool = True,
    with_gripper: bool = False,
    sweep_arm: SweepArm = "auto",
) -> TeleopInputFrame:
    if pattern not in SUPPORTED_FAKE_TELEOP_PATTERNS:
        raise ValueError(
            f"Unsupported teleoperation pattern {pattern!r}. "
            f"Expected one of {SUPPORTED_FAKE_TELEOP_PATTERNS}."
        )

    left_translation = _LEFT_REST_TRANSLATION.copy()
    right_translation = _RIGHT_REST_TRANSLATION.copy()

    if pattern == "single_arm_sweep":
        active_arm = _resolve_sweep_arm(
            sweep_arm=sweep_arm,
            left_enable=left_enable,
            right_enable=right_enable,
        )
        sweep_phase = 2.0 * timestamp
        if active_arm == "left":
            left_translation[0] = 0.18 + 0.14 * np.sin(sweep_phase)
            left_translation[2] = 0.6 + 0.05 * np.cos(sweep_phase)
        else:
            right_translation[0] = 0.18 + 0.14 * np.sin(sweep_phase)
            right_translation[2] = 0.6 + 0.05 * np.cos(sweep_phase)
    elif pattern == "dual_arm_circle":
        orbit_phase = 2.0 * timestamp
        left_translation[0] = 0.2 + 0.1 * np.cos(orbit_phase)
        left_translation[1] = 0.2 + 0.1 * np.sin(orbit_phase)
        right_translation[0] = 0.2 + 0.1 * np.sin(orbit_phase)
        right_translation[1] = -0.2 - 0.1 * np.cos(orbit_phase)

    left_trigger, right_trigger = _build_gripper_targets(
        timestamp, with_gripper=with_gripper)

    return TeleopInputFrame(
        left=TeleopHandInput(
            pose=_build_pose(left_translation),
            button_pressed=left_enable,
            trigger=left_trigger,
        ),
        right=TeleopHandInput(
            pose=_build_pose(right_translation),
            button_pressed=right_enable,
            trigger=right_trigger,
        ),
    )


def build_fake_bridge_data(
    timestamp: float,
    *,
    pattern: FakeTeleoperationPattern = "dual_arm_circle",
    left_enable: bool = True,
    right_enable: bool = True,
    with_gripper: bool = False,
    sweep_arm: SweepArm = "auto",
) -> dict[str, dict[str, object]]:
    return build_fake_input_frame(
        timestamp,
        pattern=pattern,
        left_enable=left_enable,
        right_enable=right_enable,
        with_gripper=with_gripper,
        sweep_arm=sweep_arm,
    ).to_bridge_data()
