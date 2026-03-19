from __future__ import annotations

from enum import IntEnum


class LocomotionControlState(IntEnum):
    """低层 locomotion 控制状态。"""

    INVALID = 0
    IDLE = 1
    INITIALIZING = 2
    POLICY_CONTROL = 3
