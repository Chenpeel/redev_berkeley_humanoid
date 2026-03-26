"""Compatibility adapter for legacy ``recoil.can_interface`` imports."""

from berkeley_humanoid_lite_lowlevel.recoil import Bus


class CanInterface(Bus):
    """Legacy alias that preserves the historical class name."""

