"""Legacy compatibility exports for recoil scripts."""

from berkeley_humanoid_lite_lowlevel.recoil import Bus as Bus

from .can_interface import CanInterface as CanInterface

__all__ = ["Bus", "CanInterface"]
