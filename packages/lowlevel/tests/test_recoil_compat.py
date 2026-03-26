from berkeley_humanoid_lite_lowlevel.recoil import Bus
from recoil import CanInterface
from recoil.can_interface import CanInterface as ImportedCanInterface


def test_legacy_recoil_can_interface_import_is_available() -> None:
    assert ImportedCanInterface is CanInterface
    assert issubclass(CanInterface, Bus)
    assert hasattr(CanInterface, "read_position_measured")
