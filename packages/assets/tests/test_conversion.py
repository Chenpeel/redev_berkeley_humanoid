from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

from berkeley_humanoid_lite_assets import (
    get_format_config_path,
    get_format_dir,
    get_mesh_dir,
    get_robot_dir,
    get_scad_dir,
    get_urdf_export_config_path,
)
from berkeley_humanoid_lite_assets.conversion import (
    build_mjcf_sensor_block,
    inject_mjcf_sensors,
    load_onshape_export_layout,
    rewrite_mesh_references,
)


class AssetConversionTestCase(unittest.TestCase):
    def test_generic_path_helpers_match_robot_layout(self) -> None:
        robot_dir = get_robot_dir()

        self.assertEqual(get_mesh_dir(), robot_dir / "meshes")
        self.assertEqual(get_scad_dir(), robot_dir / "scad")
        self.assertEqual(get_format_dir("urdf"), robot_dir / "urdf")
        self.assertEqual(get_format_config_path("urdf"), get_urdf_export_config_path())

    def test_load_onshape_export_layout_reads_existing_configuration(self) -> None:
        layout = load_onshape_export_layout(get_urdf_export_config_path())

        self.assertEqual(layout.export_directory.name, "urdf")
        self.assertEqual(layout.robot_directory, get_robot_dir())
        self.assertEqual(layout.mesh_directory, get_mesh_dir())
        self.assertEqual(layout.staging_directory, get_format_dir("urdf") / "assets")

    def test_rewrite_mesh_references_normalizes_supported_prefixes(self) -> None:
        rewritten = rewrite_mesh_references("package://./assets/merged/example.stl")
        self.assertEqual(rewritten, "../meshes/example.stl")

    def test_inject_mjcf_sensors_adds_single_sensor_block(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            mjcf_path = Path(temporary_directory) / "robot.xml"
            mjcf_path.write_text("<mujoco><worldbody/><actuator/></mujoco>", encoding="utf-8")

            inject_mjcf_sensors(mjcf_path, output_stem="berkeley_humanoid_lite")
            inject_mjcf_sensors(mjcf_path, output_stem="berkeley_humanoid_lite")

            content = mjcf_path.read_text(encoding="utf-8")
            self.assertEqual(content.count("<sensor>"), 1)
            self.assertIn('name="imu_gyro"', content)
            self.assertIn('name="leg_left_hip_roll_pos"', content)

    def test_build_mjcf_sensor_block_contains_expected_joint_sensor_names(self) -> None:
        sensor_block = build_mjcf_sensor_block("berkeley_humanoid_lite_biped")

        self.assertIn('name="leg_left_hip_roll_pos"', sensor_block)
        self.assertNotIn('name="arm_left_shoulder_pitch_pos"', sensor_block)


if __name__ == "__main__":
    unittest.main()
