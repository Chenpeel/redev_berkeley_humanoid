from __future__ import annotations

import unittest

import numpy as np
from berkeley_humanoid_lite_lowlevel.teleoperation.webxr import (
    DEFAULT_WEBXR_POSITION_SCALE,
    QuestWebXrBridgeServer,
    QuestWebXrServerConfig,
    build_bridge_data_from_webxr_message,
    build_idle_bridge_data,
)


class TeleoperationWebXrTests(unittest.TestCase):
    def test_build_idle_bridge_data_matches_bridge_contract(self) -> None:
        bridge_data = build_idle_bridge_data()

        self.assertEqual(set(bridge_data), {"left", "right"})
        self.assertFalse(bridge_data["left"]["button_pressed"])
        self.assertFalse(bridge_data["right"]["button_pressed"])
        self.assertEqual(np.asarray(bridge_data["left"]["pose"]).shape, (4, 4))
        self.assertEqual(np.asarray(bridge_data["right"]["pose"]).shape, (4, 4))

    def test_build_bridge_data_from_webxr_message_transforms_pose_and_trigger(self) -> None:
        bridge_data = build_bridge_data_from_webxr_message(
            {
                "controllers": {
                    "left": {
                        "position": {"x": 0.1, "y": 0.2, "z": 0.3},
                        "quaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                        "gripActive": True,
                        "trigger": 0.75,
                    },
                    "right": {},
                }
            }
        )

        left_pose = np.asarray(bridge_data["left"]["pose"], dtype=np.float64)
        np.testing.assert_allclose(
            left_pose[:3, 3],
            np.array([-0.1, 0.1, 0.8], dtype=np.float64),
        )
        self.assertTrue(bridge_data["left"]["button_pressed"])
        self.assertAlmostEqual(float(bridge_data["left"]["trigger"]), 0.75)

    def test_build_bridge_data_accepts_telegrip_dual_controller_shape(self) -> None:
        bridge_data = build_bridge_data_from_webxr_message(
            {
                "leftController": {
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "quaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    "gripActive": False,
                    "trigger": 1.0,
                },
                "rightController": {
                    "position": {"x": -0.2, "y": 0.1, "z": -0.1},
                    "quaternion": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                    "gripActive": True,
                    "trigger": 0.4,
                },
            },
            position_scale=0.5,
        )

        right_pose = np.asarray(bridge_data["right"]["pose"], dtype=np.float64)
        np.testing.assert_allclose(
            right_pose[:3, 3],
            np.array([0.25, -0.1, 0.65], dtype=np.float64),
        )
        self.assertTrue(bridge_data["right"]["button_pressed"])
        self.assertAlmostEqual(float(bridge_data["right"]["trigger"]), 0.4)

    def test_build_bridge_data_rejects_non_positive_scale(self) -> None:
        with self.assertRaises(ValueError):
            build_bridge_data_from_webxr_message({}, position_scale=0.0)

    def test_default_scale_constant_matches_expected_contract(self) -> None:
        self.assertEqual(DEFAULT_WEBXR_POSITION_SCALE, 1.0)

    def test_render_ui_index_includes_controller_overlay_and_resolved_urls(self) -> None:
        bridge = QuestWebXrBridgeServer(
            QuestWebXrServerConfig(
                host="127.0.0.1",
                https_port=9443,
                websocket_port=9442,
            )
        )

        rendered_html = bridge.render_ui_index().decode("utf-8")

        self.assertIn("https://127.0.0.1:9443", rendered_html)
        self.assertIn("wss://127.0.0.1:9442", rendered_html)
        self.assertIn('id="scene-banner"', rendered_html)
        self.assertIn('id="scene-floor"', rendered_html)
        self.assertIn('id="left-controller-text"', rendered_html)
        self.assertIn('id="right-controller-text"', rendered_html)
        self.assertIn('material="side: double; transparent: true"', rendered_html)

    def test_static_app_asset_keeps_reference_space_fallback_and_tracking_status(self) -> None:
        bridge = QuestWebXrBridgeServer(QuestWebXrServerConfig())

        script_path = bridge.resolve_static_asset("/app.js")

        self.assertIsNotNone(script_path)
        script = script_path.read_text(encoding="utf-8")
        self.assertIn("referenceSpaceCandidates", script)
        self.assertIn("Retrying XR with local space", script)
        self.assertIn("Both controllers tracked", script)


if __name__ == "__main__":
    unittest.main()
