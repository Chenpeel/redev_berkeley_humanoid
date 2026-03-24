from __future__ import annotations

from pathlib import Path
import unittest
import xml.etree.ElementTree as element_tree

from berkeley_humanoid_lite_assets.paths import get_mjcf_path
from berkeley_humanoid_lite_assets.robots import BIPED_VARIANT, FULL_BODY_VARIANT, RobotVariantSpecification


def _get_included_model_path(robot_variant: RobotVariantSpecification) -> Path:
    scene_path = get_mjcf_path(robot_variant.mjcf_scene_file_name)
    root = element_tree.parse(scene_path).getroot()
    include_element = root.find("include")

    assert include_element is not None, f"{scene_path.name} 缺少 include 节点"
    included_file = include_element.get("file")
    assert included_file, f"{scene_path.name} 的 include 缺少 file 属性"

    model_path = scene_path.parent / included_file
    assert model_path.is_file(), f"{scene_path.name} 引用的主体 XML 不存在: {model_path}"
    return model_path


def _iter_mesh_paths(model_path: Path) -> list[Path]:
    root = element_tree.parse(model_path).getroot()
    mesh_paths: list[Path] = []
    for mesh_element in root.findall("./asset/mesh"):
        mesh_file = mesh_element.get("file")
        assert mesh_file, f"{model_path.name} 存在缺少 file 属性的 mesh 节点"
        mesh_paths.append((model_path.parent / mesh_file).resolve())
    return mesh_paths


class MjcfAssetIntegrityTestCase(unittest.TestCase):
    def test_mjcf_mesh_references_resolve_to_existing_files(self) -> None:
        for robot_variant in (BIPED_VARIANT, FULL_BODY_VARIANT):
            with self.subTest(variant=robot_variant.name):
                model_path = _get_included_model_path(robot_variant)
                mesh_paths = _iter_mesh_paths(model_path)

                self.assertTrue(mesh_paths, f"{model_path.name} 未声明任何 mesh 资源")
                missing_paths = [mesh_path for mesh_path in mesh_paths if not mesh_path.is_file()]
                self.assertFalse(
                    missing_paths,
                    (
                        f"{robot_variant.name} 变体存在失效 mesh 引用: "
                        + ", ".join(str(mesh_path) for mesh_path in missing_paths)
                    ),
                )


if __name__ == "__main__":
    unittest.main()
