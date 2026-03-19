from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as element_tree

from ..robots import FULL_BODY_JOINT_NAMES, LEG_JOINT_NAMES


_OUTPUT_STEM_TO_JOINT_NAMES = {
    "berkeley_humanoid_lite": FULL_BODY_JOINT_NAMES,
    "berkeley_humanoid_lite_biped": LEG_JOINT_NAMES,
}
_MESH_REFERENCE_REPLACEMENTS: tuple[tuple[str, str], ...] = (
    ("package://./assets/merged/", "../meshes/"),
    ("package://assets/merged/", "../meshes/"),
    ("./assets/merged/", "../meshes/"),
    ("assets/merged/", "../meshes/"),
    ("package://../meshes/", "../meshes/"),
    ("package://", "./"),
)


@dataclass(frozen=True)
class MjcfSensorDefinition:
    sensor_type: str
    attributes: dict[str, str]


def rewrite_mesh_references(content: str) -> str:
    """统一导出文件中的 mesh 引用路径。"""
    updated_content = content
    for source, target in _MESH_REFERENCE_REPLACEMENTS:
        updated_content = updated_content.replace(source, target)
    return updated_content


def build_mjcf_sensor_definitions(output_stem: str) -> tuple[MjcfSensorDefinition, ...]:
    """根据模型名构造默认 MJCF 传感器定义。"""
    try:
        joint_names = _OUTPUT_STEM_TO_JOINT_NAMES[output_stem]
    except KeyError as error:
        available_names = ", ".join(sorted(_OUTPUT_STEM_TO_JOINT_NAMES))
        raise ValueError(f"未知导出模型名称: {output_stem}. 可选值: {available_names}") from error

    definitions: list[MjcfSensorDefinition] = []
    for sensor_type, sensor_suffix in (("jointpos", "pos"), ("jointvel", "vel"), ("jointactuatorfrc", "torque")):
        for joint_name in joint_names:
            sensor_name = joint_name.removesuffix("_joint")
            definitions.append(
                MjcfSensorDefinition(
                    sensor_type=sensor_type,
                    attributes={"name": f"{sensor_name}_{sensor_suffix}", "joint": joint_name},
                )
            )

    definitions.extend(
        (
            MjcfSensorDefinition("framequat", {"name": "imu_quat", "objtype": "site", "objname": "imu"}),
            MjcfSensorDefinition("gyro", {"name": "imu_gyro", "site": "imu"}),
            MjcfSensorDefinition("accelerometer", {"name": "imu_acc", "site": "imu"}),
            MjcfSensorDefinition("framepos", {"name": "frame_pos", "objtype": "site", "objname": "imu"}),
            MjcfSensorDefinition("framelinvel", {"name": "frame_vel", "objtype": "site", "objname": "imu"}),
        )
    )
    return tuple(definitions)


def build_mjcf_sensor_block(output_stem: str) -> str:
    """构造 MJCF 传感器区块的字符串表示。"""
    lines = ["", "  <sensor>"]
    for definition in build_mjcf_sensor_definitions(output_stem):
        attributes = " ".join(f'{key}="{value}"' for key, value in definition.attributes.items())
        lines.append(f"    <{definition.sensor_type} {attributes}/>")
    lines.append("  </sensor>")
    return "\n".join(lines)


def inject_mjcf_sensors(mjcf_path: str | Path, *, output_stem: str) -> Path:
    """以结构化 XML 方式注入标准 MJCF 传感器。"""
    resolved_mjcf_path = Path(mjcf_path).expanduser().resolve()
    parser = element_tree.XMLParser(target=element_tree.TreeBuilder(insert_comments=True))
    tree = element_tree.parse(resolved_mjcf_path, parser=parser)
    root = tree.getroot()

    existing_sensor_block = root.find("sensor")
    if existing_sensor_block is not None:
        root.remove(existing_sensor_block)

    sensor_block = element_tree.Element("sensor")
    for definition in build_mjcf_sensor_definitions(output_stem):
        element_tree.SubElement(sensor_block, definition.sensor_type, attrib=definition.attributes)

    child_elements = list(root)
    actuator_index = next((index for index, child in enumerate(child_elements) if child.tag == "actuator"), None)
    if actuator_index is None:
        root.append(sensor_block)
    else:
        root.insert(actuator_index + 1, sensor_block)

    element_tree.indent(tree, space="  ")
    tree.write(resolved_mjcf_path, encoding="utf-8")
    return resolved_mjcf_path
