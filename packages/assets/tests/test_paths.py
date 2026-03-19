from berkeley_humanoid_lite_assets.paths import (
    get_data_dir,
    get_mjcf_export_config_path,
    get_mjcf_path,
    get_robot_dir,
    get_urdf_export_config_path,
    get_urdf_path,
    get_usd_path,
)


def test_robot_asset_paths_resolve_to_existing_files():
    robot_dir = get_robot_dir()

    assert robot_dir.is_dir()
    assert get_data_dir().is_dir()
    assert get_mjcf_export_config_path().is_file()
    assert get_mjcf_path("bhl_scene.xml").is_file()
    assert get_urdf_export_config_path().is_file()
    assert get_urdf_path().is_file()
    assert get_usd_path().is_file()
