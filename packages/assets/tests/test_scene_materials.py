from __future__ import annotations

from io import BytesIO

import berkeley_humanoid_lite_assets.paths as asset_paths
import berkeley_humanoid_lite_assets.scene_materials as scene_materials


class FakeResponse(BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()
        return False


def configure_package_root(monkeypatch, tmp_path):
    package_root = tmp_path / "berkeley_humanoid_lite_assets"
    (package_root / "data").mkdir(parents=True)
    monkeypatch.setattr(asset_paths, "_PACKAGE_ROOT", package_root)
    return package_root


def test_resolve_scene_material_path_prefers_standard_name(monkeypatch, tmp_path):
    configure_package_root(monkeypatch, tmp_path)
    materials_dir = asset_paths.ensure_scene_materials_dir()
    (materials_dir / "Shingles_01.mdl").write_text("preset", encoding="utf-8")
    preferred_path = materials_dir / "ground_surface.mdl"
    preferred_path.write_text("manual", encoding="utf-8")

    assert scene_materials.resolve_scene_material_path() == preferred_path


def test_get_scene_material_preset_uses_requested_version():
    preset = scene_materials.get_scene_material_preset("isaac-shingles-01", preset_version="5.1")

    assert preset.version == "5.1"
    assert preset.downloads[0].source_url.endswith("/Assets/Isaac/5.1/NVIDIA/Materials/Base/Architecture/Shingles_01.mdl")
    assert len(preset.downloads) == 4


def test_download_scene_material_preset_stages_bundle(monkeypatch, tmp_path):
    configure_package_root(monkeypatch, tmp_path)

    def fake_urlopen(url):
        return FakeResponse(url.encode("utf-8"))

    monkeypatch.setattr(scene_materials, "urlopen", fake_urlopen)
    prepared_paths = scene_materials.download_scene_material_preset("isaac-shingles-01", preset_version="5.1")

    assert [path.relative_to(asset_paths.get_scene_materials_dir()).as_posix() for path in prepared_paths] == [
        "Shingles_01.mdl",
        "Shingles_01/Shingles_01_BaseColor.png",
        "Shingles_01/Shingles_01_ORM.png",
        "Shingles_01/Shingles_01_Normal.png",
    ]
    assert prepared_paths[0].read_text(encoding="utf-8").endswith("/Assets/Isaac/5.1/NVIDIA/Materials/Base/Architecture/Shingles_01.mdl")
