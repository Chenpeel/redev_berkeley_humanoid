import argparse
from pathlib import Path

from berkeley_humanoid_lite_assets import export_onshape_to_urdf, get_urdf_export_config_path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate a URDF file from the Onshape robot configuration.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        help="Path to the export configuration file.",
        default=get_urdf_export_config_path(),
    )
    arguments = parser.parse_args()

    output_path = export_onshape_to_urdf(arguments.config)
    print(f"Generated URDF file: {output_path}")


if __name__ == "__main__":
    main()
