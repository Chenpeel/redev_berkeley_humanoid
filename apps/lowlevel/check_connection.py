# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

from berkeley_humanoid_lite_lowlevel.cli import run_with_friendly_lowlevel_errors


def main() -> None:
    from berkeley_humanoid_lite_lowlevel.workflows import check_locomotion_connection

    check_locomotion_connection()


if __name__ == "__main__":
    run_with_friendly_lowlevel_errors(main)
