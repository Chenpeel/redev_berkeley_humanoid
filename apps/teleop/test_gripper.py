# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import argparse

from berkeley_humanoid_lite_lowlevel.workflows import stream_gripper_targets


def main() -> None:
    parser = argparse.ArgumentParser(description="Stream teleoperation gripper targets")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Gripper serial port")
    parser.add_argument("--baudrate", type=int, default=115200, help="Gripper serial baudrate")
    parser.add_argument("--left-target", type=float, default=0.8, help="Normalized left gripper target")
    parser.add_argument("--right-target", type=float, default=0.8, help="Normalized right gripper target")
    parser.add_argument("--period-seconds", type=float, default=0.1, help="Command streaming period")
    arguments = parser.parse_args()

    stream_gripper_targets(
        port=arguments.port,
        baudrate=arguments.baudrate,
        left_target=arguments.left_target,
        right_target=arguments.right_target,
        period_seconds=arguments.period_seconds,
    )


if __name__ == "__main__":
    main()
