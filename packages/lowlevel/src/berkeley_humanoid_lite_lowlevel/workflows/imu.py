from __future__ import annotations

import time

from berkeley_humanoid_lite_lowlevel.sensors import SerialOrientationStream


DEFAULT_IMU_SERIAL_DEVICE = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:4.1:1.0"
DEFAULT_IMU_BAUDRATE = 1_000_000
DEFAULT_IMU_TIMEOUT = 0.001


def stream_orientation(
    *,
    device: str = DEFAULT_IMU_SERIAL_DEVICE,
    baudrate: int = DEFAULT_IMU_BAUDRATE,
    timeout: float = DEFAULT_IMU_TIMEOUT,
) -> None:
    """持续输出串口姿态数据。"""
    with SerialOrientationStream(device, baudrate=baudrate, timeout=timeout) as orientation_stream:
        try:
            while True:
                frame_started_at = time.perf_counter()
                sample = orientation_stream.read_sample()
                if sample is None:
                    continue

                roll, pitch, yaw = sample.to_euler_degrees()
                elapsed = time.perf_counter() - frame_started_at
                frequency_hz = 0.0 if elapsed <= 0.0 else 1.0 / elapsed

                print(
                    f"x: {roll:.2f} deg, "
                    f"y: {pitch:.2f} deg, "
                    f"z: {yaw:.2f} deg, "
                    f"freq: {frequency_hz:.2f} Hz",
                    end="\r",
                )
        except KeyboardInterrupt:
            print()
            print("Keyboard interrupt")
