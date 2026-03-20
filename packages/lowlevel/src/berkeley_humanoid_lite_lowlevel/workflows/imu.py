from __future__ import annotations

from dataclasses import dataclass
import time

import serial

from berkeley_humanoid_lite_lowlevel.robot.imu import Baudrate, SerialImu
from berkeley_humanoid_lite_lowlevel.sensors import SerialOrientationStream, discover_orientation_devices
from berkeley_humanoid_lite_lowlevel.sensors.orientation import AUTO_DETECT_SERIAL_DEVICE


DEFAULT_IMU_SERIAL_DEVICE = AUTO_DETECT_SERIAL_DEVICE
DEFAULT_IMU_PROTOCOL = "auto"
DEFAULT_IMU_BAUDRATE = "auto"
DEFAULT_IMU_TIMEOUT = 0.01
DEFAULT_IMU_PROBE_DURATION = 0.5

_PROTOCOL_AUTO = "auto"
_PROTOCOL_HIWONDER = "hiwonder"
_PROTOCOL_PACKET = "packet"
SUPPORTED_IMU_PROTOCOLS = (_PROTOCOL_AUTO, _PROTOCOL_HIWONDER, _PROTOCOL_PACKET)

_HIWONDER_BAUDRATE_CODES = {
    4800: Baudrate.BAUD_4800,
    9600: Baudrate.BAUD_9600,
    19200: Baudrate.BAUD_19200,
    38400: Baudrate.BAUD_38400,
    57600: Baudrate.BAUD_57600,
    115200: Baudrate.BAUD_115200,
    230400: Baudrate.BAUD_230400,
    460800: Baudrate.BAUD_460800,
}
_HIWONDER_DEFAULT_BAUDRATES = (460800, 115200, 230400, 9600, 19200, 38400, 57600)
_PACKET_DEFAULT_BAUDRATES = (1_000_000,)


@dataclass(frozen=True)
class ImuStreamConfiguration:
    protocol: str
    device: str
    baudrate: int


def parse_baudrate_argument(baudrate: str | int | None) -> int | None:
    """解析命令行里的波特率参数，支持 auto。"""
    if baudrate is None:
        return None
    if isinstance(baudrate, int):
        return baudrate

    normalized = baudrate.strip().lower()
    if normalized == "auto":
        return None
    return int(normalized)


def normalize_hiwonder_baudrate(baudrate: int) -> int:
    """把 HiWonder 波特率转换为驱动内部使用的枚举值。"""
    if baudrate in _HIWONDER_BAUDRATE_CODES.values():
        return baudrate
    if baudrate in _HIWONDER_BAUDRATE_CODES:
        return _HIWONDER_BAUDRATE_CODES[baudrate]

    supported_values = ", ".join(str(value) for value in _HIWONDER_BAUDRATE_CODES)
    raise ValueError(f"Unsupported HiWonder baudrate: {baudrate}. Supported values: {supported_values}")


def _resolve_probe_devices(device: str) -> tuple[str, ...]:
    if device != AUTO_DETECT_SERIAL_DEVICE:
        return (device,)

    devices = tuple(discover_orientation_devices())
    if not devices:
        raise serial.SerialException(
            "No IMU serial device found. Pass --device to specify the serial port explicitly."
        )
    return devices


def _build_probe_configurations(
    *,
    protocol: str,
    device: str,
    baudrate: int | None,
) -> list[ImuStreamConfiguration]:
    devices = _resolve_probe_devices(device)

    if protocol == _PROTOCOL_AUTO:
        protocols = (_PROTOCOL_HIWONDER, _PROTOCOL_PACKET)
    else:
        protocols = (protocol,)

    configurations: list[ImuStreamConfiguration] = []
    for candidate_device in devices:
        for candidate_protocol in protocols:
            if baudrate is None:
                baudrates = (
                    _HIWONDER_DEFAULT_BAUDRATES
                    if candidate_protocol == _PROTOCOL_HIWONDER
                    else _PACKET_DEFAULT_BAUDRATES
                )
            else:
                baudrates = (baudrate,)

            for candidate_baudrate in baudrates:
                configurations.append(
                    ImuStreamConfiguration(
                        protocol=candidate_protocol,
                        device=candidate_device,
                        baudrate=candidate_baudrate,
                    )
                )
    return configurations


def _probe_hiwonder(device: str, *, baudrate: int, timeout: float, probe_duration: float) -> bool:
    imu = SerialImu(
        port=device,
        baudrate=normalize_hiwonder_baudrate(baudrate),
        read_timeout=timeout,
    )
    started_at = time.perf_counter()
    try:
        while time.perf_counter() - started_at < probe_duration:
            if imu.read_frame():
                return True
    finally:
        imu.close()
    return False


def _probe_packet(device: str, *, baudrate: int, timeout: float, probe_duration: float) -> bool:
    with SerialOrientationStream(device, baudrate=baudrate, timeout=timeout) as orientation_stream:
        started_at = time.perf_counter()
        while time.perf_counter() - started_at < probe_duration:
            if orientation_stream.read_sample() is not None:
                return True
    return False


def detect_imu_stream(
    *,
    protocol: str = DEFAULT_IMU_PROTOCOL,
    device: str = DEFAULT_IMU_SERIAL_DEVICE,
    baudrate: str | int | None = DEFAULT_IMU_BAUDRATE,
    timeout: float = DEFAULT_IMU_TIMEOUT,
    probe_duration: float = DEFAULT_IMU_PROBE_DURATION,
) -> ImuStreamConfiguration:
    """自动探测当前 IMU 的协议、串口和波特率。"""
    normalized_baudrate = parse_baudrate_argument(baudrate)
    last_error: Exception | None = None

    for configuration in _build_probe_configurations(
        protocol=protocol,
        device=device,
        baudrate=normalized_baudrate,
    ):
        try:
            if configuration.protocol == _PROTOCOL_HIWONDER:
                if _probe_hiwonder(
                    configuration.device,
                    baudrate=configuration.baudrate,
                    timeout=timeout,
                    probe_duration=probe_duration,
                ):
                    return configuration
            else:
                if _probe_packet(
                    configuration.device,
                    baudrate=configuration.baudrate,
                    timeout=timeout,
                    probe_duration=probe_duration,
                ):
                    return configuration
        except (serial.SerialException, OSError, ValueError) as error:
            last_error = error

    if last_error is not None:
        raise serial.SerialException(
            "Unable to detect a working IMU stream configuration. "
            "Pass --protocol / --device / --baudrate to override explicitly."
        ) from last_error

    raise serial.SerialException(
        "Unable to detect a working IMU stream configuration. "
        "Pass --protocol / --device / --baudrate to override explicitly."
    )


def resolve_imu_stream_configuration(
    *,
    protocol: str = DEFAULT_IMU_PROTOCOL,
    device: str = DEFAULT_IMU_SERIAL_DEVICE,
    baudrate: str | int | None = DEFAULT_IMU_BAUDRATE,
    timeout: float = DEFAULT_IMU_TIMEOUT,
    probe_duration: float = DEFAULT_IMU_PROBE_DURATION,
) -> ImuStreamConfiguration:
    """解析最终 IMU 配置；显式参数完整时直接使用，含 auto 时再走探测。"""
    normalized_baudrate = parse_baudrate_argument(baudrate)
    if (
        protocol != _PROTOCOL_AUTO
        and device != AUTO_DETECT_SERIAL_DEVICE
        and normalized_baudrate is not None
    ):
        return ImuStreamConfiguration(
            protocol=protocol,
            device=device,
            baudrate=normalized_baudrate,
        )

    return detect_imu_stream(
        protocol=protocol,
        device=device,
        baudrate=normalized_baudrate,
        timeout=timeout,
        probe_duration=probe_duration,
    )


def _stream_packet(configuration: ImuStreamConfiguration, *, timeout: float) -> None:
    with SerialOrientationStream(configuration.device, baudrate=configuration.baudrate, timeout=timeout) as orientation_stream:
        print(
            f"Streaming packet IMU from {configuration.device} at {configuration.baudrate} baud"
        )
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


def _stream_hiwonder(configuration: ImuStreamConfiguration, *, timeout: float) -> None:
    imu = SerialImu(
        port=configuration.device,
        baudrate=normalize_hiwonder_baudrate(configuration.baudrate),
        read_timeout=timeout,
    )
    print(
        f"Streaming HiWonder IM10A from {configuration.device} at {configuration.baudrate} baud"
    )
    imu.run_forever()

    try:
        while True:
            print(
                f"roll: {imu.angle[0]:7.2f} deg, "
                f"pitch: {imu.angle[1]:7.2f} deg, "
                f"yaw: {imu.angle[2]:7.2f} deg, "
                f"gx: {imu.angular_velocity[0]:7.2f} deg/s, "
                f"gy: {imu.angular_velocity[1]:7.2f} deg/s, "
                f"gz: {imu.angular_velocity[2]:7.2f} deg/s, "
                f"qw: {imu.quaternion[0]:6.3f}",
                end="\r",
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
        print("Keyboard interrupt")
    finally:
        imu.stop()
        imu.close()


def stream_orientation(
    *,
    protocol: str = DEFAULT_IMU_PROTOCOL,
    device: str = DEFAULT_IMU_SERIAL_DEVICE,
    baudrate: str | int | None = DEFAULT_IMU_BAUDRATE,
    timeout: float = DEFAULT_IMU_TIMEOUT,
    probe_duration: float = DEFAULT_IMU_PROBE_DURATION,
) -> None:
    """持续输出串口 IMU 姿态数据，支持自动探测协议和波特率。"""
    configuration = resolve_imu_stream_configuration(
        protocol=protocol,
        device=device,
        baudrate=baudrate,
        timeout=timeout,
        probe_duration=probe_duration,
    )

    if configuration.protocol == _PROTOCOL_HIWONDER:
        _stream_hiwonder(configuration, timeout=timeout)
        return

    _stream_packet(configuration, timeout=timeout)
