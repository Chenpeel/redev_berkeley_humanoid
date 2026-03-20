from __future__ import annotations

import time
from dataclasses import dataclass

import serial

from berkeley_humanoid_lite_lowlevel.robot.imu import Baudrate, SamplingRate, SerialImu
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
_HIWONDER_SAMPLING_RATE_CODES = {
    0.2: SamplingRate.RATE_0_2_HZ,
    0.5: SamplingRate.RATE_0_5_HZ,
    1.0: SamplingRate.RATE_1_HZ,
    2.0: SamplingRate.RATE_2_HZ,
    5.0: SamplingRate.RATE_5_HZ,
    10.0: SamplingRate.RATE_10_HZ,
    20.0: SamplingRate.RATE_20_HZ,
    50.0: SamplingRate.RATE_50_HZ,
    100.0: SamplingRate.RATE_100_HZ,
    200.0: SamplingRate.RATE_200_HZ,
}
_HIWONDER_OUTPUT_PROFILES = {
    "control": {
        "time": False,
        "acceleration": False,
        "angular_velocity": True,
        "angle": True,
        "magnetic_field": False,
        "port_status": False,
        "pressure": False,
        "gps": False,
        "velocity": False,
        "quaternion": True,
        "gps_position_accuracy": False,
    },
    "minimal": {
        "time": False,
        "acceleration": False,
        "angular_velocity": False,
        "angle": True,
        "magnetic_field": False,
        "port_status": False,
        "pressure": False,
        "gps": False,
        "velocity": False,
        "quaternion": True,
        "gps_position_accuracy": False,
    },
    "full": {
        "time": True,
        "acceleration": True,
        "angular_velocity": True,
        "angle": True,
        "magnetic_field": True,
        "port_status": False,
        "pressure": True,
        "gps": False,
        "velocity": False,
        "quaternion": True,
        "gps_position_accuracy": False,
    },
}
SUPPORTED_HIWONDER_OUTPUT_PROFILES = tuple(_HIWONDER_OUTPUT_PROFILES)


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


def normalize_hiwonder_sampling_rate(rate_hz: float | int) -> int:
    """把 HiWonder 采样率转换为驱动内部使用的枚举值。"""
    normalized_rate_hz = float(rate_hz)
    for supported_rate_hz, sampling_rate_code in _HIWONDER_SAMPLING_RATE_CODES.items():
        if abs(normalized_rate_hz - supported_rate_hz) < 1e-9:
            return sampling_rate_code

    supported_values = ", ".join(
        str(int(value) if value.is_integer() else value)
        for value in _HIWONDER_SAMPLING_RATE_CODES
    )
    raise ValueError(
        f"Unsupported HiWonder sampling rate: {rate_hz}. Supported values: {supported_values}"
    )


def resolve_hiwonder_output_content(
    *,
    profile: str = "control",
    time_output: bool | None = None,
    acceleration_output: bool | None = None,
    angular_velocity_output: bool | None = None,
    angle_output: bool | None = None,
    magnetic_field_output: bool | None = None,
    pressure_output: bool | None = None,
    quaternion_output: bool | None = None,
) -> dict[str, bool]:
    if profile not in _HIWONDER_OUTPUT_PROFILES:
        supported_profiles = ", ".join(SUPPORTED_HIWONDER_OUTPUT_PROFILES)
        raise ValueError(f"Unsupported HiWonder output profile: {profile}. Supported profiles: {supported_profiles}")

    output_content = dict(_HIWONDER_OUTPUT_PROFILES[profile])
    explicit_values = {
        "time": time_output,
        "acceleration": acceleration_output,
        "angular_velocity": angular_velocity_output,
        "angle": angle_output,
        "magnetic_field": magnetic_field_output,
        "pressure": pressure_output,
        "quaternion": quaternion_output,
    }
    for key, value in explicit_values.items():
        if value is not None:
            output_content[key] = value
    return output_content


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


def configure_hiwonder_output(
    *,
    device: str = DEFAULT_IMU_SERIAL_DEVICE,
    baudrate: str | int | None = DEFAULT_IMU_BAUDRATE,
    timeout: float = DEFAULT_IMU_TIMEOUT,
    probe_duration: float = DEFAULT_IMU_PROBE_DURATION,
    target_baudrate: int | None = None,
    rate_hz: float = 10.0,
    profile: str = "control",
    time_output: bool | None = None,
    acceleration_output: bool | None = None,
    angular_velocity_output: bool | None = None,
    angle_output: bool | None = None,
    magnetic_field_output: bool | None = None,
    pressure_output: bool | None = None,
    quaternion_output: bool | None = None,
    save: bool = False,
) -> ImuStreamConfiguration:
    configuration = resolve_imu_stream_configuration(
        protocol=_PROTOCOL_HIWONDER,
        device=device,
        baudrate=baudrate,
        timeout=timeout,
        probe_duration=probe_duration,
    )
    output_content = resolve_hiwonder_output_content(
        profile=profile,
        time_output=time_output,
        acceleration_output=acceleration_output,
        angular_velocity_output=angular_velocity_output,
        angle_output=angle_output,
        magnetic_field_output=magnetic_field_output,
        pressure_output=pressure_output,
        quaternion_output=quaternion_output,
    )

    imu = SerialImu(
        port=configuration.device,
        baudrate=normalize_hiwonder_baudrate(configuration.baudrate),
        read_timeout=timeout,
    )
    effective_baudrate = configuration.baudrate if target_baudrate is None else int(target_baudrate)
    try:
        imu.unlock()
        time.sleep(0.1)
        if target_baudrate is not None:
            imu.set_baudrate(normalize_hiwonder_baudrate(target_baudrate))
            time.sleep(0.1)
        imu.set_sampling_rate(normalize_hiwonder_sampling_rate(rate_hz))
        time.sleep(0.1)
        imu.set_output_content(**output_content)
        time.sleep(0.1)
        if save:
            imu.save()
            time.sleep(0.1)
    finally:
        imu.close()

    output_summary = ", ".join(
        key for key, enabled in output_content.items() if enabled
    ) or "none"
    persistence = "saved to device" if save else "applied temporarily"
    print(
        f"Configured HiWonder IMU on {configuration.device} at {effective_baudrate} baud: "
        f"rate={rate_hz:g}Hz, profile={profile}, outputs={output_summary} ({persistence})"
    )
    return ImuStreamConfiguration(
        protocol=configuration.protocol,
        device=configuration.device,
        baudrate=effective_baudrate,
    )


def _stream_packet(configuration: ImuStreamConfiguration, *, timeout: float) -> None:
    with SerialOrientationStream(
        configuration.device,
        baudrate=configuration.baudrate,
        timeout=timeout,
    ) as orientation_stream:
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
