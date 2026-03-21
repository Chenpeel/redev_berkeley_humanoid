from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np
from loop_rate_limiters import RateLimiter

import berkeley_humanoid_lite_lowlevel.recoil as recoil

DEFAULT_ACTUATOR_BITRATE = 1_000_000
ActuatorAngleStage = tuple[str, float]


@dataclass(frozen=True)
class ActuatorAngleAssessment:
    stage_name: str
    target_angle_radians: float
    settled: bool
    passed: bool
    settle_time_seconds: float | None
    hold_max_position_error_radians: float
    hold_max_velocity_radians_per_second: float
    final_position_error_radians: float | None
    final_velocity_radians_per_second: float | None
    failure_reason: str | None = None


def create_actuator_bus(channel: str, bitrate: int = DEFAULT_ACTUATOR_BITRATE) -> recoil.Bus:
    return recoil.Bus(channel=channel, bitrate=bitrate)


def ping_actuator(bus: recoil.Bus, device_id: int) -> bool:
    return bool(bus.ping(device_id))


def calibrate_actuator_electrical_offset(
    bus: recoil.Bus,
    device_id: int,
    *,
    wait_seconds: float = 20.0,
) -> None:
    bus.set_mode(device_id, recoil.Mode.CALIBRATION)
    time.sleep(wait_seconds)


def interpolate_value(start: float, end: float, alpha: float) -> float:
    clamped_alpha = min(max(alpha, 0.0), 1.0)
    return start * (1.0 - clamped_alpha) + end * clamped_alpha


def resolve_angle_radians(
    angle_radians: float | None,
    angle_degrees: float | None,
    *,
    default: float | None = None,
) -> float | None:
    if angle_radians is not None:
        return angle_radians
    if angle_degrees is not None:
        return math.radians(angle_degrees)
    return default


def build_actuator_angle_sequence(
    target_angle_radians: float,
    return_angle_radians: float | None = None,
    cycles: int | None = None,
) -> list[ActuatorAngleStage]:
    if cycles is None:
        return [("target", target_angle_radians)]

    if cycles <= 0:
        raise ValueError("cycles must be a positive integer")

    resolved_return_angle = 0.0 if return_angle_radians is None else return_angle_radians
    sequence: list[ActuatorAngleStage] = []
    for cycle_index in range(cycles):
        cycle_name = f"cycle {cycle_index + 1}/{cycles}"
        sequence.append((f"{cycle_name}: target", target_angle_radians))
        sequence.append((f"{cycle_name}: return", resolved_return_angle))
    return sequence


def sample_within_tolerance(
    position_error_radians: float,
    velocity_radians_per_second: float,
    *,
    position_tolerance_radians: float,
    velocity_tolerance_radians_per_second: float,
) -> bool:
    return (
        position_error_radians <= position_tolerance_radians
        and velocity_radians_per_second <= velocity_tolerance_radians_per_second
    )


def _format_optional_float(value: float | None) -> str:
    if value is None:
        return "n/a"
    return f"{value:.2f}"


def print_actuator_angle_assessment(assessment: ActuatorAngleAssessment) -> None:
    status = "PASS" if assessment.passed else "FAIL"
    settle_time = _format_optional_float(assessment.settle_time_seconds)
    final_position_error_degrees = _format_optional_float(
        None
        if assessment.final_position_error_radians is None
        else math.degrees(assessment.final_position_error_radians)
    )
    final_velocity_degrees = _format_optional_float(
        None
        if assessment.final_velocity_radians_per_second is None
        else math.degrees(assessment.final_velocity_radians_per_second)
    )

    print(
        f"[{status}] {assessment.stage_name}: settle={settle_time}s  "
        f"hold_err_max={math.degrees(assessment.hold_max_position_error_radians):.2f} deg  "
        f"hold_vel_max={math.degrees(assessment.hold_max_velocity_radians_per_second):.2f} deg/s  "
        f"final_err={final_position_error_degrees} deg  "
        f"final_vel={final_velocity_degrees} deg/s"
    )
    if assessment.failure_reason is not None:
        print(f"Reason: {assessment.failure_reason}")


def move_actuator_to_angle(
    bus: recoil.Bus,
    rate: RateLimiter,
    device_id: int,
    *,
    target_angle_radians: float,
    max_speed_radians_per_second: float,
    hold_seconds: float,
    stage_name: str,
    position_tolerance_radians: float,
    velocity_tolerance_radians_per_second: float,
    settle_timeout_seconds: float,
    required_stable_samples: int,
) -> ActuatorAngleAssessment:
    current_angle_radians = bus.read_position_measured(device_id)
    if current_angle_radians is None:
        raise RuntimeError(f"Failed to read actuator #{device_id} position")

    ramp_duration_seconds = max(
        abs(target_angle_radians - current_angle_radians) / max_speed_radians_per_second,
        0.2,
    )

    print()
    print(f"[{stage_name}]")
    print(
        f"Current angle: {current_angle_radians:.4f} rad "
        f"({math.degrees(current_angle_radians):.2f} deg)"
    )
    print(
        f"Target angle:  {target_angle_radians:.4f} rad "
        f"({math.degrees(target_angle_radians):.2f} deg)"
    )
    print(f"Ramp duration: {ramp_duration_seconds:.2f} s")
    print(
        "Acceptance: "
        f"position<={math.degrees(position_tolerance_radians):.2f} deg, "
        f"velocity<={math.degrees(velocity_tolerance_radians_per_second):.2f} deg/s, "
        f"stable_samples>={required_stable_samples}, "
        f"settle_timeout={settle_timeout_seconds:.2f} s"
    )

    ramp_start_time = time.monotonic()
    while True:
        elapsed_seconds = time.monotonic() - ramp_start_time
        alpha = elapsed_seconds / ramp_duration_seconds
        command_angle_radians = interpolate_value(current_angle_radians, target_angle_radians, alpha)

        measured_position, measured_velocity = bus.write_read_pdo_2(device_id, command_angle_radians, 0.0)
        if measured_position is not None and measured_velocity is not None:
            print(
                f"\rtarget={command_angle_radians:+.4f} rad  "
                f"measured={measured_position:+.4f} rad  "
                f"vel={measured_velocity:+.4f} rad/s",
                end="",
                flush=True,
            )

        if alpha >= 1.0:
            break

        rate.sleep()

    settle_start_time = time.monotonic()
    hold_start_time: float | None = None
    settle_time_seconds: float | None = None
    stable_samples = 0
    hold_max_position_error_radians = 0.0
    hold_max_velocity_radians_per_second = 0.0
    final_position_error_radians: float | None = None
    final_velocity_radians_per_second: float | None = None

    while True:
        measured_position, measured_velocity = bus.write_read_pdo_2(device_id, target_angle_radians, 0.0)
        if measured_position is not None and measured_velocity is not None:
            position_error_radians = abs(measured_position - target_angle_radians)
            velocity_radians_per_second = abs(measured_velocity)
            final_position_error_radians = position_error_radians
            final_velocity_radians_per_second = velocity_radians_per_second

            if sample_within_tolerance(
                position_error_radians,
                velocity_radians_per_second,
                position_tolerance_radians=position_tolerance_radians,
                velocity_tolerance_radians_per_second=velocity_tolerance_radians_per_second,
            ):
                stable_samples += 1
                if stable_samples >= required_stable_samples and settle_time_seconds is None:
                    settle_time_seconds = time.monotonic() - settle_start_time
                    hold_start_time = time.monotonic()
            else:
                stable_samples = 0

            if settle_time_seconds is not None:
                hold_max_position_error_radians = max(
                    hold_max_position_error_radians,
                    position_error_radians,
                )
                hold_max_velocity_radians_per_second = max(
                    hold_max_velocity_radians_per_second,
                    velocity_radians_per_second,
                )

            print(
                f"\rtarget={target_angle_radians:+.4f} rad  "
                f"measured={measured_position:+.4f} rad  "
                f"err={position_error_radians:+.4f} rad  "
                f"vel={measured_velocity:+.4f} rad/s  "
                f"stable={stable_samples}/{required_stable_samples}",
                end="",
                flush=True,
            )

        if settle_time_seconds is None and time.monotonic() - settle_start_time >= settle_timeout_seconds:
            print()
            return ActuatorAngleAssessment(
                stage_name=stage_name,
                target_angle_radians=target_angle_radians,
                settled=False,
                passed=False,
                settle_time_seconds=None,
                hold_max_position_error_radians=hold_max_position_error_radians,
                hold_max_velocity_radians_per_second=hold_max_velocity_radians_per_second,
                final_position_error_radians=final_position_error_radians,
                final_velocity_radians_per_second=final_velocity_radians_per_second,
                failure_reason="failed to settle within the allotted timeout",
            )

        if hold_start_time is not None and time.monotonic() - hold_start_time >= hold_seconds:
            break

        rate.sleep()

    print()

    passed = (
        hold_max_position_error_radians <= position_tolerance_radians
        and hold_max_velocity_radians_per_second <= velocity_tolerance_radians_per_second
    )
    failure_reason = None
    if not passed:
        failure_reason = "hold phase exceeded the configured error or velocity tolerance"

    return ActuatorAngleAssessment(
        stage_name=stage_name,
        target_angle_radians=target_angle_radians,
        settled=True,
        passed=passed,
        settle_time_seconds=settle_time_seconds,
        hold_max_position_error_radians=hold_max_position_error_radians,
        hold_max_velocity_radians_per_second=hold_max_velocity_radians_per_second,
        final_position_error_radians=final_position_error_radians,
        final_velocity_radians_per_second=final_velocity_radians_per_second,
        failure_reason=failure_reason,
    )


def run_actuator_angle_sequence(
    bus: recoil.Bus,
    device_id: int,
    *,
    target_angle_radians: float,
    return_angle_radians: float | None = None,
    cycles: int | None = None,
    position_kp: float = 0.2,
    position_kd: float = 0.005,
    torque_limit: float = 0.2,
    max_speed_radians_per_second: float = math.radians(30.0),
    hold_seconds: float = 2.0,
    control_frequency_hz: float = 200.0,
    position_tolerance_radians: float = math.radians(2.0),
    velocity_tolerance_radians_per_second: float = math.radians(10.0),
    settle_timeout_seconds: float = 2.0,
    required_stable_samples: int = 20,
) -> None:
    if max_speed_radians_per_second <= 0.0:
        raise ValueError("max_speed_radians_per_second must be positive")
    if hold_seconds < 0.0:
        raise ValueError("hold_seconds must be non-negative")
    if control_frequency_hz <= 0.0:
        raise ValueError("control_frequency_hz must be positive")
    if position_tolerance_radians <= 0.0:
        raise ValueError("position_tolerance_radians must be positive")
    if velocity_tolerance_radians_per_second <= 0.0:
        raise ValueError("velocity_tolerance_radians_per_second must be positive")
    if settle_timeout_seconds <= 0.0:
        raise ValueError("settle_timeout_seconds must be positive")
    if required_stable_samples <= 0:
        raise ValueError("required_stable_samples must be positive")

    sequence = build_actuator_angle_sequence(
        target_angle_radians=target_angle_radians,
        return_angle_radians=return_angle_radians,
        cycles=cycles,
    )
    rate = RateLimiter(frequency=control_frequency_hz)

    bus.write_position_kp(device_id, position_kp)
    bus.write_position_kd(device_id, position_kd)
    bus.write_torque_limit(device_id, torque_limit)
    bus.set_mode(device_id, recoil.Mode.POSITION)
    bus.feed(device_id)

    failure: ActuatorAngleAssessment | None = None
    try:
        for stage_name, stage_target_radians in sequence:
            assessment = move_actuator_to_angle(
                bus,
                rate,
                device_id,
                target_angle_radians=stage_target_radians,
                max_speed_radians_per_second=max_speed_radians_per_second,
                hold_seconds=hold_seconds,
                stage_name=stage_name,
                position_tolerance_radians=position_tolerance_radians,
                velocity_tolerance_radians_per_second=velocity_tolerance_radians_per_second,
                settle_timeout_seconds=settle_timeout_seconds,
                required_stable_samples=required_stable_samples,
            )
            print_actuator_angle_assessment(assessment)
            if not assessment.passed:
                failure = assessment
                break
    except KeyboardInterrupt:
        print("\nInterrupted, switching actuator back to idle.")
    finally:
        bus.set_mode(device_id, recoil.Mode.IDLE)

    if failure is not None:
        raise RuntimeError(f"{failure.stage_name} failed: {failure.failure_reason}")


def run_actuator_sine_motion(
    bus: recoil.Bus,
    device_id: int,
    *,
    position_kp: float = 0.2,
    position_kd: float = 0.005,
    torque_limit: float = 0.2,
    motion_frequency_hz: float = 1.0,
    motion_amplitude_radians: float = 1.0,
    control_frequency_hz: float = 200.0,
) -> None:
    rate = RateLimiter(frequency=control_frequency_hz)

    bus.write_position_kp(device_id, position_kp)
    bus.write_position_kd(device_id, position_kd)
    bus.write_torque_limit(device_id, torque_limit)
    bus.set_mode(device_id, recoil.Mode.POSITION)
    bus.feed(device_id)

    try:
        while True:
            target_angle = np.sin(2.0 * np.pi * motion_frequency_hz * time.time()) * motion_amplitude_radians
            measured_position, measured_velocity = bus.write_read_pdo_2(device_id, target_angle, 0.0)
            if measured_position is not None and measured_velocity is not None:
                print(f"Measured pos: {measured_position:.3f}\tvel: {measured_velocity:.3f}")
            rate.sleep()
    finally:
        bus.set_mode(device_id, recoil.Mode.IDLE)
