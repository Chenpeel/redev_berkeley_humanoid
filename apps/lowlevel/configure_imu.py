import argparse

from berkeley_humanoid_lite_lowlevel.workflows import configure_hiwonder_output
from berkeley_humanoid_lite_lowlevel.workflows.imu import (
    DEFAULT_IMU_BAUDRATE,
    DEFAULT_IMU_PROBE_DURATION,
    DEFAULT_IMU_SERIAL_DEVICE,
    DEFAULT_IMU_TIMEOUT,
    SUPPORTED_HIWONDER_OUTPUT_PROFILES,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="Configure a HiWonder IMU serial output profile.")
    parser.add_argument(
        "--device",
        type=str,
        default=DEFAULT_IMU_SERIAL_DEVICE,
        help="Current serial device path or 'auto' to probe common IMU ports",
    )
    parser.add_argument(
        "--baudrate",
        type=str,
        default=DEFAULT_IMU_BAUDRATE,
        help="Current serial baudrate or 'auto' to probe common values",
    )
    parser.add_argument(
        "--set-baudrate",
        type=int,
        default=460800,
        help="Target HiWonder baudrate to apply before configuring output content",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=10.0,
        help="Target HiWonder sampling rate in Hz",
    )
    parser.add_argument(
        "--profile",
        type=str,
        choices=SUPPORTED_HIWONDER_OUTPUT_PROFILES,
        default="control",
        help="Predefined output profile",
    )
    parser.add_argument("--timeout", type=float, default=DEFAULT_IMU_TIMEOUT, help="Serial read timeout in seconds")
    parser.add_argument(
        "--probe-duration",
        type=float,
        default=DEFAULT_IMU_PROBE_DURATION,
        help="Probe time in seconds for each auto-detection candidate",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Persist the configuration on the device so it survives power cycles",
    )
    parser.add_argument("--time-output", dest="time_output", action="store_true", help="Enable time frames")
    parser.add_argument("--no-time-output", dest="time_output", action="store_false", help="Disable time frames")
    parser.set_defaults(time_output=None)
    parser.add_argument(
        "--acceleration-output",
        dest="acceleration_output",
        action="store_true",
        help="Enable acceleration frames",
    )
    parser.add_argument(
        "--no-acceleration-output",
        dest="acceleration_output",
        action="store_false",
        help="Disable acceleration frames",
    )
    parser.set_defaults(acceleration_output=None)
    parser.add_argument(
        "--angular-velocity-output",
        dest="angular_velocity_output",
        action="store_true",
        help="Enable angular velocity frames",
    )
    parser.add_argument(
        "--no-angular-velocity-output",
        dest="angular_velocity_output",
        action="store_false",
        help="Disable angular velocity frames",
    )
    parser.set_defaults(angular_velocity_output=None)
    parser.add_argument("--angle-output", dest="angle_output", action="store_true", help="Enable angle frames")
    parser.add_argument("--no-angle-output", dest="angle_output", action="store_false", help="Disable angle frames")
    parser.set_defaults(angle_output=None)
    parser.add_argument(
        "--magnetic-field-output",
        dest="magnetic_field_output",
        action="store_true",
        help="Enable magnetic field frames",
    )
    parser.add_argument(
        "--no-magnetic-field-output",
        dest="magnetic_field_output",
        action="store_false",
        help="Disable magnetic field frames",
    )
    parser.set_defaults(magnetic_field_output=None)
    parser.add_argument("--pressure-output", dest="pressure_output", action="store_true", help="Enable pressure frames")
    parser.add_argument(
        "--no-pressure-output",
        dest="pressure_output",
        action="store_false",
        help="Disable pressure frames",
    )
    parser.set_defaults(pressure_output=None)
    parser.add_argument(
        "--quaternion-output",
        dest="quaternion_output",
        action="store_true",
        help="Enable quaternion frames",
    )
    parser.add_argument(
        "--no-quaternion-output",
        dest="quaternion_output",
        action="store_false",
        help="Disable quaternion frames",
    )
    parser.set_defaults(quaternion_output=None)
    arguments = parser.parse_args()

    configure_hiwonder_output(
        device=arguments.device,
        baudrate=arguments.baudrate,
        timeout=arguments.timeout,
        probe_duration=arguments.probe_duration,
        target_baudrate=arguments.set_baudrate,
        rate_hz=arguments.rate_hz,
        profile=arguments.profile,
        time_output=arguments.time_output,
        acceleration_output=arguments.acceleration_output,
        angular_velocity_output=arguments.angular_velocity_output,
        angle_output=arguments.angle_output,
        magnetic_field_output=arguments.magnetic_field_output,
        pressure_output=arguments.pressure_output,
        quaternion_output=arguments.quaternion_output,
        save=arguments.save,
    )


if __name__ == "__main__":
    main()
