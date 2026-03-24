# Berkeley Humanoid Redev

[中文](README.md) | English

This repository is a redevelopment workspace built on top of
[Berkeley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite).
It focuses on refactoring, normalization, modularization, engineering cleanup,
and secondary development for simulation, training, deployment, and hardware workflows.

## Directory

```text
.
├── apps/
├── artifacts/
├── configs/
├── docs/
└── packages/
    ├── assets/
    ├── lowlevel/
    └── sim/
```

## Installation

```bash
git clone https://github.com/Chenpeel/redev_berkeley_humanoid.git
```

Python `3.11` is required.

```bash
# Default development environment (without Isaac training dependencies)
uv sync --all-packages --group dev

# Isaac training / Isaac Sim related tools
uv sync --all-packages --group dev --group training

# USD export
uv sync --all-packages --group dev --group assets-conversion

# Low-level runtime
uv sync --all-packages --group dev --group lowlevel-runtime

# Teleoperation runtime
uv sync --all-packages --group dev --group lowlevel-runtime --group teleoperation
```

The `lowlevel-runtime` group installs runtime dependencies such as
`python-can`, `setuptools<81`, `pyserial`, `onnxruntime`, and `inputs`.
Make sure this group is installed before running
`calibrate_joints.py`, `run_idle.py`, `run_locomotion.py`, or `test_joystick.py`.

## Make Shortcuts

```bash
make help
```

The root-level `make` targets are split into two groups:

- `verify` / `lint` / `test` for Python workspace checks
- `lowlevel-*` for C++/native build and runtime shortcuts under `packages/lowlevel/native`

## Module Guide

### assets

- Manage URDF / MJCF / USD resources
- Provide resource path APIs
- Provide export and conversion workflows

```bash
uv run python apps/assets/export_onshape_to_urdf.py
uv run python apps/assets/export_onshape_to_mjcf.py
uv run python apps/assets/convert_urdf_to_usd.py
uv run python apps/assets/prefetch_scene_material.py --preset isaac-shingles-01 --preset-version 5.1
uv run python apps/assets/prefetch_scene_material.py --source /path/to/ground_surface.mdl
```

`apps/assets/convert_urdf_to_usd.py` requires the `training` dependency group.

Isaac scenes prefer `.mdl` materials under
`packages/assets/src/berkeley_humanoid_lite_assets/data/scenes/default/materials/`.
The preset
`--preset isaac-shingles-01 --preset-version 5.1`
downloads `Shingles_01.mdl` and related textures.
If the preset material is unavailable, the workflow falls back to a local preview material.

### lowlevel

- Motor connectivity checks, configuration read/write, and single-actuator debugging
- IMU / joystick / locomotion runtime
- Calibration, policy deployment, and hardware configuration management

> Note
>
> Root-level `make lowlevel-*` targets refer to the C++/native implementation,
> i.e. `packages/lowlevel/native`.
>
> Scripts under `apps/lowlevel/*.py` are still Python entry points.
> Do not confuse the two paths.

##### CAN

> Scan CAN transports

```bash
bash apps/lowlevel/start_can_transports.sh
bash apps/lowlevel/stop_can_transports.sh
```

##### Basic Checks

```bash
uv run python apps/lowlevel/check_connection.py
uv run python apps/lowlevel/motor/ping.py --id 1
uv run python apps/lowlevel/test_joystick.py
```

###### Gamepad Test

> Connect a Bluetooth gamepad
>
> You can start with a wired connection first.
> After wired testing succeeds, switch to Bluetooth if needed.
>
> Using an Xbox-like controller as an example:
> the current setup was tested with a PXN controller.
> Once Linux pairing succeeds, you can continue using
> `test_joystick.py` and `udp_joystick.py`.

```bash
sudo systemctl enable --now bluetooth
bluetoothctl
# Run the following commands inside bluetoothctl
power on
agent on
default-agent
scan on
# After the controller enters pairing mode, record the scanned MAC address
# MAC_Addr="98:B6:EA:18:D8:07" is an example from one controller
# Find the correct MAC address from names such as Xbox Controller, etc.
pair ${MAC_Addr}
trust ${MAC_Addr}
connect ${MAC_Addr}
exit
```

> Python
>
> Read the local gamepad and print the decoded locomotion command

```bash
uv run python apps/lowlevel/test_joystick.py
```

> Python -> native runtime
>
> Broadcast joystick commands to localhost using the `10011` UDP protocol
> expected by the C++ runtime

```bash
uv run python apps/lowlevel/udp_joystick.py
```

> Native UDP receive debug
>
> Only verify whether the C++ side receives joystick UDP packets

```bash
make lowlevel-build-udp-test
make lowlevel-run-udp-test
```

###### IMU Test

> Recommended setup
>
> For the HiWonder IM10A, it is recommended to switch to a higher baud rate
> and reduce output content first, so the `gyro` frame remains stable instead
> of being flooded at `9600`.
>
> The following command configures the IMU as:
>
> - baud rate `460800`
> - sampling rate `100 Hz`
> - output limited to `angular_velocity + angle + quaternion`
> - `--save` persists the configuration on the device

```bash
uv run python apps/lowlevel/configure_imu.py \
  --device /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --baudrate 9600 \
  --set-baudrate 460800 \
  --rate-hz 100 \
  --profile control \
  --save
```

> In the command above, `--baudrate 9600` means:
> use the current device baud rate to connect first and then change settings.
> If your IMU has already been saved as `460800`,
> use `--baudrate 460800` for later adjustments.

> Python
>
> Automatically detect protocol / serial device / baud rate

```bash
uv run python apps/lowlevel/test_imu.py
```

> Explicit HiWonder IM10A serial parameters

```bash
uv run python apps/lowlevel/test_imu.py \
  --protocol hiwonder \
  --device /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --baudrate 460800
```

> C++
>
> Build only the IMU test binary

```bash
make lowlevel-build-imu-test
```

> Automatically detect protocol / serial device / baud rate

```bash
make lowlevel-run-imu-test
```

> Explicit HiWonder IM10A serial parameters

```bash
make lowlevel-run-imu-test IMU_TEST_ARGS="\
  --protocol hiwonder \
  --device /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --baudrate 460800"
```

##### Calibration and Runtime

> C++ / native main program
>
> Make sure the actual CAN interfaces are already up before running.
> The default lower-body mapping is `can0` / `can1`.
> If you use the official lower-body wiring with `can2` / `can3`,
> pass the bus names explicitly.
> Run `make help` first if you only want to inspect available shortcuts.

```bash
make lowlevel-build
make lowlevel-run
```

> Explicit IMU parameters for the native main program

```bash
make lowlevel-run LOWLEVEL_ARGS="\
  --protocol hiwonder \
  --device /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --baudrate 460800"
```

> Explicit lower-body buses for official lower-body wiring

```bash
make lowlevel-run LOWLEVEL_ARGS="\
  --left-leg-bus can2 \
  --right-leg-bus can3 \
  --protocol hiwonder \
  --device /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --baudrate 460800"
```

> Python calibration and runtime scripts
>
> The following commands are Python entry points,
> not wrappers around `make lowlevel-*`.

```bash
uv run python apps/lowlevel/calibrate_joints.py
uv run python apps/lowlevel/run_idle.py --config configs/policies/policy_biped_50hz.yaml
uv run python apps/lowlevel/run_locomotion.py --config configs/policies/policy_biped_50hz.yaml
```

> Explicit lower-body bus names for the Python entry points

```bash
uv run python apps/lowlevel/check_connection.py --left-leg-bus can2 --right-leg-bus can3
uv run python apps/lowlevel/calibrate_joints.py --left-leg-bus can2 --right-leg-bus can3
uv run python apps/lowlevel/run_idle.py --config configs/policies/policy_biped_50hz.yaml --left-leg-bus can2 --right-leg-bus can3
uv run python apps/lowlevel/run_locomotion.py --config configs/policies/policy_biped_50hz.yaml --left-leg-bus can2 --right-leg-bus can3
```

### teleop

- Teleoperation solving
- Gripper / idle / connectivity checks

```bash
uv run python apps/teleop/check_connection.py
uv run python apps/teleop/run_idle.py
uv run python apps/teleop/run_teleop.py
uv run python apps/teleop/test_solver.py
uv run python apps/teleop/test_gripper.py
```

### Sim

- MuJoCo sim2sim
- sim2real observation streaming
- Isaac task registration
- Training workflow and checkpoint management

```bash
# MuJoCo policy playback
uv run python apps/sim2sim/play_mujoco.py --config configs/policies/policy_biped_50hz.yaml

# sim2real observation visualization
uv run python apps/sim2real/visualize.py --config configs/policies/policy_biped_50hz.yaml

# Isaac task list
uv run python apps/list_envs.py

# Train and play
uv run python apps/rsl_rl/train.py --task Velocity-Berkeley-Humanoid-Lite-v0 --headless
uv run python apps/rsl_rl/play.py --task Velocity-Berkeley-Humanoid-Lite-v0 --experiment_name humanoid --load_run '.*' --checkpoint 'model_.*\.pt' --headless
```

#### Training

Install Isaac training dependencies before running training.
Available tasks:

- `Velocity-Berkeley-Humanoid-Lite-v0`
- `Velocity-Berkeley-Humanoid-Lite-Biped-v0`
- `Stand-Berkeley-Humanoid-Lite-v0`
- `Stand-Berkeley-Humanoid-Lite-Biped-v0`
- `Recovery-Berkeley-Humanoid-Lite-v0`
- `Recovery-Berkeley-Humanoid-Lite-Biped-v0`

The newly added balance tasks are split as follows:

- `Stand-*` is used for zero-velocity standing stability training.
  It reuses the locomotion observation/action layout and adds body-contact termination.
- `Recovery-*` extends the stand task with interval push disturbances and a wider reset distribution for push-recovery training.
- At the moment, only the Isaac RL side has been added:
  task registration, env cfg, PPO runner cfg, and registry tests.
  `Getup-*` and low-level automatic state switching are not implemented yet.

Train from scratch:

```bash
# humanoid
uv run python apps/rsl_rl/train.py \
  --task Velocity-Berkeley-Humanoid-Lite-v0 \
  --headless

# biped
uv run python apps/rsl_rl/train.py \
  --task Velocity-Berkeley-Humanoid-Lite-Biped-v0 \
  --headless
```

##### Balance Task Training

```bash
# stand humanoid
uv run python apps/rsl_rl/train.py \
  --task Stand-Berkeley-Humanoid-Lite-v0 \
  --headless

# stand biped
uv run python apps/rsl_rl/train.py \
  --task Stand-Berkeley-Humanoid-Lite-Biped-v0 \
  --headless

# recovery humanoid
uv run python apps/rsl_rl/train.py \
  --task Recovery-Berkeley-Humanoid-Lite-v0 \
  --headless

# recovery biped
uv run python apps/rsl_rl/train.py \
  --task Recovery-Berkeley-Humanoid-Lite-Biped-v0 \
  --headless
```

Optional arguments:

- `--num_envs`
- `--max_iterations`
- `--seed`
- `--run_name`
- `--experiment_name`
- `--video`

Training artifact locations:

- Training root: `artifacts/untested_ckpts/rsl_rl/`
- Default humanoid experiment: `artifacts/untested_ckpts/rsl_rl/humanoid/`
- Default biped experiment: `artifacts/untested_ckpts/rsl_rl/biped/`
- Default stand humanoid experiment: `artifacts/untested_ckpts/rsl_rl/stand_humanoid/`
- Default stand biped experiment: `artifacts/untested_ckpts/rsl_rl/stand_biped/`
- Default recovery humanoid experiment: `artifacts/untested_ckpts/rsl_rl/recovery_humanoid/`
- Default recovery biped experiment: `artifacts/untested_ckpts/rsl_rl/recovery_biped/`

Playback and export deployment models:

```bash
uv run python apps/rsl_rl/play.py \
  --task Velocity-Berkeley-Humanoid-Lite-v0 \
  --experiment_name humanoid \
  --load_run '.*' \
  --checkpoint 'model_.*\.pt' \
  --headless
```

`play.py` does two things:

- play back the latest run and latest matching `model_*.pt` checkpoint
- export `exported/policy.pt` and `exported/policy.onnx` under the selected run directory

To play stand or recovery tasks, replace `--task` with the corresponding
`Stand-*` or `Recovery-*` task name.

It also updates the default deployment config:

- `configs/policies/policy_latest.yaml`

## Configs and Artifacts

- Policy configs: `configs/policies/`
- Hardware configs: `configs/hardware/`
- Calibration artifacts: `artifacts/calibration/`
- Training logs and checkpoints: `artifacts/untested_ckpts/rsl_rl/`
- Default deployment config: `configs/policies/policy_latest.yaml`

## Acknowledgement and Upstream Note

This repository is a refactored and redeveloped workspace built on top of the upstream
[Berkeley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite)
project.

Within this workspace, we reused publicly released upstream content and carried out
substantial engineering-oriented redevelopment work on top of it, including
modular restructuring, interface normalization, workflow cleanup, packaging,
configuration organization, and additional functionality.
The reused scope includes, but is not limited to:

- robot assets and description files, including `assets`, meshes, URDF, MJCF, USD, and related scene content
- simulation and training tasks, workflows, and supporting scripts
- low-level control, deployment paths, and related utility code

The original robot design, assets, paper, open-source release, and foundational implementation
belong to the original authors and their team.
This repository is a redevelopment workspace for specific engineering needs,
and it does not replace the upstream project itself.

If you use this repository for research, development, or redistribution,
please acknowledge the upstream Berkeley Humanoid Lite project as well and
follow the corresponding license terms for code and assets.
