# Berkeley Humanoid Redev

> 本仓库用于[Berkeley Humanoid Lite](https://github.com/HybridRobotics/Berkeley-Humanoid-Lite)二次开发

## 目录

```text
.
├── apps/
├── artifacts/
├── configs/
├── docs/
├── packages/
│   ├── assets/
│   ├── lowlevel/
│   └── sim/
└── tools/
```

## 安装

```bash
# 基础开发环境
uv sync --all-packages --group dev

# usd 导出
uv sync --all-packages --group dev --group assets-conversion

# lowlevel 运行
uv sync --all-packages --group dev --group lowlevel-runtime

# teleoperation 运行
uv sync --all-packages --group dev --group lowlevel-runtime --group teleoperation
```

## 模块

### assets

- 管理 URDF / MJCF / USD 资源
- 提供资源路径 API
- 提供导出与转换 workflow

```bash
uv run python apps/assets/export_onshape_to_urdf.py
uv run python apps/assets/export_onshape_to_mjcf.py
uv run python apps/assets/convert_urdf_to_usd.py
```

### lowlevel

- 电机连通性、配置读写、单电机调试
- IMU / joystick / locomotion 运行时
- 标定、策略部署、硬件配置管理

```bash
# CAN
bash apps/lowlevel/start_can_transports.sh
bash apps/lowlevel/stop_can_transports.sh

# 基础检查
uv run python apps/lowlevel/check_connection.py
uv run python apps/lowlevel/motor/ping.py --id 1
uv run python apps/lowlevel/test_imu.py
uv run python apps/lowlevel/test_joystick.py

# 标定与运行
uv run python apps/lowlevel/calibrate_joints.py
uv run python apps/lowlevel/run_idle.py --config configs/policies/policy_biped_50hz.yaml
uv run python apps/lowlevel/run_locomotion.py --config configs/policies/policy_biped_50hz.yaml
```

### teleop

- teleoperation 求解
- gripper / idle / 连接检测

```bash
uv run python apps/teleop/check_connection.py
uv run python apps/teleop/run_idle.py
uv run python apps/teleop/run_teleop.py
uv run python apps/teleop/test_solver.py
uv run python apps/teleop/test_gripper.py
```

### sim

- MuJoCo sim2sim
- sim2real 观测流
- Isaac 任务注册
- 训练 workflow 与 checkpoint 管理

```bash
# MuJoCo policy 回放
uv run python apps/sim2sim/play_mujoco.py --config configs/policies/policy_biped_50hz.yaml

# sim2real 观测可视化
uv run python apps/sim2real/visualize.py --config configs/policies/policy_biped_50hz.yaml

# Isaac 任务列表
uv run python apps/list_envs.py

# 训练与回放
uv run python apps/rsl_rl/train.py --task BerkeleyHumanoidLite-Humanoid-v0
uv run python apps/rsl_rl/play.py --task BerkeleyHumanoidLite-Humanoid-v0
```

## 配置与产物

- 策略配置：`configs/policies/`
- 硬件配置：`configs/hardware/`
- 标定产物：`artifacts/calibration/`
- 训练日志与 checkpoint：`artifacts/logs/rsl_rl/`
- 默认部署配置：`configs/policies/policy_latest.yaml`

## 开发命令

```bash
make verify
make lint
make test
make lock
make tree
```

