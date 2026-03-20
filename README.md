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
```

## 安装

要求 Python `3.11`。

```bash
# 默认开发环境（不含 Isaac 训练依赖）
uv sync --all-packages --group dev

# Isaac 训练 / Isaac Sim 相关工具
uv sync --all-packages --group dev --group training

# usd 导出
uv sync --all-packages --group dev --group assets-conversion

# lowlevel 运行
uv sync --all-packages --group dev --group lowlevel-runtime

# teleoperation 运行
uv sync --all-packages --group dev --group lowlevel-runtime --group teleoperation
```


## 模块指引

### assets

- 管理 URDF / MJCF / USD 资源
- 提供资源路径 API
- 提供导出与转换 workflow

```bash
uv run python apps/assets/export_onshape_to_urdf.py
uv run python apps/assets/export_onshape_to_mjcf.py
uv run python apps/assets/convert_urdf_to_usd.py
uv run python apps/assets/prefetch_scene_material.py --preset isaac-shingles-01 --preset-version 5.1
uv run python apps/assets/prefetch_scene_material.py --source /path/to/ground_surface.mdl
```

其中 `apps/assets/convert_urdf_to_usd.py` 需要先安装 `training` 组。

Isaac 场景会优先使用 `packages/assets/src/berkeley_humanoid_lite_assets/data/scenes/default/materials/` 下可用的 `.mdl` 材质文件；`--preset isaac-shingles-01 --preset-version 5.1` 会下载 `Shingles_01.mdl` 及配套贴图，不存在时回退到本地 preview 材质。

### lowlevel

- 电机连通性、配置读写、单电机调试
- IMU / joystick / locomotion 运行时
- 标定、策略部署、硬件配置管理

#####  CAN

> 扫描CAN

```bash
bash apps/lowlevel/start_can_transports.sh
bash apps/lowlevel/stop_can_transports.sh
```

##### 基础检查
```bash
uv run python apps/lowlevel/check_connection.py
uv run python apps/lowlevel/motor/ping.py --id 1
uv run python apps/lowlevel/test_joystick.py
```

###### 手柄测试

> Python
>
> 本机读取手柄并打印解析后的 locomotion 命令

```bash
uv run python apps/lowlevel/test_joystick.py
```

> Python -> native runtime
>
> 将手柄命令按 C++ runtime 约定的 `10011` UDP 协议广播到本机

```bash
uv run python apps/lowlevel/udp_joystick.py
```

> native UDP 收包调试
>
> 仅验证 C++ 侧是否收到 joystick UDP 数据包

```bash
cmake -S packages/lowlevel/native -B build/lowlevel/native
cmake --build build/lowlevel/native --target test-udp -j
./build/lowlevel/native/test-udp
```

###### IMU 测试

> Python 
>
> 默认自动检测协议 / 串口 / 波特率

```bash
uv run python apps/lowlevel/test_imu.py
```

> 显式指定 HiWonder IM10A 串口参数

```bash
uv run python apps/lowlevel/test_imu.py \
  --protocol hiwonder \
  --device /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --baudrate 9600
```

> C++ 
>
> 仅编译 IMU 测试程序

```bash
cmake -S packages/lowlevel/native -B build/lowlevel/native
cmake --build build/lowlevel/native --target test-imu -j
```

> 默认自动检测协议 / 串口 / 波特率

```bash
./build/lowlevel/native/test-imu
```

> 显式指定 HiWonder IM10A 串口参数

```bash
./build/lowlevel/native/test-imu \
  --protocol hiwonder \
  --device /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --baudrate 9600
```

##### 标定与运行

```bash
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

### Sim

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
uv run python apps/rsl_rl/train.py --task Velocity-Berkeley-Humanoid-Lite-v0 --headless
uv run python apps/rsl_rl/play.py --task Velocity-Berkeley-Humanoid-Lite-v0 --experiment_name humanoid --load_run '.*' --checkpoint 'model_.*\.pt' --headless
```

#### 训练

运行前先安装 Isaac 训练依赖，可训练任务：

- `Velocity-Berkeley-Humanoid-Lite-v0`
- `Velocity-Berkeley-Humanoid-Lite-Biped-v0`

从零训练：

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

可选参数：

- `--num_envs`
- `--max_iterations`
- `--seed`
- `--run_name`
- `--experiment_name`
- `--video`

训练产物位置：

- 训练根目录：`artifacts/untested_ckpts/rsl_rl/`
- humanoid 默认实验目录：`artifacts/untested_ckpts/rsl_rl/humanoid/`
- biped 默认实验目录：`artifacts/untested_ckpts/rsl_rl/biped/`

回放并导出部署模型：

```bash
uv run python apps/rsl_rl/play.py \
  --task Velocity-Berkeley-Humanoid-Lite-v0 \
  --experiment_name humanoid \
  --load_run '.*' \
  --checkpoint 'model_.*\.pt' \
  --headless
```

`play.py` 会做两件事：

- 从匹配到的最新 run 和最新 `model_*.pt` checkpoint 回放策略
- 在该 run 目录下导出 `exported/policy.pt` 和 `exported/policy.onnx`

同时会更新默认部署配置：

- `configs/policies/policy_latest.yaml`


## 配置与产物

- 策略配置：`configs/policies/`
- 硬件配置：`configs/hardware/`
- 标定产物：`artifacts/calibration/`
- 训练日志与 checkpoint：`artifacts/untested_ckpts/rsl_rl/`
- 默认部署配置：`configs/policies/policy_latest.yaml`


## 开发命令

```bash
make verify
make lint
make test
make lock
make tree
```
