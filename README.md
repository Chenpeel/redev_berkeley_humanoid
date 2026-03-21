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

`lowlevel-runtime` 会安装 `python-can`、`setuptools<81`、`pyserial`、`onnxruntime`、`inputs` 等运行依赖；
`calibrate_joints.py`、`run_idle.py`、`run_locomotion.py`、`test_joystick.py` 都需要先同步这一组。

## Make 快捷

```bash
make help
```


目录下 `make` 目标分为两类：

- `verify` / `lint` / `test` 是 Python 工作区检查
- `lowlevel-*` 是 `packages/lowlevel/native` 下的 C++/native 构建与运行快捷入口


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

> 说明
>
> 根目录的 `make lowlevel-*` 目标对应的是 C++/native 版本，
> 即 `packages/lowlevel/native`。
>
> `apps/lowlevel/*.py` 下的脚本仍然是 Python 入口，两者不要混用。

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

> 蓝牙连接手柄
>
> 可以先使用有线连接测试，通过后再进行测试蓝牙连接
>
> 以 类Xbox 为例，测试使用的是PXN，Linux 配对完成后可继续使用上面的 `test_joystick.py` 和 `udp_joystick.py`

```bash
sudo systemctl enable --now bluetooth
bluetoothctl
# 在 bluetoothctl 中依次执行
power on
agent on
default-agent
scan on
# 手柄进入配对模式后，记录扫描到的 MAC 地址
# MAC_Addr="98:B6:EA:18:D8:07" 这是我的手柄的MAC
# 自行通过类似 Xbox Controller 等类似的字眼查找对应的MAC
pair ${MAC_Addr}
trust ${MAC_Addr}
connect ${MAC_Addr}
exit
```

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
make lowlevel-build-udp-test
make lowlevel-run-udp-test
```

###### IMU 测试

> 推荐配置
>
> 对 HiWonder IM10A，建议先切到高波特率并压缩输出内容，
> 避免在 `9600` 下因为输出帧过多导致 `gyro` 帧不稳定。
>
> 下面这条命令会把 IMU 配置为：
>
> - 波特率 `460800`
> - 采样率 `100 Hz`
> - 输出内容仅保留 `angular_velocity + angle + quaternion`
> - `--save` 会把配置保存到设备，掉电后仍然生效

```bash
uv run python apps/lowlevel/configure_imu.py \
  --device /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --baudrate 9600 \
  --set-baudrate 460800 \
  --rate-hz 100 \
  --profile control \
  --save
```

> 上面命令里的 `--baudrate 9600` 指的是“当前设备还在旧串口配置上时，用什么波特率连上去改配置”。
> 如果你的 IMU 已经保存为 `460800`，后续再次调整输出内容时，把这里改成 `--baudrate 460800` 即可。

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
  --baudrate 460800
```

> C++ 
>
> 仅编译 IMU 测试程序

```bash
make lowlevel-build-imu-test
```

> 默认自动检测协议 / 串口 / 波特率

```bash
make lowlevel-run-imu-test
```

> 显式指定 HiWonder IM10A 串口参数

```bash
make lowlevel-run-imu-test IMU_TEST_ARGS="\
  --protocol hiwonder \
  --device /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --baudrate 460800"
```

##### 标定与运行

> C++ / native 主程序
>
> 运行前先确保 `can0` / `can1` 已经拉起。
> 如果只想查看快捷命令，先执行 `make help`。

```bash
make lowlevel-build
make lowlevel-run
```

> 显式指定 native 主程序的 IMU 参数

```bash
make lowlevel-run LOWLEVEL_ARGS="\
  --protocol hiwonder \
  --device /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --baudrate 460800"
```

> Python 标定与运行脚本
>
> 下面这些命令是 Python 入口，不是 `make lowlevel-*` 的封装。

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
