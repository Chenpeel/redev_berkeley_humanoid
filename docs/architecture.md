# 仓库架构说明

## 目标

当前仓库按职责拆分为清晰的模块边界：

- `assets` 负责资源文件、路径 API 与导出/转换 workflow
- `sim` 负责仿真环境、任务定义、训练 workflow、sim2sim / sim2real 组装
- `lowlevel` 负责低层控制、硬件接口、传感器、teleoperation 和 workflow 组装
- `apps` 只负责调用 `packages/*` 暴露的入口函数

## 边界原则

### 1. `apps` 只做编排

- 可复用逻辑必须放回 `packages/*`
- `apps/lowlevel` 与 `apps/teleop` 保持薄入口
- `apps/list_envs.py`、`apps/rsl_rl/train.py`、`apps/rsl_rl/play.py` 显式注册任务

### 2. 配置和产物单独收口

- 共享策略配置放在 `configs/policies/`
- 硬件相关 JSON 放在 `configs/hardware/`
- checkpoint、日志和标定产物放在 `artifacts/`
- 运行时代码不硬编码特殊目录或工作目录相对路径

## 模块关系

```text
berkeley_humanoid_lite_assets
    ├── 提供 URDF / MJCF / USD 与资源路径 API
    └── 提供导出、后处理与格式转换 workflow

berkeley_humanoid_lite
    ├── 消费 berkeley_humanoid_lite_assets
    ├── 通过 register_tasks() 显式注册仿真任务
    ├── training/        训练、checkpoint、导出和部署配置生成
    ├── streams/         sim2real 观测流适配
    └── workflows/       sim2sim / sim2real 面向 apps 的编排入口

berkeley_humanoid_lite_lowlevel
    ├── actuator/        单电机配置读写、连通性检查与调试动作
    ├── policy/          策略部署配置与推理
    ├── robot/           locomotion / bimanual 运行时、标定、执行器通信、夹爪协议与配置读写
    ├── sensors/         串口姿态流等传感器协议解析
    ├── teleoperation/   teleop 求解器
    └── workflows/       面向 apps 的编排入口

apps/*
    └── 只调用 packages/* 暴露的入口函数
```

## lowlevel 结构

`packages/lowlevel` 继续按职责拆分：

- `actuator/`
  负责单电机配置读写、参数覆盖、连通性检查、标定和运动调试
- `policy/`
  负责部署配置加载与策略推理控制器
- `robot/`
  负责 locomotion / bimanual 机器人的控制状态、标定存储、执行器通信、夹爪协议、运行时和硬件配置读写
- `sensors/`
  负责低层传感器协议解析，例如串口姿态流
- `teleoperation/`
  负责 teleoperation 逆运动学求解器
- `workflows/`
  负责把 `actuator / policy / robot / teleoperation` 编排成稳定的应用入口

## 后续方向

- 继续拆分 `packages/sim/environments/mujoco.py` 中仍偏重的 runtime / control / observation 逻辑
- 增加 sim 与 lowlevel 的纯逻辑单元测试
- 继续统一 Python 与 native/C++ 层术语和命名

## IMU 部署边界

当前项目里和真实硬件对应的官方 USB IMU 基线是 `docs/imu_usb.py`。

- 官方协议：
  - 同步头是 `0x55`
  - 官方示例默认波特率是 `9600`
  - 常见输出帧是 `0x51/0x52/0x53`
  - locomotion 运行链路额外依赖 `0x59 quaternion` 帧
- 仓库里的 `packet` 协议不是这块官方 USB IMU：
  - `packet` 对应 `bno085_arduino.ino`
  - 当前这块官方 IMU 应走 `hiwonder` 路径

### Python Runtime

- 正式入口是 `apps/lowlevel/run_locomotion.py`
- Python locomotion 只支持 `hiwonder`
- 若自动探测到 `packet`，现在会直接报错
- 进入 reset 前会等待：
  - quaternion ready
  - angular velocity ready
- Python HiWonder 解析器已经具备：
  - checksum 校验
  - 坏帧滑窗重同步
  - 基于有效传感器帧的 probe 门槛

### Native Runtime

- native runtime 默认仍支持：
  - `hiwonder`
  - `packet`
- 但当前这块官方 USB IMU 的推荐链路仍然是 `hiwonder`
- native HiWonder 路径现在已经具备：
  - checksum 校验
  - 缓冲区解析与重同步
  - 只接受有效传感器帧的 probe 门槛
  - 启动前等待 quaternion 与 gyro ready
  - 运行中 stale / incomplete IMU 的节流告警
- native 仍保留 `packet` 兼容能力，是为了另一条自定义 IMU 链路，不应和官方 USB IMU 混用

### 运行前检查

对这块官方 HiWonder USB IMU，上线前至少确认：

1. 协议是 `hiwonder`，不是 `packet`
2. 串口路径正确，不要默认假设固定是 `/dev/ttyUSB0`
3. 波特率与设备当前配置一致
4. 输出内容至少包含：
   - angular velocity
   - quaternion
5. 进入 locomotion 前，IMU 已经 ready

### 推荐流程

1. 先用 `apps/lowlevel/test_imu.py` 或 native `test-imu` 路径确认串口、波特率和协议正确。
2. 如果设备还是官方默认输出配置，先用 `apps/lowlevel/configure_imu.py` 切到项目需要的输出内容。
3. 再进入 `run_locomotion.py` 或 native runtime。
4. 运行中如果看到 stale / incomplete IMU 告警，优先检查：
   - 串口连接
   - 波特率
   - IMU 输出配置
   - 设备是否持续输出 `0x59 quaternion`
