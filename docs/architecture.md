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

