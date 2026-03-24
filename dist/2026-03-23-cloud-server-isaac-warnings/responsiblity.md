# Cloud Server Isaac Warning Responsibility

## Scope Owner
- 主代理负责：
  - 收集日志
  - 归因到仓库代码、Isaac Lab 依赖和云服务器运行环境
  - 在 `dist/2026-03-23-cloud-server-isaac-warnings/` 下维护任务文档
- 用户负责：
  - 在云服务器上执行复现与回归验证
  - 确认云服务器网络 / 代理是否允许访问 Isaac 远端资源
  - 决定是否保留 `--kit_args`、`--headless` 或自定义 `.kit` 方案

## Investigation Boundaries
- 训练入口与参数透传：
  - `apps/rsl_rl/train.py`
  - `.venv/lib/python3.11/site-packages/isaaclab/source/isaaclab/isaaclab/app/app_launcher.py`
- 仿真任务上下文：
  - `packages/sim/src/berkeley_humanoid_lite/tasks/balance/common.py`
  - `packages/sim/src/berkeley_humanoid_lite/tasks/locomotion/velocity/velocity_env_cfg.py`
  - `packages/sim/src/berkeley_humanoid_lite/tasks/registry.py`
- 场景材质路径与预设：
  - `packages/assets/src/berkeley_humanoid_lite_assets/scene_materials.py`
  - `packages/assets/src/berkeley_humanoid_lite_assets/paths.py`
  - `packages/assets/tests/test_scene_materials.py`
- 远端地面资源依赖：
  - `.venv/lib/python3.11/site-packages/isaaclab/source/isaaclab/isaaclab/terrains/terrain_importer.py`
  - `.venv/lib/python3.11/site-packages/isaaclab/source/isaaclab/isaaclab/sim/spawners/from_files/from_files_cfg.py`
  - `.venv/lib/python3.11/site-packages/isaaclab/source/isaaclab/isaaclab/utils/assets.py`

## Environment Boundaries
- 以下内容属于云服务器运行环境，不在仓库代码直接控制范围内：
  - 出站网络是否可访问 `omniverse-content-production.s3-us-west-2.amazonaws.com`
  - 代理是否对训练进程生效
  - `zenity` 弹窗行为
  - `Gtk` 主题解析错误
  - `libEGL` / `DRI3` 图形栈告警
  - Isaac / Omniverse 安装包中的 `.kit` 默认配置
- 这些环境问题会影响资源拉取、启动体验或渲染性能，但不应与任务配置代码混淆。

## Non-goals
- 本轮不直接修改：
  - `apps/rsl_rl/train.py`
  - Isaac Lab / Isaac Sim 安装目录中的 `.kit` 文件
  - 云服务器桌面主题、X11、EGL 或 DRI3 系统配置
- 本轮不继续追查材质 bundle 缺失，因为最终失败根因已由代理问题闭环。
- 本轮不把 `APP_HANG_DETECTOR_ENABLED=false` 当作已验证方案写入主流程。
- 本轮不处理 ground plane 材质的代码级回退逻辑，只记录为后续可选改进项。

## Coordination Notes
- 如果后续要做代码改造，建议拆成三条互不冲突的工作流：
  1. 资源可用性治理
     - 远端 Isaac 资源缓存
     - 代理 / 镜像 / 离线资源文档
  2. 启动链治理
     - 增加项目级 `--kit-arg` 包装或默认 headless 文档
  3. 材质健壮性治理
     - 只在 bundle 完整时启用 MDL
     - 否则自动退回 `PreviewSurfaceCfg`
- 当前优先级已经从 hangDetector 排查切换为“记录代理是最终根因，其他项不继续展开”。
