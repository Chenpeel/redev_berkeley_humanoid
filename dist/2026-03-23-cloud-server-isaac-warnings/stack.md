# Cloud Server Isaac Warning Stack

## Status
- 日期：2026-03-23
- 当前阶段：已完成最终根因归档，用户已确认开启代理后问题恢复。
- 本轮未修改任何仓库代码，也未改动 Isaac 安装目录中的 `.kit` 文件。

## Primary Root Cause
- 最终失败根因是云服务器网络 / 代理未就绪，导致 Isaac Lab 在创建 `plane` 地面时无法访问远端 `default_environment.usd`。
- 报错栈已经明确指向：
  - `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Environments/Grid/default_environment.usd`
- 失败发生在场景创建阶段，日志显示：
  - `Time taken for scene creation : 505.428592 seconds`
- 用户确认开启代理后问题恢复，说明这次失败本质上是环境侧远端资源可达性问题。

## Verified Evidence
- `TerrainImporter` 在 `plane` 模式下会调用 `import_ground_plane()`。
- `import_ground_plane()` 内部构造的是 `sim_utils.GroundPlaneCfg(...)`。
- `GroundPlaneCfg.usd_path` 默认值是：
  - `f"{ISAAC_NUCLEUS_DIR}/Environments/Grid/default_environment.usd"`
- `ISAAC_NUCLEUS_DIR` 来自：
  - `/persistent/isaac/asset_root/cloud`
- 当前环境里该资源最终解析到了远端 S3 URL，而不是仓库本地文件。
- 本地检索没有找到离线的 `default_environment.usd` 副本，因此只要网络不可达，就会在这一层失败。

## Launch Path Findings
- `apps/rsl_rl/train.py` 调用了 `AppLauncher.add_app_launcher_args(parser)`。
- `AppLauncher` 已确认原生暴露：
  - `--kit_args`
- `AppLauncher._resolve_kit_args()` 会把该参数按空格拆分并追加到 `sys.argv`，因此：
  - `--kit_args=--/app/hangDetector/enabled=false`
  是当前可用的启动稳定性增强方式。
- 在当前本地依赖检索中，没有发现 `APP_HANG_DETECTOR_ENABLED` 这个环境变量名称，因此它只能算未验证备选项，不能作为主结论。
- 这条 `kit_args` 方案能解决的是首次冷启动时的 `hangDetector` 弹窗问题，不能替代网络 / 代理修复。

## Task Context Findings
- 日志中的：
  - `Physics step-size     : 0.005`
  - `Environment step-size : 0.04`
- 与 `balance/common.py` 中 `profile.decimation = 8` 的环境构造逻辑对齐，说明这次更像 `Stand` 或 `Recovery` 任务，而不是旧 `Velocity` 任务。
- 这一点与用户口头描述的 `Stand-Berkeley-Humanoid-Lite-v0` 一致。

## Investigation Timeline
- 第一阶段：
  - 首次冷启动超过 `120s`
  - `hangDetector` 触发 `zenity` 弹窗
  - 训练被 GUI 提示阻塞，真正错误还没有暴露
- 第二阶段：
  - 通过 `--kit_args=--/app/hangDetector/enabled=false` 绕过挂起弹窗
  - 暴露出真实错误：远端 `default_environment.usd` 无法打开
- 第三阶段：
  - 用户开启代理
  - 问题恢复

## Secondary Findings
- `hangDetector`
  - 性质：排查过程中的中间阻塞，不是最终失败根因。
  - 适合作为首次冷启动时的稳定性增强项保留。
- `simulation_context.py` 的 PhysX warning
  - 性质：提示型，不是本轮阻塞根因。
  - 当前 `Stand / Recovery` 任务链路本身更接近 `push_by_setting_velocity`，不是直接依赖 reset 外力事件来解释这次失败。
- `terrain_importer.py` 的 ground plane warning
  - 性质：视觉材质告警，不是最终失败根因。
  - 仓库代码会优先选择项目内 `.mdl` 材质。
  - 用户云服务器当前只确认存在：
    - `Shingles_01.mdl`
    - `Shingles_01/Shingles_01_BaseColor.png`
  - 但测试约定的完整预设 bundle 还应包含：
    - `Shingles_01/Shingles_01_ORM.png`
    - `Shingles_01/Shingles_01_Normal.png`
  - 这说明云服务器上的材质预设很可能是不完整的，但本轮不继续展开。
- `libEGL` / `DRI3` / `Gtk`
  - 性质：云服务器图形栈与主题栈噪声。
  - 与仓库代码没有直接字面量调用关系。
  - 可能影响 GUI 体验或渲染加速，但不是当前训练失败的第一原因。

## Decisions
- 最终结论：
  - 代理 / 网络是当前问题的主根因。
- 当前已闭环的修复动作：
  - 为云服务器训练进程开启代理。
- 可选保留的增强项：
  - `uv run python apps/rsl_rl/train.py --task Stand-Berkeley-Humanoid-Lite-v0 --kit_args=--/app/hangDetector/enabled=false`
- 如果任务不需要 GUI，可选替代方案：
  - `uv run python apps/rsl_rl/train.py --headless --task Stand-Berkeley-Humanoid-Lite-v0`
- 暂不把 `APP_HANG_DETECTOR_ENABLED=false` 写成首选方案，因为在当前依赖检索中没有验证到对应配置入口。
- 将材质 bundle 不完整与远程 GUI 栈警告降级为已记录但不继续展开的后续项。

## Resolution
- 用户确认：
  - 开启代理后问题恢复。
- 因此本任务以“环境侧代理未配置，导致远端 Isaac 资源不可达”结案。

## Residual Notes
- 是否长期保留 `--kit_args=--/app/hangDetector/enabled=false`，取决于后续冷启动是否仍会遇到 GUI 挂起弹窗。
- 如果未来希望完全离线运行，需要额外解决远端 Isaac 资源的本地缓存或镜像问题。
