# Cloud Server Isaac Warning Report Plan

## Goal
- 固化本次云服务器 `Stand` 任务排查的最终根因、证据链和处置结论。
- 区分“排查过程中的中间阻塞”与“最终失败根因”，避免后续继续把 `zenity / Gtk / DRI3 / terrain / proxy` 混在一起排查。
- 保持本轮为“报告沉淀”而不是代码改造，不直接修改训练入口或 Isaac Kit 配置。

## Input Context
- 用户提供了第一阶段启动日志，包含：
  - `simulation_context.py` 的 PhysX 提示
  - `terrain_importer.py` 的 ground plane 材质警告
  - `zenity` 弹窗与大量 `Gtk-WARNING`
  - `libEGL` / `DRI3` 图形环境警告
- 用户随后提供了第二阶段失败栈：
  - `Time taken for scene creation : 505.428592 seconds`
  - `FileNotFoundError: Unable to open the usd file at path: https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Environments/Grid/default_environment.usd`
- 用户进一步定位到 Isaac Lab Kit 文件中的关键配置：
  - `hangDetector.enabled = true`
  - `hangDetector.timeout = 120`
  - `renderer.startupMessageDisplayed = true`
- 用户确认云服务器上的场景材质目录存在：
  - `Shingles_01.mdl`
  - `Shingles_01/Shingles_01_BaseColor.png`
- 用户最终确认：
  - 开启代理后问题恢复

## Verified Repository Findings
- `apps/rsl_rl/train.py` 通过 `AppLauncher.add_app_launcher_args(parser)` 暴露 Isaac Lab 启动参数。
- 本地依赖中的 `AppLauncher` 已确认原生支持 `--kit_args`，并会将其拆分后追加到 `sys.argv`。
- `balance/common.py` 构造的环境继承自 `LocomotionVelocityEnvCfg`，默认将 `decimation` 设为 `8`，与日志中的 `0.005 * 8 = 0.04` 对齐，说明本次更像 `Stand / Recovery` 任务而不是旧 `Velocity` 任务。
- `velocity_env_cfg.py` 使用 `TerrainImporterCfg(terrain_type="plane")`，并通过 `_ground_visual_material_cfg()` 优先选择项目内 `.mdl` 材质。
- Isaac Lab 的 `TerrainImporter` 在 `plane` 模式下会内部构造 `sim_utils.GroundPlaneCfg(...)`。
- `GroundPlaneCfg.usd_path` 默认指向：
  - `f"{ISAAC_NUCLEUS_DIR}/Environments/Grid/default_environment.usd"`
- `ISAAC_NUCLEUS_DIR` 来自：
  - `carb.settings.get_settings().get("/persistent/isaac/asset_root/cloud")`
  这意味着默认 ground plane 依赖的是云端 Isaac 资源根，而不是仓库内本地文件。
- `scene_materials.py` 与对应测试表明 `isaac-shingles-01` 预设期望包含 4 个文件：
  - `Shingles_01.mdl`
  - `Shingles_01/Shingles_01_BaseColor.png`
  - `Shingles_01/Shingles_01_ORM.png`
  - `Shingles_01/Shingles_01_Normal.png`

## Root Cause Summary
- 最终失败根因是 **云服务器网络 / 代理未就绪，导致 Isaac Lab 无法访问远端 `default_environment.usd`**。
- 该远端 USD 是 `plane` 地面场景的默认资源，不在仓库内，也不在当前本地安装中作为离线文件提供。
- 开启代理后问题恢复，说明这次场景创建失败本质上是环境侧网络可达性问题。
- `hangDetector` 触发的 `zenity` 弹窗属于排查过程中的中间阻塞：
  - 它会掩盖真正的资源拉取失败
  - 但不是最终导致 `FileNotFoundError` 的主根因
- `terrain_importer`、`libEGL`、`Gtk` 等告警存在，但都不是本轮最终失败的第一根因。

## Final Resolution
1. 环境侧修复：
   - 为云服务器训练进程开启代理，使其能访问 Isaac 远端资源。
2. 结果：
   - `default_environment.usd` 可访问后，`Stand` 任务恢复正常。
3. 运行建议：
   - 首次冷启动若仍可能超过 `120s`，可以保留：
     - `--kit_args=--/app/hangDetector/enabled=false`
   - 这属于启动稳定性增强项，不是最终根因修复项。
4. 当前不再优先处理：
   - 材质 bundle 完整性
   - `Gtk` / `DRI3` 图形栈噪声
   - 自定义 `.kit` 覆盖
   除非后续再次出现独立问题。

## Validation
- 已验证：
  - 禁用 `hangDetector` 后，可以继续暴露出真正的场景创建错误。
  - 错误明确指向远端 `default_environment.usd` 资源不可达。
  - 用户确认开启代理后问题恢复。
- 未进一步展开：
  - 是否还需要长期保留 `--kit_args=--/app/hangDetector/enabled=false`
  - 是否要把远端 Isaac 资源改成长期离线缓存方案
