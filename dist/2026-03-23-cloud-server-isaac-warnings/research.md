# Cloud Server Isaac Warning Research

## One-Line Conclusion
- 这次 `Stand` 任务最终失败的主根因不是任务代码死锁，也不是材质 warning，而是 **云服务器未通过代理访问到 Isaac 远端 ground plane 资源 `default_environment.usd`**；开启代理后问题恢复。`hangDetector` 只是排查过程中先暴露出来的中间阻塞。

## Code Path Mapping

### Training Entry
- `apps/rsl_rl/train.py`
  - 创建 `argparse` 解析器
  - 调用 `AppLauncher.add_app_launcher_args(parser)`
  - 再实例化 `AppLauncher(args_cli)`

### AppLauncher Support
- `.venv/lib/python3.11/site-packages/isaaclab/source/isaaclab/isaaclab/app/app_launcher.py`
  - `add_app_launcher_args()` 明确定义了 `--kit_args`
  - `_resolve_kit_args()` 会把该字符串按空格切分后加入 `sys.argv`
- 结论：
  - 当前项目的训练入口不需要额外改造，就能直接透传 `--kit_args`

### Ground Plane Remote Dependency
- `.venv/lib/python3.11/site-packages/isaaclab/source/isaaclab/isaaclab/terrains/terrain_importer.py`
  - `plane` 地形会调用 `import_ground_plane()`
- `.venv/lib/python3.11/site-packages/isaaclab/source/isaaclab/isaaclab/sim/spawners/from_files/from_files_cfg.py`
  - `GroundPlaneCfg.usd_path` 默认值是：
    - `f"{ISAAC_NUCLEUS_DIR}/Environments/Grid/default_environment.usd"`
- `.venv/lib/python3.11/site-packages/isaaclab/source/isaaclab/isaaclab/utils/assets.py`
  - `ISAAC_NUCLEUS_DIR` 来自：
    - `/persistent/isaac/asset_root/cloud`
- 结论：
  - 当前 `plane` 地面默认依赖远端 Isaac 资源根
  - 不是仓库内的本地 USD 文件
  - 网络 / 代理不通时会在场景创建阶段直接失败

## What The Final Stack Proved
- 第二阶段日志中已经出现：
  - `Time taken for scene creation : 505.428592 seconds`
  - `FileNotFoundError: Unable to open the usd file at path: https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Environments/Grid/default_environment.usd`
- 这说明真实失败点在：
  - `TerrainImporter -> GroundPlaneCfg -> spawn_ground_plane -> create_prim -> add_usd_reference`
- 因为目标 USD 不可访问，所以环境在场景创建阶段失败，而不是训练逻辑失败。

## Why `hangDetector` Appeared First
- 在真实 `FileNotFoundError` 暴露之前，GUI Kit 首次冷启动已经可能超过 `120s`。
- 这会先触发：
  - `hangDetector.enabled = true`
  - `hangDetector.timeout = 120`
- 从而拉起 `zenity` 弹窗并伴随大量 `Gtk` 输出。
- 因此排查顺序上先看到的是“挂起提示”，后看到的才是“远端 USD 打不开”。
- 这也是为什么 `hangDetector` 属于中间阻塞，而不是最终根因。

## Why `zenity` / `Gtk` Appeared
- `zenity` 不是仓库代码里显式调用的工具。
- 它更像是 Kit hangDetector 触发后的 GUI 提示载体。
- 这也解释了为什么会伴随大量：
  - `Gtk-WARNING`
  - 主题解析错误
  - `--icon-name is deprecated`
- 这些都属于弹窗进程和远程桌面主题栈的副产物，而不是训练逻辑本身。

## Why `terrain_importer` Warning Is Not The Main Blocker
- 仓库的 ground plane 场景确实来自：
  - `TerrainImporterCfg(terrain_type="plane")`
- 视觉材质也确实来自项目内 `_ground_visual_material_cfg()`。
- 但这条 warning 的语义更像：
  - ground plane 拿不到可解析的 diffuse color，退回默认黑色
- 它会影响场景外观，不足以解释最终的远端 USD 访问失败。

## Material Bundle Observation
- 用户云服务器上目前可见：
  - `Shingles_01.mdl`
  - `Shingles_01/Shingles_01_BaseColor.png`
- 但仓库测试预设约定的完整 bundle 还包括：
  - `Shingles_01/Shingles_01_ORM.png`
  - `Shingles_01/Shingles_01_Normal.png`
- 这意味着当前服务器材质状态可能是：
  - 预设下载中断
  - 手动拷贝不完整
  - 或历史遗留的半成品目录
- 这个问题值得修，但它不是本轮主线；在用户确认“开启代理后问题恢复”之后，它不再影响本次结论。

## Why Proxy Fixed The Issue
- 当前 ground plane 的默认 USD 路径是远端 Isaac 资源地址。
- 当云服务器代理未生效时：
  - 远端资源不可访问
  - 场景创建卡住并最终报 `FileNotFoundError`
- 当用户开启代理后：
  - 远端 `default_environment.usd` 可访问
  - 问题恢复
- 这形成了完整闭环，因此可以把“代理未配置”确认为最终根因。

## Recommended Remediation Order
1. 先保证训练进程具备访问远端 Isaac 资源的网络 / 代理条件。
2. 如果首次冷启动仍会触发 GUI 挂起弹窗，再加：
   - `--kit_args=--/app/hangDetector/enabled=false`
3. 如果训练不依赖 GUI，可再考虑：
   - `--headless`
4. 材质 bundle、`Gtk` 主题、`DRI3` 仅在后续再次出现独立问题时再处理。

## Practical Commands
- 当前已验证主线是：

```bash
uv run python apps/rsl_rl/train.py \
  --task Stand-Berkeley-Humanoid-Lite-v0 \
  --kit_args=--/app/hangDetector/enabled=false
```

- 前提：
  - 训练进程所在 shell 已正确启用代理

- 如果不需要 GUI：

```bash
uv run python apps/rsl_rl/train.py \
  --headless \
  --task Stand-Berkeley-Humanoid-Lite-v0
```

## Confidence Notes
- 已验证：
  - `--kit_args` 支持链路
  - GUI Kit 默认 hangDetector 配置
  - `plane` ground 默认依赖远端 `default_environment.usd`
  - 用户确认开启代理后问题恢复
- 当前结论：
  - 代理 / 网络是最终根因
  - `hangDetector` 是中间阻塞
