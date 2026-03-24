# Recovery Validation Status Plan

## Goal
- 评估当前 `Recovery-Berkeley-Humanoid-Lite-*` 任务的静态验证是否已经完成。
- 明确在“保持 `stand` 干净、把抗扰放在 `recovery`”前提下，现有实现已经具备什么、还缺什么、下一步需要什么。
- 以 `dist/2026-03-23-stand-balance-getup/` 的文档风格为模板，输出可持续追踪的任务记录。

## Scope
- 只评估 `packages/sim` 中 recovery 任务的：
  - 任务配置
  - 任务注册
  - README 声明
  - 自动化静态验证
- 不改动训练代码。
- 不做新的 recovery 训练实验。
- 不涉及 lowlevel runtime、跌倒检测或 get-up 实现。

## Current Question
- `stand` 已经有很好的训练表现，决定保持为“干净的静态站立基线”。
- 需要确认：
  - 当前 `recovery` 是否已经完成静态验证
  - 当前 `recovery` 是否已经具备随机推扰
  - 在进入下一阶段训练前，还缺哪些验证项和工程准备

## Validation Plan
1. 检查 recovery 任务配置：
   - humanoid / biped 是否都具备 `disturbance_profile`
   - 扰动是 reset 触发还是 interval 触发
2. 检查 recovery 任务是否已经接入：
   - task registry
   - README 训练命令
   - runner experiment name
3. 运行现有静态验证：
   - `python3 -m compileall`
   - `uv run ruff check`
   - `env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest`
4. 判断哪些验证已经覆盖，哪些仍缺失。

## Expected Output
- 一份结论清楚的 status stack：
  - 当前 recovery 静态验证完成到哪一层
  - 缺失的自动化测试
  - 缺失的训练前准备项
  - 是否建议直接进入 recovery 阶段训练
