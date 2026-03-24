# Recovery Validation Status Responsibility

## Scope Split
- 主代理负责：
  - 检查 recovery 配置、注册、README 和测试覆盖
  - 执行静态验证命令
  - 将现状与缺口写入 `dist/2026-03-23-recovery-validation-status/`
- 用户负责：
  - 决定何时结束当前 `stand` 训练
  - 决定是否在下一轮用 `stand` checkpoint 作为 `recovery` warm-start
  - 后续执行 recovery 训练与结果验收

## Investigation Boundaries
- Recovery 任务实现：
  - `packages/sim/src/berkeley_humanoid_lite/tasks/balance/recovery/config/humanoid/env_cfg.py`
  - `packages/sim/src/berkeley_humanoid_lite/tasks/balance/recovery/config/biped/env_cfg.py`
  - `packages/sim/src/berkeley_humanoid_lite/tasks/balance/common.py`
- 任务注册与训练入口：
  - `packages/sim/src/berkeley_humanoid_lite/tasks/registry.py`
  - `packages/sim/src/berkeley_humanoid_lite/training/arguments.py`
  - `packages/sim/src/berkeley_humanoid_lite/training/checkpoints.py`
- 当前自动化测试：
  - `packages/sim/tests/test_task_registry.py`
  - `packages/sim/tests/test_workflows.py`
- 外部公开说明：
  - `README.md`

## Non-goals
- 本轮不改 reward、termination、扰动参数。
- 本轮不把 `recovery` 重新并回 `stand`。
- 本轮不实现新的 recovery 专项测试，只记录缺口。
- 本轮不处理 runtime 自动技能切换、跌倒检测或 get-up。

## Coordination Notes
- 当前阶段的正确任务边界是：
  - `stand` 保持干净，负责稳定站姿基线
  - `recovery` 承接更宽 reset 分布和 interval 推扰
- 如果后续要补自动化验证，建议拆成两条独立工作流：
  1. recovery 配置/注册单测
  2. recovery 训练 smoke test / warm-start 工作流
