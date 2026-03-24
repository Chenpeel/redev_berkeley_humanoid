# Recovery In-Place Reward Responsibility

## Scope Owner
- 主代理负责：
  - 设计并实现 `recovery` 的原地抗扰奖励增强
  - 保证 `stand` 任务不被污染
  - 更新最小测试与文档记录
- 用户负责：
  - 重新训练或回放 recovery
  - 根据实际恢复动作质量决定是否继续加强原地约束或降低扰动强度

## Module Boundaries
- 奖励逻辑：
  - `packages/sim/src/berkeley_humanoid_lite/tasks/balance/mdp/rewards.py`
  - `packages/sim/src/berkeley_humanoid_lite/tasks/balance/mdp/__init__.py`
  - `packages/sim/src/berkeley_humanoid_lite/tasks/balance/common.py`
- recovery 配置：
  - `packages/sim/src/berkeley_humanoid_lite/tasks/balance/recovery/config/humanoid/env_cfg.py`
  - `packages/sim/src/berkeley_humanoid_lite/tasks/balance/recovery/config/biped/env_cfg.py`
- 配置测试：
  - `packages/sim/tests/test_balance_task_configs.py`

## Non-goals
- 不修改 `stand` 的 reward。
- 不修改 recovery 的 reset 分布、扰动强度和 episode 时长。
- 不实现新的课程学习。
- 不修改 lowlevel runtime、跌倒检测或 get-up。

## Coordination Notes
- 这次改动的工程含义是：
  - `stand` 继续做纯静态基线
  - `recovery` 继续做抗扰恢复
  - 但 recovery 的策略偏好会更靠近 ankle/hip strategy，而不是 stepping/shuffling
- 如果这版导致恢复成功率明显下降，下一步优先考虑：
  - 适度降低扰动强度
  - 做扰动 curriculum
  - 而不是回头污染 `stand`
