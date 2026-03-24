# Recovery In-Place Reward Stack

## Status
- 日期：2026-03-24
- 当前阶段：已完成 `recovery` 原地抗扰 reward 增强的第一轮代码改造与静态检查。
- 本轮未改动 `stand` reward，也未动 runtime。

## Problem Confirmation
- 用户明确提出当前 `recovery` 的问题不是“站不稳”，而是策略可能通过移动来对冲扰动。
- 代码检查确认这一点是合理的：
  - 当前 `recovery` reward 主要奖励零速度和姿态稳定
  - 但没有显式惩罚 foot lift 或 foot sliding
- 因此策略即使靠 stepping/shuffling 达到较低速度误差，也不会被专门打压。

## Implemented Changes
- 新增：
  - `packages/sim/src/berkeley_humanoid_lite/tasks/balance/mdp/rewards.py`
    - `feet_air_time_penalty`
- 更新：
  - `packages/sim/src/berkeley_humanoid_lite/tasks/balance/mdp/__init__.py`
    - 导出新的 reward helper
  - `packages/sim/src/berkeley_humanoid_lite/tasks/balance/common.py`
    - `BalanceRewardProfile` 增加：
      - `feet_body_patterns`
      - `feet_slide_weight`
      - `feet_air_time_penalty_weight`
    - `build_rewards_cfg()` 支持条件性接入：
      - `feet_slide`
      - `feet_air_time_penalty`
  - `packages/sim/src/berkeley_humanoid_lite/tasks/balance/recovery/config/humanoid/env_cfg.py`
    - 开启：
      - `feet_slide_weight=-0.2`
      - `feet_air_time_penalty_weight=-0.1`
  - `packages/sim/src/berkeley_humanoid_lite/tasks/balance/recovery/config/biped/env_cfg.py`
    - 开启：
      - `feet_slide_weight=-0.25`
      - `feet_air_time_penalty_weight=-0.15`
  - `packages/sim/tests/test_balance_task_configs.py`
    - 新增 recovery 配置层最小测试

## Rationale
- `feet_slide`
  - 直接惩罚接触状态下的足端水平滑移
  - 主要抑制 shuffling
- `feet_air_time_penalty`
  - 直接惩罚脚离地时间
  - 主要抑制 stepping
- 两项都只接到 `recovery`
  - 避免破坏已经收敛得很干净的 `stand`

## Validation
- `timeout 60s python3 -m compileall packages/sim/src/berkeley_humanoid_lite/tasks/balance packages/sim/tests/test_balance_task_configs.py`
  - 通过
- `uv run ruff check packages/sim/src/berkeley_humanoid_lite/tasks/balance packages/sim/tests/test_balance_task_configs.py`
  - 通过
- `env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -rs packages/sim/tests/test_balance_task_configs.py`
  - 2 skipped
  - skip 原因：
    - 当前 pytest 环境无法导入 `isaaclab.utils`

## Current Limitation
- 这次改动只改变 reward 偏好，不保证“绝对不挪脚”。
- 如果当前 interval 扰动强度已经超过 support polygon 能承受的范围，那么纯靠 reward 也不一定能逼出理想的原地抗扰。
- 如果后续训练发现：
  - stepping 依然明显
  - 或成功率明显下降
  下一步应该评估：
  - 扰动强度是否过大
  - 是否需要 disturbance curriculum
  - 是否需要进一步增加基于 base drift 的约束

## Next Actions
1. 重新训练或至少回放 `recovery`，观察 stepping/shuffling 是否下降。
2. 对比：
   - `Episode_Termination/body_contact`
   - `Episode_Termination/base_orientation`
   - 恢复后姿态质量
3. 如果动作明显更稳且不乱挪脚，保留这版 reward。
4. 如果策略变得僵硬、恢复率下降，再考虑分阶段 curriculum，而不是直接撤销全部原地约束。
