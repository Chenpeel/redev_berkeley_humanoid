# Recovery In-Place Reward Plan

## Goal
- 将 `recovery` 从“允许通过移动卸掉扰动”的偏向，收敛到更偏向“原地抗扰”的奖励设计。
- 在不污染 `stand` 干净基线的前提下，只对 `recovery` 增加更明确的原地抗扰约束。
- 保持本轮为 reward shaping 微调，不改 lowlevel runtime，不改 get-up 路线。

## Problem Statement
- 当前 `recovery` 虽然已经学会在 interval 扰动下稳定存活，但 reward 里缺少显式的“不要挪脚 / 不要滑脚 / 不要靠步态对冲”约束。
- 现有 reward 主要是：
  - 零速度跟踪
  - 姿态稳定
  - 动作率 / 力矩 / 加速度惩罚
  - joint deviation
- 这些项会鼓励站稳，但不会明确禁止通过 stepping 或 foot sliding 抵消扰动。

## Change Plan
1. 为 balance MDP 新增一个可复用 reward helper：
   - `feet_air_time_penalty`
2. 在 `BalanceRewardProfile` 中增加可选项：
   - `feet_slide_weight`
   - `feet_air_time_penalty_weight`
   - `feet_body_patterns`
3. 在 `build_rewards_cfg()` 中按配置条件性接入：
   - `feet_slide`
   - `feet_air_time_penalty`
4. 只对 humanoid / biped recovery 配置启用这些新项。
5. 为 recovery 配置增加最小静态测试，确认：
   - `push_robot` 仍是 `interval`
   - recovery 带有新的原地抗扰奖励项
   - stand 不会被新项污染

## Validation Plan
- `python3 -m compileall`
- `uv run ruff check`
- `env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -rs packages/sim/tests/test_balance_task_configs.py`
- 后续由用户重新训练或回放，观察：
  - recovery 是否减少 stepping / shuffling
  - 是否因过度压制移动而导致更容易摔倒
