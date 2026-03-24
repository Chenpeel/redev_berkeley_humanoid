# Recovery Validation Status Stack

## Status
- 日期：2026-03-24
- 当前阶段：已完成 recovery 任务实现现状核对、静态校验复跑，并拿到了 stand / recovery 的首轮完整训练结果。
- 当前结论：
  - `stand` 已经形成高质量干净基线
  - `recovery` 也已经完成首轮训练并表现稳定
  - 但 recovery 的专项自动化验证和专项评估闭环仍然不完整

## Current Repository Findings
- `recovery` 任务已经存在完整的 humanoid / biped 配置文件：
  - `packages/sim/src/berkeley_humanoid_lite/tasks/balance/recovery/config/humanoid/env_cfg.py`
  - `packages/sim/src/berkeley_humanoid_lite/tasks/balance/recovery/config/biped/env_cfg.py`
- `recovery` 已经注册进 Gym task registry：
  - `Recovery-Berkeley-Humanoid-Lite-v0`
  - `Recovery-Berkeley-Humanoid-Lite-Biped-v0`
- README 已经公开声明：
  - `Recovery-*` 在 stand 任务基础上加入 interval push 扰动和更宽 reset 分布
  - 对应训练命令和默认实验目录也已经列出

## Recovery Already Has Disturbance
- 当前 `recovery` 不是空壳任务，已经明确带扰动。
- humanoid recovery：
  - `interval_range_s=(4.0, 6.0)`
  - `velocity_range={"x": (-0.6, 0.6), "y": (-0.4, 0.4)}`
- biped recovery：
  - `interval_range_s=(4.0, 6.0)`
  - `velocity_range={"x": (-0.5, 0.5), "y": (-0.35, 0.35)}`
- 在共享 `build_events_cfg()` 里，这会生成：
  - `push_robot = EventTerm(..., mode="interval")`
- 也就是说，当前 recovery 已经具备 episode 内的随机推扰，不需要先把这部分塞回 `stand`。

## Validation Results
- `timeout 60s python3 -m compileall packages/sim/src/berkeley_humanoid_lite/tasks packages/sim/tests/test_task_registry.py packages/sim/tests/test_workflows.py`
  - 通过
- `uv run ruff check packages/sim/src/berkeley_humanoid_lite/tasks/balance packages/sim/tests/test_task_registry.py packages/sim/tests/test_workflows.py`
  - 通过
- `env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 uv run pytest -rs packages/sim/tests/test_task_registry.py packages/sim/tests/test_workflows.py`
  - 3 passed, 2 skipped
  - skip 原因：
    - `isaaclab-dependent task registry import unavailable: No module named 'isaaclab.utils'`

## Training Results
- `stand` 首轮训练已完成：
  - 迭代：`4000 / 4000`
  - `Mean reward = 129.11`
  - `Mean episode length = 985.26`
  - `Episode_Termination/time_out = 0.9898`
  - `Episode_Termination/base_orientation = 0.0100`
  - `Episode_Termination/body_contact = 0.0027`
- 结论：
  - 当前 `stand` 表现已经足够强，可以作为后续所有恢复任务的干净基线。

- `recovery` 首轮训练也已完成：
  - 迭代：`6000 / 6000`
  - `Mean reward = 72.36`
  - `Mean episode length = 991.82`
  - `Episode_Termination/time_out = 0.9705`
  - `Episode_Termination/base_orientation = 0.0256`
  - `Episode_Termination/body_contact = 0.0055`
  - `Metrics/base_velocity/error_vel_xy = 0.0839`
  - `Metrics/base_velocity/error_vel_yaw = 0.2151`
- 结论：
  - 在带 interval 扰动和更宽 reset 的前提下，recovery 已经学到稳定策略，不是“只能注册、还没训通”的状态。
  - 相比 stand，reward 更低、姿态误差和终止率更高是预期现象，因为任务本身更难。

## What Is Already Validated
- 代码层面没有语法错误。
- balance 目录和现有 sim tests 能通过 ruff。
- workflow helper 测试通过。
- recovery 任务 ID 已经写进 task registry 测试。
- README 中 recovery 的定位、训练命令和产物目录已经同步。
- `stand` 与 `recovery` 两个训练任务都已经实际跑通到默认迭代上限。
- 从训练日志看，当前任务设计至少在“可收敛、可稳定运行”这一层已经成立。

## What Is Not Fully Validated Yet
- 现有 pytest 没有真正执行 recovery 专项测试。
- `test_task_registry.py` 里虽然包含 recovery task id 断言，但当前在普通 pytest 环境下被跳过了。
- 仓库里没有专门验证以下内容的自动化测试：
  - recovery 的 `disturbance_profile` 是否正确注入 `push_robot`
  - `push_robot` 是否确实是 `mode="interval"`
  - humanoid / biped recovery 的 reset 范围、termination 阈值、runner experiment name 是否符合预期
  - recovery 任务是否能在最小 headless smoke 环境里成功 `gym.make(...)`
- 当前也没有 recovery 专项指标测试，例如：
  - 恢复成功率
  - 恢复时间
  - 二次摔倒率
- 当前训练日志里虽然已经有 termination 比例，但还没有 recovery 专项验收面板，不能直接回答：
  - 扰动后多久恢复
  - 恢复过程中是否出现明显二次失稳
  - 在不同推扰方向和强度下的成功率分布

## Judgement
- 如果问题是“`recovery` 代码和任务定义有没有落地”，答案是：**已经落地**。
- 如果问题是“`recovery` 是否已经训通”，答案是：**已经训通第一版，而且训练曲线末端指标是稳定的**。
- 如果问题是“`recovery` 的静态验证是否已经完整闭环”，答案是：**还没有完整闭环，只完成了基础静态校验和注册层校验**。
- 如果问题是“要不要继续保持 `stand` 干净，把推扰放在 `recovery`”，答案是：**应该保持这样，不建议第一轮把推扰并回 `stand`**。

## Main Gaps
1. 缺 recovery 专项单测。
2. 缺 recovery 任务实例化 smoke test。
3. 缺从 `stand` checkpoint 迁移到 `recovery` 的明确训练流程文档。
4. 缺 recovery 专项评估指标，不利于后续判断“抗扰恢复是否真的学到”。
5. 缺 recovery 回放 / 定向推扰可视化验证，无法把“训练稳定”直接翻译成“恢复动作质量足够好”。

## Suggested Next Actions
1. 将当前 `stand` checkpoint 固定为干净基线，不再往里面回灌推扰。
2. 将当前 `recovery` checkpoint 作为 push-recovery 第一版基线保存下来。
3. 下一步优先做 recovery 的回放和定向评估，而不是回头污染 `stand`。
4. 建议至少补两类最小验证：
   - recovery 配置/事件单测
   - recovery headless 实例化 smoke test
5. 补一条清晰的 `stand -> recovery` warm-start 操作说明。
6. 如果回放质量确认没问题，再进入下一阶段：
   - posture recovery
   - 更强扰动课程
   - 或 recovery 专项指标采集

## Final Recommendation
- 训练策略上：
  - `stand` 保持干净
  - 抗扰恢复放在 `recovery`
- 工程质量上：
  - 现在已经不是“要不要进入 recovery 训练”的问题，而是“recovery 训练已经完成，下一步该做 recovery 评估”
  - 不要把当前“3 passed, 2 skipped”当成完整静态验收
  - 也不要把当前训练日志直接等价成“恢复能力已经完成验收”
