# Recovery Validation Status Research

## One-Line Conclusion
- 当前 `recovery` 已经是一个可训练、带 interval 随机推扰的独立任务，但它的自动化验证还主要停留在“代码能过静态检查、任务已注册”这一层，尚未形成 recovery 专项的静态闭环。

## Why It Should Stay In Recovery
- `stand` 当前已经训练得很干净：
  - episode length 基本打满
  - `time_out` 接近 1
  - `body_contact` 接近 0
- 这类结果的价值在于它是稳定的 stand 基线。
- 如果现在把随机推扰直接并回 `stand`：
  - reset 分布会变脏
  - reward 含义会变混
  - 后续训练退化时很难判断是基线坏了还是扰动设计坏了
- 因此从工程边界上，继续保持：
  - `stand = 干净基线`
  - `recovery = 抗扰恢复`
  是更清晰的。

## Recovery Is Not Empty
- 当前 recovery 明确不是“待实现任务”。
- 它已经包含：
  - 更宽的 reset 分布
  - 更宽的 orientation termination
  - 更长的 episode
  - interval 扰动事件
- 共享事件构造逻辑表明：
  - 只要 `disturbance_profile` 存在，就会生成 `push_robot`
  - 且模式是 `interval`
- 这说明 recovery 的核心训练语义已经具备。

## What Current Static Validation Actually Covers
- `compileall`
  - 证明 recovery 相关模块可编译，无明显语法问题
- `ruff`
  - 证明当前 balance 目录与测试文件无 lint 问题
- `pytest`
  - 当前只稳定覆盖 workflow helper
  - registry 测试虽然写了 recovery task id，但在当前 pytest 环境里被跳过
- README
  - 已经把 recovery 作为正式训练任务对外描述

## What Current Static Validation Does Not Cover
- 没有 recovery 配置专测：
  - 没人断言 `disturbance_profile` 一定生成 `push_robot`
  - 没人断言 `interval_range_s` 和 `velocity_range` 的具体值
- 没有 recovery smoke test：
  - 没有自动化用例去实例化 `Recovery-Berkeley-Humanoid-Lite-v0`
- 没有 recovery 指标闭环：
  - 代码里没有 recovery 成功率 / 恢复时间等专门指标测试
- 没有 warm-start 文档闭环：
  - 现在可以从结构上 warm-start
  - 但仓库还没有把 `stand -> recovery` 接续流程写成固定操作

## Practical Interpretation
- 如果你的问题是“现在能不能从训练策略上进入 recovery 阶段”：
  - 可以
- 如果你的问题是“现在是否已经有足够完备的 recovery 静态验收”：
  - 还不够
- 所以最合理的工程动作不是回去污染 `stand`，而是：
  - 保持 `stand` 干净
  - 进入 `recovery`
  - 同时补最小 recovery 专项验证

## Minimal Missing Items
1. recovery 配置单测
   - 断言 `push_robot` 被创建
   - 断言 `mode="interval"`
   - 断言 humanoid / biped 参数值
2. recovery 任务实例化 smoke test
   - 至少能在 headless 下完成最小环境构造
3. recovery 验收指标定义
   - 恢复成功率
   - 恢复时间
   - 二次摔倒率
4. `stand -> recovery` 接续训练说明
   - checkpoint 选择
   - experiment 命名
   - 是否 warm-start

## Recommendation
- 保持 `stand` 的干净性。
- 下一阶段进入 `recovery`，而不是把推扰回灌到 `stand`。
- 在正式长跑 recovery 之前，先补 recovery 专项最小验证，这样后面训练结果才更可解释。
