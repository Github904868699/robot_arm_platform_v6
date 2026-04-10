# HighTorque 实机最小闭环调试说明（本轮）

## 1. 本轮收敛的真实语义
本轮只保证并建议先验证以下链路：

1. `enable`
2. `hold current position`
3. `single-axis small step position move`
4. `stop`
5. `disable`

不在本轮范围：多轴轨迹质量、MoveIt 跟踪优化、Web/UI。

## 2. 当前 HighTorque 控制模式（最小化）
后端模式被收敛为 3 态：

- `DISABLED`
- `HOLD_ACTIVE`
- `STEP_MOVE_ACTIVE`

切换规则（简化）：

- `enable` 成功后进入 `HOLD_ACTIVE`。
- 当收到单轴小步进命令（位置变化或非零速度）时进入 `STEP_MOVE_ACTIVE`。
- 连续 `0.20s` 无新步进命令则回到 `HOLD_ACTIVE`，并冻结最后有效 target。
- `stop/disable` 回到 `DISABLED`。

## 3. 当前真正发送的命令语义
- `HOLD_ACTIVE` 下发送：`semantic=hold_current_position`
- `STEP_MOVE_ACTIVE` 下发送：`semantic=mit2_servo_cycle`

两者底层都走 MIT2 位置目标帧族；`hold_current_position` 是在上层语义上明确“当前帧用于持位”。

> 厂家专有 enable 帧语义是否完全等价“伺服上使能”尚未在本仓库中获得厂家文档级证据。
> 本仓库当前以实机可观测行为 + 日志语义进行验证，请在真机上按下面步骤确认。

## 4. 读写节拍与参数
### 4.1 HighTorque 写节拍
写周期来自：
- `ROBOT_ARM_HT_MIT2_PERIOD_MS`（全局）
- 或 joint override（若配置）

### 4.2 HighTorque 读节拍
新增显式轮询周期环境变量：
- `ROBOT_ARM_HT_POLL_PERIOD_MS_ARMED`（默认 10ms）
- `ROBOT_ARM_HT_POLL_PERIOD_MS_UNARMED`（默认 10ms）

本轮改为 enabled 也默认“全关节每轮询周期都读”（取消 enabled 轮询 round-robin 降采样），
因此每关节最慢反馈周期目标约为：

`poll_period_ms_armed * high_torque_joint_count`

### 4.3 TX 周期控制（本轮修复重点）
- HighTorque TX 线程不再用 `enabled==true` 作为 `wait_for` 立即返回条件。
- 采用“deadline 驱动”的严格周期发送：每轮按 `period_ms` 推进 `next_deadline`。
- 命令更新会触发 `command_update` 唤醒，但若尚未到 deadline，只记录并延后到周期边界发送（避免洪泛刷帧）。
- 日志中可区分：
  - `wake_by=periodic`
  - `wake_by=command_update deferred_until_period`
  - `wake_by=command_update at_period_boundary`

并在统计日志中输出：
- `configured_period_ms`
- `avg_period_ms`
- `tx_hz`
- `per_joint_hz`
- `wake_periodic`
- `wake_command`

## 5. 关键日志（真机必看）
- `HIGHTORQUE_ENABLE_SAMPLE ... sample_age_sec=...`
- `HIGHTORQUE_HOLD_INIT ... frozen_hold_target_turns=...`
- `HIGHTORQUE_MODE transition HOLD_ACTIVE -> STEP_MOVE_ACTIVE`
- `HIGHTORQUE_MODE transition STEP_MOVE_ACTIVE -> HOLD_ACTIVE ...`
- `HIGHTORQUE_HOLD_FREEZE joint=... target_turns=...`
- `HIGHTORQUE_MIT2_SERVO ... vel_src=hold_zero|controller|position_diff_fallback`
- `[Hightorque][WRITE] semantic=hold_current_position|mit2_servo_cycle`
- `hightorque_tx_loop: ... mode=HOLD_ACTIVE|STEP_MOVE_ACTIVE`
- `polling_loop_hightorque: ... period_ms=...`

## 6. 真机单轴验证建议（必须）
1. 上电、启动、enable 后，不下发轨迹，仅观察：
   - 是否稳定持位；
   - 是否持续出现 `semantic=hold_current_position`。
2. 仅对单轴（建议 joint_3）做 ±小步进（例如 ±0.01 turns）：
   - 进入 `STEP_MOVE_ACTIVE`；
   - 到位后约 0.2s 回到 `HOLD_ACTIVE`；
   - 回切时不应出现突跳。
3. 执行 `stop`：
   - 确认 stop 帧真正下发。
4. 执行 `disable`：
   - 确认 disable 帧真正下发，模式回 `DISABLED`。

## 7. 本轮仍未承诺
- 未承诺已验证厂家专有 enable 帧完整语义。
- 未承诺多轴轨迹（JTC/MoveIt）质量已达可用。
- 未承诺 Yiyou 侧行为已同步优化。

## 8. enable 后 HOLD GUARD（本轮新增）
`RobotArmHardwareSystem` 在 `request_enable()` 成功后进入 hold guard 窗口，避免立刻误入 EXECUTING。

可配置参数（hardware params）：
- `hold_guard_window_sec`（默认 0.6）
- `exec_enter_pos_threshold_rad`（默认 0.003）
- `exec_enter_vel_threshold_rad_s`（默认 0.02）
- `exec_enter_required_cycles`（默认 3）

判定原则：
- guard 窗口内抑制执行态切换；
- guard 结束后，必须“超过阈值且连续满足若干周期”才进入 EXECUTING；
- 首次进入 EXECUTING 会日志打印触发 joint、位置/速度触发值和累计周期。

## 9. enable seed 对齐（本轮新增）
- `RobotArmHardwareSystem` 在 `request_enable()` 内先生成统一 seed 快照，并打印：
  - `ENABLE_SEED_SYNC joint=... ros_rad=... backend_turns=...`
- 同一份 seed 通过 backend 接口 `set_hold_seed_snapshot(...)` 下发到 `RealMixedRobotBackend`，
  对齐更新：
  - `last_command_position_`
  - `hightorque_hold_targets_`
  - `hightorque_desired_commands_`
- backend 会打印：
  - `BACKEND_ENABLE_SEED_SYNC ...`
  - 如发现旧值不一致：`ENABLE_SEED_MISMATCH ...`

## 10. guard 期间 step 转换门控（本轮新增）
- hardware_system 显式调用 backend `set_step_transition_enabled(false|true, reason)`，
  不再让 backend 仅靠 target delta“猜测”。
- guard 期间若出现 step 候选，backend 记录：
  - `why_step_rejected_by_guard ...`
- guard 结束后真实 step 才允许：
  - `why_step_allowed ...`
