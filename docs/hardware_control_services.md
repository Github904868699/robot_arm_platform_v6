# hardware_control_services.md

## 服务列表（最小真实入口）

当前落地的硬件服务（`std_srvs/Trigger`）：

- `/robot_arm/hardware/enable`
- `/robot_arm/hardware/disable`
- `/robot_arm/hardware/stop`
- `/robot_arm/hardware/clear_fault`
- `/robot_arm/hardware/recover`
- `/robot_arm/hardware/write_single_joint_step`（单关节最小写入测试入口）

## 请求/响应语义

- 请求：空请求（Trigger）
- 响应：
  - `success: bool`
  - `message: string`

message 用于表达失败原因，如：
- `enable_requires_ready_unarmed`
- `recover_requires_fault`
- `write_requires_enabled_synced_holding`
- `write_delta_too_large`

## 可调用状态（核心）

- `enable`：仅 `READY_UNARMED`
- `disable`：非 `FAULT` 状态可调用，目标 `READY_UNARMED`
- `stop`：任意状态可调用，目标 `FAULT`
- `clear_fault`：可清除故障标记，目标 `READY_UNARMED`
- `recover`：仅 `FAULT` 状态可调用，目标 `READY_UNARMED`
- `write_single_joint_step`：仅 `ARMED_HOLDING_CURRENT`，且 discover/sync 已完成

## 不可调用状态（示例）

- 在 `FAULT` 调用 `disable` -> reject
- 在非 `FAULT` 调用 `recover` -> reject
- 在未 enable 或未 sync 调用单关节写入 -> reject

## 与状态机关系

状态流转保持：

- discover -> sync -> `READY_UNARMED`
- `enable` -> `ARMING` -> `ARMED_HOLDING_CURRENT`
- `stop` -> `FAULT`
- `recover/clear_fault` -> `READY_UNARMED`

不允许：
- auto-enable
- auto-lock
- `recover` 直接跳 `EXECUTING`

