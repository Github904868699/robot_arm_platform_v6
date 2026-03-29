# state_machine_control_plan.md

## 语义入口

- `enable request`
- `disable request`
- `stop request`
- `clear_fault request`
- `recover request`

这些入口当前以最小 ROS2 service 暴露（`std_srvs/Trigger`）：

- `/robot_arm/hardware/enable`
- `/robot_arm/hardware/disable`
- `/robot_arm/hardware/stop`
- `/robot_arm/hardware/clear_fault`
- `/robot_arm/hardware/recover`

并保留向 `RobotArmHardwareSystem` 控制方法继续下钻的路径。

## 状态定义

- `DISCOVERING`
- `SYNCING_POSITION`
- `READY_UNARMED`
- `ARMING`
- `ARMED_HOLDING_CURRENT`
- `EXECUTING`
- `FAULT`

## 核心流转

1. `on_activate`：`DISCOVERING -> SYNCING_POSITION -> READY_UNARMED`
2. `enable request`：`READY_UNARMED -> ARMING -> ARMED_HOLDING_CURRENT`
3. write 命令：仅在使能后允许，执行时进入 `EXECUTING`
4. `disable request`：回到 `READY_UNARMED`
5. `stop request` 或读写失败：进入 `FAULT`
6. `clear_fault/recover`：从 `FAULT` 回到 `READY_UNARMED`（不自动使能）

## 允许/禁止状态规则（服务语义）

- enable：仅 `READY_UNARMED` 可调用
- disable：`ARMED_HOLDING_CURRENT / EXECUTING / READY_UNARMED` 可调用
- stop：任意状态可调用，目标 `FAULT`
- clear_fault：可清除 fault 标记，目标 `READY_UNARMED`
- recover：仅 `FAULT` 可调用，且不直接跳 `EXECUTING`

## READY_UNARMED vs ARMED_HOLDING_CURRENT

- `READY_UNARMED`：状态可读，但运动写入必须拒绝。
- `ARMED_HOLDING_CURRENT`：显式 enable 后进入，holding target 来自当前读取位置。

## 为什么 discovery 不等于 lock

discovery/sync 只代表“系统知道当前位置”，不代表“允许接管执行”。

若 discovery 自动 lock，会导致启动突兀、潜在跳变和用户信任问题。
