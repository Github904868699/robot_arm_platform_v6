# enable_disable_control_plan.md

## 控制语义入口（高层）

- `enable`
- `disable`
- `stop`
- `clear_fault`
- `recover`
- `fault`（mock 注入故障入口，用于演练恢复路径）

这些入口当前通过 ROS2 `std_srvs/Trigger` 暴露（硬件服务层）：

- `/robot_arm/hardware/enable`
- `/robot_arm/hardware/disable`
- `/robot_arm/hardware/stop`
- `/robot_arm/hardware/clear_fault`
- `/robot_arm/hardware/recover`

## 请求生命周期语义（桥接层）

每次控制请求应至少表达：

- `idle`
- `in_progress`
- `success`
- `failed`

用于前端弱反馈与状态提示，不等于底层协议执行细节。

## READY_UNARMED vs ARMED_HOLDING_CURRENT

- `READY_UNARMED`：允许读状态，不允许运动写入。
- `ARMED_HOLDING_CURRENT`：显式 enable 后进入，允许控制层进入可执行态。

## 允许状态矩阵（桥接层语义）

- `enable` 仅允许在 `READY_UNARMED` 且 `connection_state=CONNECTED`
- `disable` 允许在 `ARMED_HOLDING_CURRENT / EXECUTING / READY_UNARMED`
- `stop` 允许在任意状态，请求后进入 `FAULT`
- `clear_fault` 允许清除故障标记，目标状态 `READY_UNARMED`
- `fault` 可在任意状态注入，结果固定进入 `FAULT`
- `recover` 仅允许在 `FAULT` 且 `connection_state=CONNECTED`

返回语义均采用：
- `in_progress`（收到请求）
- `success`（状态迁移成功）
- `failed`（请求被拒绝或前置条件不满足）

## 为什么 holding target 必须来自当前位置

enable 时若沿用旧目标，可能产生跳变或突兀接管。

因此必须在 enable 过程中以当前读取到的位置建立 holding target，再进入 armed holding 状态。

## recover 路径约束

- `FAULT` -> `recover(in_progress)` -> `READY_UNARMED`
- recover 不自动 enable、不自动 lock，不进入 holding 写控制
