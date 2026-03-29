# frontend_control_contract.md

## 统一高层状态字段

前端通过 `/state` 获取以下稳定字段（唯一数据源，不重复定义）：

- `robot_state`
- `connection_state`
- `armed`
- `mode`
- `cycle_ms`
- `loss_percent`
- `error_message`
- `can_retry`
- `request`:
  - `action`
  - `status` (`idle/in_progress/success/failed`)
  - `message`
- `joints`（`joint_1 ~ joint_6`）

该结构可由 mock API 或未来真实桥接共同复用。

## armed switch 映射规则

- Switch OFF -> `READY_UNARMED`
- Switch ON -> `ARMED_HOLDING_CURRENT`
- `FAULT` / `UNAVAILABLE` / loading 时 switch 禁用

前端只表达“armed 语义”，不表达底层协议控制细节。

## request.action 约定（当前 mock 与后续 bridge）

- `none`
- `load`
- `enable`
- `disable`
- `fault`
- `recover`
- `stop`
- `clear_fault`

前端只消费动作语义，不推断底层协议过程。

## 为什么前端不能感知底层协议细节

前端若感知 bus/node/protocol/vendor，会导致：

- UI 与硬件实现耦合
- 迁移成本高
- 安全边界模糊

因此前端只消费高层状态契约，协议差异由 backend/router 处理。
