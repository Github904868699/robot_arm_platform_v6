# real_backend_readonly_plan.md

## 当前只读链推进程度

`RealMixedRobotBackend` 已形成清晰三段式只读流程：

1. `discover_joints()`
2. `sync_current_positions()`
3. `read_all_joint_states()`

并明确保持写路径关闭（TODO）。

## Hightorque / Yiyou 只读分层

当前在 real backend 内部拆分为：

- `HightorqueReadChannel`
- `YiyouReadChannel`

两者分别承载各自 read path 的 discover/read 入口，再由 backend 按 routing 汇总到统一 6 轴状态。
并通过 `read_joint_by_route()` 聚合，避免上层散落 driver 分支。

当前补充：

- `HightorqueReadChannel::discover/sync_current_positions/read_joint`
- `YiyouReadChannel::discover/sync_current_positions/read_joint`
- `RealMixedRobotBackend::sync_hightorque_positions/sync_yiyou_positions`

## 为什么还不进入真实写运动

- 协议字段和异常路径仍在 Python 原型持续验证。
- 当前优先保证 discover/sync/read 的边界正确。
- 写控制若过早引入，会放大返工风险和安全风险。

## 已验证事实与 TODO 边界

- 已验证方向：双总线路径与协议分工来源于 `docs/protocol_validation.md`。
- TODO：真实帧级 read 实现、异常恢复细节、写控制闭环。
- 当前 discover 还依赖 `/dev/ttyACM0` 和 `/dev/ttyACM1` 设备存在性探测。

## 下一步只读完成后的最小目标

1. 用稳定协议读命令替换 channel 内 TODO read。
2. 完成 discover/sync/read 的错误码和 fault 策略。
3. 在保持写关闭前提下，先打通长期只读监控链。

## 当前边界（本轮确认）

- 继续保持 `write_all_joint_commands()` 返回 false（写路径关闭）。
- discover/sync/read 结构已稳定为：
  - `discover_hightorque_joints()`
  - `discover_yiyou_joints()`
  - `read_joint_by_route()`
- 不新增未验证协议字段，不伪造厂商寄存器细节。
- 读取字段当前只输出统一 `position/velocity`，协议细节仍留在 adapter 演进阶段。

## 本轮真实帧级只读落地（最小）

- Hightorque adapter 已能：
  - 打开 `/dev/ttyACM0`
  - 进行原始帧行读取 `read_raw_frame_line()`
  - 在 `read_joint_state()` 中解析最小 `position/velocity/online` 字段
- Yiyou adapter 已能：
  - 打开 `/dev/ttyACM1`
  - 执行已验证桥接初始化序列 `C/S8/M0/A1/O`
  - 原始帧行读取与最小字段解析（mode/position/velocity）

仍需继续验证：
- 帧查询命令与响应格式的稳定性
- 字段单位/比例/符号方向
- 异常帧与超时恢复策略
