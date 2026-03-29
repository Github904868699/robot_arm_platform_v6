# minimal_real_write_path.md

## 为什么本轮只做单关节最小真实写入

当前目标是验证“真实写入最小门槛”而不是进入完整执行控制：

- 只允许单关节
- 只允许小步进
- 不接 MoveIt 轨迹执行
- 不形成持续闭环写控制

这样可以降低风险，并在真实硬件上验证最基础写路径是否可控。

## 允许状态

单关节最小写入必须满足：

1. discover 已完成
2. sync current positions 已完成
3. 当前不在 `FAULT`
4. 已显式 `enable`
5. 当前状态为 `ARMED_HOLDING_CURRENT`
6. 目标关节在线
7. 写入步进 `|delta| <= single_write_max_abs_delta`

## 禁止状态

- `DISCONNECTED`
- `DISCOVERING`
- `SYNCING_POSITION`
- `READY_UNARMED`
- `FAULT`

以及任何“多关节联动”写入请求。

## safety guard

当前写入口为：

- `/robot_arm/hardware/write_single_joint_step`（Trigger）

由参数限制：

- `single_write_joint`
- `single_write_delta`
- `single_write_max_abs_delta`

并在运行时执行状态检查和关节在线检查。

## 为什么还不接多关节真执行

因为当前仍缺：

1. 多关节同步写入与时序校验
2. 完整 fault/recover 执行级保护
3. 协议字段的长期稳定验证
4. MoveIt 到真实写链的安全门控闭环

在这些前，不进入多关节真执行。

