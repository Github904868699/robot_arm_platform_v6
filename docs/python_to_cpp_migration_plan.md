# python_to_cpp_migration_plan.md

## 1. 当前 Python 原型的价值

Python 原型仍是当前协议验证主阵地：

- 实机命令试验来源
- 单位/方向/状态反馈的快速验证来源
- 与 `docs/protocol_validation.md` 一起构成“已验证事实”集合

## 2. 为什么先做 C++ skeleton，而不是直接复制 Python 原型

直接复制会在协议细节尚未稳定时过早固化实现，导致：

- 返工成本高
- 安全风险上升
- ros2_control 与协议实现耦合过深

先做 C++ skeleton 的目的：

- 先把 ros2_control 生命周期与接口边界做正确
- 先把 routing 与 fake/real backend 可替换结构立起来
- 为后续真实实现提供稳定挂载点

## 3. 未来迁移方式

1. 在 Python 原型持续补齐协议验证（含异常路径）。
2. 把稳定后的协议行为迁移到 `RealMixedRobotBackend`。
3. 在 real backend 内组合高擎/意优适配路径。
4. 通过 backend 选择机制将 `fake` 切换到 `real`。
5. MoveIt 上层保持不变，始终只面对统一 6 轴接口。

## 4. 当前最小 ROS2 对接面

当前新增最小 ROS2 对接面用于缩短迁移闭环：

- `/robot_arm/hardware/enable|disable|recover|stop|clear_fault`（`std_srvs/Trigger`）
- `/robot_arm/hardware/write_single_joint_step`（单关节最小写入）
- `/robot_arm/control/state_json`（统一高层状态 topic）

Web bridge 优先消费该统一状态，不直接接触协议细节。
