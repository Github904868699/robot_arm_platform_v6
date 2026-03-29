# cpp_hardware_skeleton_plan.md

## 为什么先做 C++ hardware skeleton，而不是直接做真实协议实现

当前协议实测结论仍在 Python 原型与 `docs/protocol_validation.md` 持续沉淀阶段。直接把未完全验证的协议细节写入 C++ 插件，会放大返工成本并带来安全风险。

因此本轮先建立 `ros2_control` 兼容骨架：

- 明确生命周期接口
- 明确状态机与命令 gating
- 明确 mixed-protocol 在硬件层内部拆分

## 统一 6 轴接口的意义

`RobotArmHardwareSystem` 对上层统一暴露 `joint_1 ~ joint_6`，保证 MoveIt / 控制器 / Web bridge 不感知 bus、node_id、endianness 与 vendor 寄存器。

## mixed protocol adapter 的角色

在硬件层内部维护两个 adapter：

- HightorqueAdapter（CAN-FD，joint_1/3/4/5/6）
- YiyouAdapter（CAN 2.0A，joint_2）

它们负责协议族差异；统一系统类只做 joint-level 聚合与生命周期管理。

## 为什么未使能时不能允许运动写入

根据启动安全约束：

1. 先发现
2. 再同步当前位置
3. 进入 `READY_UNARMED`
4. 用户显式 enable 前禁止运动写入

这可以防止启动瞬间出现突然锁定/突跳/追旧目标。

## 从 Python 原型迁移到真实 C++ hardware layer 的建议路径

1. 固化 model 配置 schema（joint->driver->bus->node_id）。
2. 在 Python 侧补齐已验证命令与异常路径回归。
3. 抽取协议无关 adapter 接口契约。
4. 在 C++ adapter 中按契约逐步替换 mock 读写。
5. 最后切换 ros2_control plugin 到真实实现，并保留 READY_UNARMED 安全流程。
