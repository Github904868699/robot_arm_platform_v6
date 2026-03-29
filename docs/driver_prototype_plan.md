# driver_prototype_plan.md

## 1) 为什么当前不能直接写“统一协议驱动”

当前硬件是混合协议双总线：

- `joint_1/3/4/5/6`：Hightorque CAN-FD
- `joint_2`：Yiyou CAN 2.0A

两侧在协议形态、数据组织和链路初始化上都不相同。若直接做“统一协议驱动”，会把协议差异硬塞进核心层，造成：

- 平台层被具体协议污染
- 后续扩展新电机/新总线成本升高
- 调试边界不清晰

因此统一的是“上层逻辑关节视图”，不是“底层协议本体”。

## 2) 为什么必须拆成两个 adapter

拆分 adapter 的价值：

- 把 CAN-FD 与 CAN 2.0A 的协议细节隔离
- 各自可以独立迭代与验证
- 减少交叉影响，便于故障定位
- 保证模型层（mapping）能清晰表达 joint->adapter->bus->node_id

这与当前实机拓扑一一对应，也符合后续可复用平台架构。

## 3) 为什么上层只能看到统一 6 轴

MoveIt/Web/任务层是“产品能力层”，不是协议调试层。

上层若感知 bus/node_id/endianness，会导致：

- UI/规划与协议细节耦合
- 迁移硬件时上层大面积改动
- 安全约束更难统一治理

因此上层只应面向：`joint_1 ~ joint_6`、状态机、任务语义。

## 4) 为什么当前先做 Python 原型而不是直接上 ros2_control plugin

当前阶段目标是“边界先正确、路径先跑通”，而非一次性实现完整插件。

先做 Python 原型有优势：

- 开发回路短，便于快速验证启动序列与安全 gating
- 能更快沉淀协议实测结论（已验证 vs 未验证）
- 降低早期架构失误在 C++ 插件中的返工成本

同时，本轮明确不替代未来 ros2_control 方案。

## 5) 后续如何从 Python 原型迁移到 ros2_control hardware plugin

建议迁移路径：

1. **稳定模型配置层**
   - 固化 joint mapping、限制、偏置等 schema
2. **稳定 adapter 接口契约**
   - 明确 open/scan/read/enable/disable/command 语义
3. **补齐协议测试矩阵**
   - 命令级回归、异常路径、断连恢复
4. **抽离可复用传输与协议库**
   - Python 中先沉淀行为规范
5. **实现 ros2_control hardware plugin**
   - 复用已稳定的数据模型与状态机约束
6. **灰度切换**
   - 在仿真/台架先替换，再进整机

## 6) 当前 Web 为什么先做壳子、不接真控制

当前 Web 先做壳子的原因：

- 先验证产品信息架构与交互路径
- 避免 UI 过早依赖尚不稳定的底层接口
- 保持“Web 不感知协议细节”的硬边界
- 可并行推进 teach/task 设计（MoveJ/MoveL/MoveC/Wait/SetIO）

这能让后续 API 接入时改动集中在 bridge 层，而不是重写前端结构。

## 已验证与未验证边界

- **已验证（来自 `docs/protocol_validation.md`）**：
  - `/dev/ttyACM0` 为 Hightorque 路线
  - `/dev/ttyACM1` 为 Yiyou 路线
  - 初始化相关命令与部分控制命令已做实测
- **未验证**：
  - 完整寄存器/帧字段全集
  - 全量异常恢复策略
  - 实时性能与并发压力行为

未验证部分在代码中仅以注释/TODO 形式保留，不进行伪实现。
