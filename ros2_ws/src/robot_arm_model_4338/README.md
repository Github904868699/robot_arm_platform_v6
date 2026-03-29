# robot_arm_model_4338

## 目标

`robot_arm_model_4338` 负责维护 4338 机型的模型配置，不承载平台通用逻辑。

## 当前内容

- `config/joint_mapping.example.yaml`
  - 演示 `joint_1 ~ joint_6` 到驱动、总线、节点 ID 的映射。
  - 仅示例，不应在 `robot_arm_core` / MoveIt / Web 层硬编码。

## 与 C++ routing layer 的关系

`robot_arm_hardware_system::JointRouter` 读取该映射，用于：

- 统一 6 轴逻辑关节到 backend 子路径的路由
- 将 mixed-protocol 事实限制在 hardware/routing 层

## 边界说明

- 允许放置：
  - joint -> driver / bus / node_id
  - 关节限位、零偏、方向反转等机型参数（后续）
- 不应放置：
  - 平台状态机核心逻辑
  - 协议收发实现
