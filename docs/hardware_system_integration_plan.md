# hardware_system_integration_plan.md

## 当前 fake backend 在链路中的位置

当前推荐执行链：

MoveIt -> joint_trajectory_controller -> ros2_control `RobotArmHardwareSystem` -> `FakeRobotHardwareBackend`

这样可以先验证控制链结构与状态机，不依赖真实协议闭环。

## 未来 real backend 替换 fake backend 的方式

`RobotArmHardwareSystem` 通过 `IRobotHardwareBackend` 抽象接口持有后端，实现可替换：

- `backend_mode=fake` -> `FakeRobotHardwareBackend`
- `backend_mode=real` -> `RealMixedRobotBackend`（当前占位）

替换时不改 MoveIt，不改 controller，仅替换 backend 实现。

## 最小 bringup 集成

新增 `robot_arm_bringup_4338/launch/bringup.launch.py`，提供：

- `use_fake_hardware`
- `backend_mode`
- `joint_mapping_path`

用于在同一组合层下切 fake/real 路径，保持 MoveIt 层不变。

## MoveIt 为什么只能看到统一 6 轴接口

MoveIt 属于规划/执行上层，应仅处理逻辑关节：
- `joint_1 ~ joint_6`

不应接触：
- bus
- node_id
- packet format
- endianness
- vendor register

这些细节只能留在 routing layer + real backend。

## 当前阶段为什么不直接让 MoveIt 接真实 mixed protocol

因为真实协议仍在 Python 原型中持续验证（命令、单位、状态、异常路径）。

当前先把 ros2_control system + backend 抽象 + routing 边界做对，可降低后续迁移风险，并避免把未定协议细节污染上层。

## 当前硬件服务入口

新增 `/robot_arm/hardware/*` 最小服务层用于落地控制入口：

- enable / disable / stop / clear_fault / recover
- write_single_joint_step（单关节最小写入）

该服务层负责状态机约束与 safety guard，避免上层直接接触协议细节。
