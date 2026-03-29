# moveit_fake_execution_plan.md

## 为什么当前先走 fake/mock backend

当前目标是先打通标准执行链与接口边界，而不是提前接入未完成验证的 mixed protocol 细节。

当前推荐链路：

MoveIt -> joint_trajectory_controller -> ros2_control -> fake/mock hardware

## MoveIt 与硬件层边界

MoveIt 只负责规划与执行接口，不应直接接触：

- bus mapping
- node id
- packet format
- endianness
- vendor register map

这些细节必须留在硬件适配/系统层。

## joint_trajectory_controller 的位置

`joint_trajectory_controller` 是 MoveIt 执行动作到 ros2_control 的标准桥接层，负责接收 FollowJointTrajectory 并驱动关节命令接口。

## 后续替换为真实 hardware system 的路径

1. 继续用 fake backend 验证 MoveIt 执行链稳定性。
2. 保持 controller 命名与关节接口一致。
3. 将 `robot_arm.ros2_control.xacro` 的 plugin 从 `mock_components/GenericSystem` 切换到 `robot_arm_hardware_system/RobotArmHardwareSystem`。
4. 在 C++ hardware system 内部完成 mixed-protocol adapter 真正实现。
5. 逐步灰度联调，避免一次性切换风险。
