# robot_arm_hardware_system

C++ `ros2_control` hardware skeleton（统一 6 轴系统接口）。

## 本轮定位

- 这是最终 `ros2_control` 方向的底座，不是完整协议实现。
- 真实协议事实仍以 Python 原型与 `docs/protocol_validation.md` 为主。
- 上层只看到 `joint_1 ~ joint_6`。

## 结构

- `RobotArmHardwareSystem`
  - ros2_control `SystemInterface` 骨架
  - 生命周期、read/write、状态与命令缓存
  - 持有 routing layer
  - 持有 backend 抽象接口
  - 提供控制入口：enable/disable/stop/clear_fault/recover（未来可映射 service）
- `JointRouter`
  - 从 model config 读取并校验 joint->driver/bus/node_id
- `IRobotHardwareBackend`
  - fake / real 可替换后端接口
- `FakeRobotHardwareBackend`
  - 6 轴 mock 读写，打通控制链
- `RealMixedRobotBackend`
  - 先做真机只读链骨架（discover/sync/read）

## 安全约束

- 启动遵循：discover -> sync current -> READY_UNARMED
- discovery 不等于 enable
- 不自动 enable / 不自动 lock
- 未使能状态 write 必须拒绝运动命令
- enable 后 holding target 来自当前位置

## 说明

当前 real backend 的写运动仍是 TODO，不包含未验证协议字段实现。
