# robot_arm_core

平台通用核心层（最小原型骨架）。

## 目标

- 对外暴露统一 6 轴语义，不暴露底层混合协议细节。
- 通过适配器组合驱动 mixed-protocol 双总线。

## 当前状态

- 提供 `MixedRobotDriver` 骨架。
- 提供 `hardware_control_services.py`：
  - `/robot_arm/hardware/*` 最小真实服务入口
  - 发布 `/robot_arm/control/state_json` 统一状态
  - 提供单关节最小真实写入测试入口
- joint 路由映射由 model/config 注入，避免 core 硬编码机型映射。
- 当前不是最终 ros2_control hardware plugin。
