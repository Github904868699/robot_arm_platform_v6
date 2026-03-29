# bringup_integration_plan.md

## 本轮 bringup 组合了哪些层

新增 `robot_arm_bringup_4338` 最小组合启动，串起：

- `robot_arm_moveit_config_4338`（MoveIt/ros2_control）
- `robot_arm_hardware_system`（通过 ros2_control plugin）
- 预留 `robot_arm_web_bridge` 与控制入口服务的外部接入位

## fake / real 切换方式

通过 launch 参数：

- `use_fake_hardware:=true`（默认）
- `use_fake_hardware:=false`（走 real backend）
- `backend_mode:=fake|real`
- `joint_mapping_path:=.../joint_mapping.example.yaml`

`MoveIt` 与 controller 侧保持不变，只切底层 backend。

## MoveIt 为什么仍只看统一 6 轴

MoveIt 仍只处理：

- `joint_1 ~ joint_6`

不感知 bus/node/protocol/vendor。混合协议差异只在 model/routing/backend 中处理。

## 当前距离真机执行还差什么

1. `RealMixedRobotBackend` 的帧级真实 read 完整实现（Hightorque/Yiyou）。
2. `robot_arm_hardware_system` 控制入口与真实硬件服务的全链路对接。
3. 写运动闭环（本轮仍禁止）。
4. 异常与恢复策略的生产级完善（fault/recover/timeout/丢包）。

## 服务启动路径（当前最小）

硬件服务节点：

```bash
PYTHONPATH=ros2_ws/src/robot_arm_core:ros2_ws/src/robot_arm_web_bridge:ros2_ws/src/robot_arm_hardware_hightorque_canfd:ros2_ws/src/robot_arm_hardware_yiyou_can20a \
python3 -m robot_arm_core.hardware_control_services
```

Web bridge API：

```bash
PYTHONPATH=ros2_ws/src/robot_arm_web_bridge \
python3 -m robot_arm_web_bridge.mock_api_server
```
