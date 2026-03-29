# robot_arm_bringup_4338

4338 机型最小组合启动包（轻量）。

## 目标

未来统一编排以下层：

- robot_arm_description_4338
- robot_arm_moveit_config_4338
- fake / real ros2_control hardware
- robot_arm_core
- robot_arm_safety
- robot_arm_web_bridge

## 本轮范围

提供最小 bringup 组合：

- MoveIt 配置（`robot_arm_moveit_config_4338`）
- fake / real ros2_control 切换入口
- 自动启动 `robot_arm_core/hardware_control_services`
- 自动启动 `robot_arm_web_bridge/ros_control_entry_server`
- 自动启动 `robot_arm_web_bridge/mock_api_server`

## 运行方式（示例）

```bash
ros2 launch robot_arm_bringup_4338 bringup.launch.py use_fake_hardware:=true
```

切换 real 路径（结构级）：

```bash
ros2 launch robot_arm_bringup_4338 bringup.launch.py use_fake_hardware:=false backend_mode:=real
```

启动日志会打印参数链和期望 plugin（fake 或 real），用于快速核对。

可按需关闭节点：

```bash
ros2 launch robot_arm_bringup_4338 bringup.launch.py start_hardware_services:=false start_bridge_nodes:=false
```

## 硬件服务与 bridge 启动（当前最小）

```bash
PYTHONPATH=ros2_ws/src/robot_arm_core:ros2_ws/src/robot_arm_web_bridge:ros2_ws/src/robot_arm_hardware_hightorque_canfd:ros2_ws/src/robot_arm_hardware_yiyou_can20a \
python3 -m robot_arm_core.hardware_control_services
```

```bash
PYTHONPATH=ros2_ws/src/robot_arm_web_bridge \
python3 -m robot_arm_web_bridge.mock_api_server
```
