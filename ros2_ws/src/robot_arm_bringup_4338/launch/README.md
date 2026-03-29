# bringup launch

本目录提供最小 `bringup.launch.py`：

- 组合 `robot_arm_moveit_config_4338/launch/demo.launch.py`
- 暴露 fake/real 切换参数：
  - `use_fake_hardware`
  - `backend_mode`
  - `joint_mapping_path`
  - `start_hardware_services`
  - `start_bridge_nodes`

默认会启动：

- `robot_arm_core/hardware_control_services`
- `robot_arm_web_bridge/ros_control_entry_server`
- `robot_arm_web_bridge/mock_api_server`
