# web_bridge_ros_state_integration.md

## 目标

让 `web_bridge` 的 `/state`、`/joints` 不再只依赖本地 mock，而是优先消费 ROS2 的统一高层状态。

## 当前状态来源

`robot_arm_web_bridge/mock_api_server.py` 现在通过 `RosStateConnector` 读取：

- topic: `/robot_arm/control/state_json`
- 由 `ros_control_entry_server.py` 发布统一高层状态 JSON

并通过 HTTP header 标注来源：

- `X-Bridge-State-Source: ros2`
- `X-Bridge-State-Source: fallback`

## mock 与 real 的关系

- **real优先**：如果 ROS2 状态与控制服务可用，bridge 输出 ROS2 状态。
- **fallback兜底**：若 ROS2 不可用，bridge 维持同一契约并在 header 中明确来源为 `fallback`。
- 这样可保证前端 API 稳定，且不会把 ROS2 细节泄漏到前端。

## 为什么前端仍只看统一高层状态

前端继续只消费：

- `robot_state`
- `connection_state`
- `armed`
- `mode`
- `cycle_ms`
- `loss_percent`
- `error_message`
- `can_retry`
- `request`
- `joints`

前端不感知 bus/node/protocol/vendor。混合协议细节仍留在 backend/adapter/routing 层。

## 当前控制服务对接

bridge 控制请求当前优先调用：

- `/robot_arm/hardware/enable`
- `/robot_arm/hardware/disable`
- `/robot_arm/hardware/recover`
- `/robot_arm/hardware/stop`
- `/robot_arm/hardware/clear_fault`
