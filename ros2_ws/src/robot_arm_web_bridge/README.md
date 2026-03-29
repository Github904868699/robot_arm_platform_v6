# robot_arm_web_bridge

Web 后端桥接层。

## 职责

- 对外提供高层 API（HTTP/WebSocket）
- 把 Web 任务模型翻译为 ROS2 接口调用（后续）
- 执行 enable/state/motion gating
- 隔离 Web 与底层 bus/protocol 细节

## 本轮提供：统一状态契约 + ROS2优先状态桥接 API

- 状态契约：`robot_arm_web_bridge/contract.py`
- API server：`robot_arm_web_bridge/mock_api_server.py`
- ROS2 状态连接层：`robot_arm_web_bridge/ros_state_connector.py`
- 硬件服务节点由 `robot_arm_core/hardware_control_services.py` 提供

启动：

```bash
python3 ros2_ws/src/robot_arm_web_bridge/robot_arm_web_bridge/mock_api_server.py
```

默认地址：`http://localhost:8090`

## 当前 mock 接口

- `GET /state`（优先 ROS2 状态，fallback 到本地占位；header: `X-Bridge-State-Source`）
- `GET /joints`（同上，来源可见 `X-Bridge-State-Source`）
- `POST /enable`（返回 `request + state`）
- `POST /disable`（返回 `request + state`）
- `POST /stop`（返回 `request + state`）
- `POST /clear_fault`（返回 `request + state`）
- `POST /fault`（mock 注入故障）
- `POST /recover`（FAULT -> READY_UNARMED）
- `GET /mode`
- `GET /teach/sequence`
- `POST /teach/sequence`

返回字段只包含逻辑层语义，不暴露 bus/node/protocol/vendor 字段。

## ROS2 硬件控制入口（最小）

- `/robot_arm/hardware/enable`
- `/robot_arm/hardware/disable`
- `/robot_arm/hardware/recover`
- `/robot_arm/hardware/stop`
- `/robot_arm/hardware/clear_fault`

均为 `std_srvs/Trigger`，语义与状态机一致。
