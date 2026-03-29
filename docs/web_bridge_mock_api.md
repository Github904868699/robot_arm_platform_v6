# web_bridge_mock_api.md

## mock API 列表

- `GET /state`
- `GET /joints`
- `POST /enable`
- `POST /disable`
- `POST /stop`
- `POST /clear_fault`
- `POST /fault`
- `POST /recover`
- `GET /mode`
- `GET /teach/sequence`
- `POST /teach/sequence`

## `/state` 统一状态契约

```json
{
  "robot_state": "READY_UNARMED",
  "connection_state": "CONNECTED",
  "armed": false,
  "mode": "Control",
  "cycle_ms": 8.3,
  "loss_percent": 0.0,
  "error_message": null,
  "can_retry": false,
  "request": {
    "action": "none",
    "status": "idle",
    "message": null
  },
  "joints": {
    "joint_1": 0.02,
    "joint_2": -0.31,
    "joint_3": 0.88,
    "joint_4": -0.41,
    "joint_5": 0.11,
    "joint_6": 0.63
  }
}
```

## 控制请求响应（/enable /disable /stop /clear_fault /fault /recover）

```json
{
  "request": {
    "action": "enable",
    "status": "success",
    "message": null
  },
  "state": { "...": "same as /state contract" }
}
```

request status 约定：
- `idle`
- `in_progress`
- `success`
- `failed`

## 控制语义约束

- `enable`：仅 `READY_UNARMED` + `CONNECTED` 允许
- `disable`：`ARMED_HOLDING_CURRENT / EXECUTING / READY_UNARMED` 允许
- `stop`：任意状态可触发，进入 `FAULT`
- `clear_fault`：清除 fault 标记并回到 `READY_UNARMED`
- `fault`：进入 `FAULT`，并置 `armed=false`
- `recover`：仅 `FAULT` + `CONNECTED` 允许，恢复到 `READY_UNARMED`

## /state 与 /joints 的状态来源

`/state`、`/joints` 现在优先读取 ROS2 topic `/robot_arm/control/state_json`。

若 ROS2 状态暂不可用，会 fallback 到本地占位契约，响应 header 标注：

- `X-Bridge-State-Source: ros2`
- `X-Bridge-State-Source: fallback`

## 控制请求对应的 ROS2 硬件服务

- `POST /enable` -> `/robot_arm/hardware/enable`
- `POST /disable` -> `/robot_arm/hardware/disable`
- `POST /stop` -> `/robot_arm/hardware/stop`
- `POST /clear_fault` -> `/robot_arm/hardware/clear_fault`
- `POST /recover` -> `/robot_arm/hardware/recover`

## 为什么前端不直接感知协议细节

前端只应处理高层语义（状态/模式/任务步骤），不应暴露 bus/node/protocol/vendor。

## 后续接 ROS2 真状态方式

1. 保持 API 字段稳定。
2. 在 web bridge 内把 mock 状态源替换成 ROS2 真状态。
3. enable/disable 接口再逐步映射到真实控制后端。
