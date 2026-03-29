# web_ui

前端壳子（React + Vite + TypeScript）。

## 本轮范围

- 保持中英文切换与现有布局。
- 首页消费 web bridge 的统一状态契约（`GET /state`）。
- armed 控制改为单一胶囊式 switch（非 checkbox 风格）。
- `/enable` / `/disable` 使用 request lifecycle（idle/in_progress/success/failed）语义回传。
- 状态支持 loading / unavailable / error 兜底。
- teach sequence 卡片展示 mock steps。

## 接口依赖

默认请求：`http://localhost:8090`

可通过环境变量覆盖：

```bash
VITE_WEB_BRIDGE_BASE_URL=http://localhost:8090
```

## 运行

```bash
# 终端1：启动 mock API
python3 ../ros2_ws/src/robot_arm_web_bridge/robot_arm_web_bridge/mock_api_server.py

# 终端2：启动前端
npm install
npm run dev
```
