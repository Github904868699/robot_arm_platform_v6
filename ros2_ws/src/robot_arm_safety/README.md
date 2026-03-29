# robot_arm_safety

安全层（最小状态机说明 + 骨架）。

## 非协商约束

1. 启动不主动运动。
2. 扫描到全部关节后只进入 `READY_UNARMED`。
3. discovery != enable。
4. enable 后必须以“当前位置”建立 holding target。
5. 未使能状态拒绝 motion commands（jog/trajectory/cartesian/home/work/teach playback）。

## 状态机（最小）

- `DISCONNECTED`
- `DISCOVERING`
- `SYNCING_POSITION`
- `READY_UNARMED`
- `ARMING`
- `ARMED_HOLDING_CURRENT`
- `EXECUTING`
- `FAULT`

后续可在本包加入可测试的状态机实现与守护条件。
