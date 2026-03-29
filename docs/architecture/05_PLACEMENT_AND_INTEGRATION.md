# 放到项目里的位置

根据当前项目说明，协议分裂必须只留在 **hardware adapter layer**，上层仍然看见统一 6 轴；而仓库在 `ros2_ws/src/` 下已经有（或曾计划有）协议专属硬件包：高擎 `robot_arm_hardware_hightorque_canfd`、意优 `robot_arm_hardware_yiyou_can20a`。因此这两个底层文件建议直接放在各自的硬件适配包里，而不是放到 `robot_arm_core`。  

## 推荐路径

### 高擎
`ros2_ws/src/robot_arm_hardware_hightorque_canfd/robot_arm_hardware_hightorque_canfd/hightorque_low_level.py`

### 意优
`ros2_ws/src/robot_arm_hardware_yiyou_can20a/robot_arm_hardware_yiyou_can20a/yiyou_low_level.py`

## 为什么放这里

- `robot_arm_core` 在项目说明里被定义为“通用机械臂核心逻辑”，不应该写死当前 bus / id 拓扑或当前协议。  
- README 明确说当前机械臂是 **mixed-protocol dual-bus**：`joint_1/3/4/5/6` 走 HighTorque CAN-FD，`joint_2` 走 Yiyou CAN 2.0A；协议拆分只能留在 model layer 和 hardware adapter layer。  
- 你们之前第一轮骨架也已经把 `robot_arm_hardware_hightorque_canfd` 和 `robot_arm_hardware_yiyou_can20a` 作为协议专属包边界。  

## Codex 后续应该怎么用它们

不要把这两个文件当成新的 ROS 包。  
更稳的做法是：

1. 让 Codex 保留现有包边界。
2. 在各自 adapter 包里：
   - 原有 `hightorque_fdcan_adapter.py` 改为更薄的一层，对外暴露较高层动作；
   - 由它内部调用 `hightorque_low_level.py`。
   - 原有 `yiyou_can_adapter.py` 同理内部调用 `yiyou_low_level.py`。
3. 上层 `robot_arm_hardware_system` / `robot_arm_core` 只调用 adapter，不直接碰串口帧字节。

## 这两个文件当前各自覆盖的范围

### `hightorque_low_level.py`
只覆盖当前已经实测打通的一组：
- bridge init
- tint16 query
- stop
- brake
- mode2 位置不限 + 速度控制
- tint16 状态解析

### `yiyou_low_level.py`
只覆盖当前已经实测打通的一组：
- bridge init
- read position
- read mode
- read enable state
- enable / disable
- stop
- target speed / target position 写入
- 标准回包解析

## 你上传给 Codex 时的注意点

- 不要让 Codex 再复制一份第二套协议实现。
- 不要让 Codex 把协议逻辑放回 `robot_arm_core`。
- 不要让 Codex 先删 `hardware_control_services.py`。
- 优先让 Codex 用这两个低层文件替换旧 adapter 内部的手拼字符串逻辑。
