# test_sequence_for_real_read_and_single_write.md

## 建议测试顺序（最小）

1. **bringup fake**
   - 先用 fake 路径确认 MoveIt 与控制链可启动。

2. **启动真实硬件服务节点（只读+最小写）**
   - 启动 `robot_arm_core.hardware_control_services`。

3. **bringup real readonly**
   - `use_fake_hardware:=false`
   - `backend_mode:=real`
   - 带 `joint_mapping_path`。

4. **验证 `/state`**
   - 调用 web bridge `/state`
   - 检查 `X-Bridge-State-Source` 是否为 `ros2`。

5. **验证 enable**
   - 调用 `/robot_arm/hardware/enable`
   - 确认状态进入 `ARMED_HOLDING_CURRENT`。

6. **验证单关节最小真实写入**
   - 调用 `/robot_arm/hardware/write_single_joint_step`
   - 仅允许一个关节、最小步进。

7. **明确禁止**
   - 不执行多关节真实 MoveIt trajectory
   - 不执行持续真实闭环写控制

