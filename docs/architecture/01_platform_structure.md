
---

## 3. `docs/architecture/01_platform_structure.md`

```md
# 01_platform_structure.md

## Purpose

This document defines the intended repository structure for `ros_arm_platform`.

The project is designed as a reusable robot arm platform, with clear separation between:

- platform-level reusable logic
- current 4338 arm model
- current hardware/protocol adapters
- front-end web application

This separation exists so that future robot arms can be supported by adding new model and hardware packages, instead of rewriting the entire system.

---

## Top-Level Structure

```text
ros_arm_platform/
├─ AGENTS.md
├─ README.md
├─ assets/
├─ configs/
├─ docs/
│  ├─ architecture/
│  └─ vendor/
├─ ros2_ws/
│  └─ src/
│     ├─ robot_arm_core/
│     ├─ robot_arm_interfaces/
│     ├─ robot_arm_safety/
│     ├─ robot_arm_teach/
│     ├─ robot_arm_web_bridge/
│     ├─ robot_arm_description_4338/
│     ├─ robot_arm_moveit_config_4338/
│     ├─ robot_arm_model_4338/
│     ├─ robot_arm_hardware_hightorque_canfd/
│     ├─ robot_arm_hardware_yiyou_can20a/
│     └─ robot_arm_bringup_4338/
├─ tools/
└─ web_ui/