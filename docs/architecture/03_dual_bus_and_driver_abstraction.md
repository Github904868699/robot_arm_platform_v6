
---

## 4. `docs/architecture/03_dual_bus_and_driver_abstraction.md`

```md
# 03_dual_bus_and_driver_abstraction.md

## Purpose

This document defines how the current robot's mixed bus / mixed protocol hardware should be abstracted in software.

The system must present a unified 6-joint robot to upper layers, while internally supporting different bus types and protocol families.

---

## Current Hardware Reality

This robot is a mixed-protocol dual-bus system.

### Hightorque side
- `joint_1`
- `joint_3`
- `joint_4`
- `joint_5`
- `joint_6`
- protocol: `CAN-FD`
- motor: Hightorque `HTDW-4438-30-NE`

### Yiyou side
- `joint_2`
- protocol: `CAN 2.0A`
- motor: Yiyou `PP` series planetary joint

---

## Immediate Consequence

This is not a dual-CANFD-only topology.

It must be represented in software as:
- one Hightorque CAN-FD adapter
- one Yiyou CAN 2.0A adapter
- one unified 6-joint robot abstraction above them

The two buses must not be merged in software abstraction.

---

## Upper-Layer Rule

Upper layers must **not** know any of the following:
- which joint uses which protocol
- which joint sits on which bus
- Hightorque packet formats
- Yiyou register maps
- raw node ids

Upper layers should only know:
- `joint_1`
- `joint_2`
- `joint_3`
- `joint_4`
- `joint_5`
- `joint_6`

---

## Model Layer Responsibility

`robot_arm_model_4338` should own mapping information such as:

```yaml
joint_1:
  driver: hightorque_canfd
  bus: canfd_main
  node_id: 1

joint_2:
  driver: yiyou_can20a
  bus: can_j2
  node_id: 2

joint_3:
  driver: hightorque_canfd
  bus: canfd_main
  node_id: 3

joint_4:
  driver: hightorque_canfd
  bus: canfd_main
  node_id: 4

joint_5:
  driver: hightorque_canfd
  bus: canfd_main
  node_id: 5

joint_6:
  driver: hightorque_canfd
  bus: canfd_main
  node_id: 6