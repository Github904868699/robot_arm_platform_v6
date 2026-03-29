
---

## 5. `docs/protocol_validation.md`

```md
# protocol_validation.md

## Purpose

This document records **real hardware bring-up facts** that have already been manually verified on Linux.

It is more authoritative than assumptions derived from legacy code.

---

## Current Bus Reality

The current robot is a mixed-protocol dual-bus system.

### Bus A
- device node: `/dev/ttyACM0`
- protocol family: Hightorque CAN-FD
- joints observed: `joint_1`, `joint_3`, `joint_4`, `joint_5`, `joint_6`

### Bus B
- device node: `/dev/ttyACM1`
- protocol family: Yiyou CAN 2.0A
- joint observed: `joint_2`

---

## Verified Serial Bridge Initialization

### Yiyou / joint_2 side
Device:
- `/dev/ttyACM1`

Manual session:
```text
C
S8
M0
A1
O
