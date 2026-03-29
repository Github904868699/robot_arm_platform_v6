# 04_web_and_motion_task_model.md

## Purpose

This document defines how the future Web UI, teach workflow, and motion task model should be designed.

The goal is **not** to make the Web frontend directly act like MoveIt or directly send low-level protocol commands.

Instead, the Web layer should provide a **product-level task editing and control experience**, while the backend translates these actions into:
- MoveIt planning requests
- ros2_control execution
- hardware adapter commands
- future IO actions

---

## Core Principle

### The Web UI should express robot intent, not bus/protocol details

The frontend should not know:
- CAN-FD frame formats
- CAN 2.0A register maps
- node ids
- which bus each joint belongs to
- raw motor enable frames

The frontend should only know:
- robot state
- current mode
- current joint state
- current TCP pose
- editable motion/task steps
- whether the robot is enabled
- whether the system is ready or in fault

---

## Web Product Goals

The future Web UI should support:

1. clear robot overview
2. high-end product-style layout
3. teach point recording
4. point list editing
5. per-step speed / acceleration settings
6. dwell time at a point
7. future IO insertion
8. future circular motion insertion
9. eventual intelligent mode entry point
10. separation between product mode and advanced/developer mode

---

## Layout Intent

The Web UI should follow the layout direction already discussed:

### Top Center
Main mode selector:
- Control
- Teach
- Visual
- Intelligent

This should behave like a segmented control / primary mode switch.

### Center
Large 3D robot view:
- visual focus of the system
- current pose display
- later can include tool/workspace overlays
- should not be crowded with noisy debug information

### Left Side
Passive state card(s):
- joint_1 ~ joint_6 values
- current robot state
- connection status
- enabled/unarmed/fault state
- optional IO status summary later

### Right Side
Active control cards:
- Cartesian control card
- teach card
- motion sequence card
- IO card placeholder
- advanced card(s)

---

## Separation of Concerns

### Web layer
Responsible for:
- UI
- editing
- mode switching
- point recording requests
- displaying state
- editing motion/task steps

### Web bridge / backend layer
Responsible for:
- API translation
- validation
- state machine enforcement
- MoveIt calls
- ros2 actions/services
- hardware-safe execution dispatch

### Motion backend
Responsible for:
- converting task steps into executable motion segments
- translating MoveJ / MoveL / MoveC / Wait / SetIO
- selecting correct planning/execution path

---

## Teach Workflow Philosophy

The Web teach system should not just be a low-level joint panel.

It should feel like a real robot task editor.

### Expected flow
1. user moves robot manually or via jog
2. user records current point
3. system stores:
   - joint values
   - optional TCP pose
   - metadata
4. user edits point properties
5. user inserts movement types and non-motion steps between points
6. user executes sequence safely

---

## Recommended Task Model

The system should use a **custom motion task model** instead of storing raw MoveIt requests directly.

This makes the system:
- readable
- editable
- portable
- future-proof

---

## Recommended Motion/Task Step Types

### MoveJ
Joint-space movement.

Use when:
- posture change matters more than exact Cartesian path
- fast repositioning is desired

Typical parameters:
- target joint values
- velocity scaling
- acceleration scaling
- blend radius (future)
- optional label

### MoveL
Linear Cartesian movement.

Use when:
- TCP should move along a straight line
- path shape matters

Typical parameters:
- target TCP pose
- velocity scaling
- acceleration scaling
- blend radius (future)
- optional label

### MoveC
Circular movement / arc segment.

Use when:
- circular arc is desired
- process path needs smooth curved motion
- future industrial trajectory style is needed

Typical parameters:
- start pose (implicit or previous point)
- via point or arc definition point
- target pose
- velocity scaling
- acceleration scaling
- optional direction / arc settings
- optional label

Note:
- backend should ideally map this to Pilz CIRC where applicable
- Web stores it as a task-level motion type, not as a raw planner-specific request

### Wait
Dwell / pause / hold.

Use when:
- robot must stop and remain still for a specified time
- process requires time at a position

Typical parameters:
- duration (seconds)
- optional label / note

Important:
- this should be treated as a task-level step
- not as part of motion planning itself

### SetIO
IO output action.

Use when:
- turn on gripper
- turn off gripper
- trigger vacuum
- toggle digital output
- future process synchronization

Typical parameters:
- channel or symbolic IO name
- target value
- optional delay
- optional label

Even before IO is implemented, the Web UI should reserve this type in the task model.

### Comment
Non-executable annotation.

Use when:
- user wants to describe a step
- debugging or process documentation is useful

Typical parameters:
- free text

---

## Suggested Internal Data Shape

An example task sequence could conceptually look like:

```json
[
  {
    "type": "MoveJ",
    "name": "Go to pre-pick",
    "target_joints": {
      "joint_1": 0.0,
      "joint_2": 0.4,
      "joint_3": -0.2,
      "joint_4": 0.0,
      "joint_5": 0.1,
      "joint_6": 0.0
    },
    "velocity_scale": 0.3,
    "acceleration_scale": 0.3
  },
  {
    "type": "Wait",
    "name": "Settle",
    "duration": 0.5
  },
  {
    "type": "SetIO",
    "name": "Gripper on",
    "channel": "gripper_enable",
    "value": true
  },
  {
    "type": "MoveL",
    "name": "Lift",
    "target_pose": {
      "x": 0.42,
      "y": 0.10,
      "z": 0.35,
      "roll": 3.14,
      "pitch": 0.0,
      "yaw": 1.57
    },
    "velocity_scale": 0.15,
    "acceleration_scale": 0.15
  },
  {
    "type": "MoveC",
    "name": "Arc transfer",
    "via_pose": {
      "x": 0.50,
      "y": 0.12,
      "z": 0.33,
      "roll": 3.14,
      "pitch": 0.0,
      "yaw": 1.57
    },
    "target_pose": {
      "x": 0.58,
      "y": 0.18,
      "z": 0.29,
      "roll": 3.14,
      "pitch": 0.0,
      "yaw": 1.57
    },
    "velocity_scale": 0.2,
    "acceleration_scale": 0.2
  }
]
```

This is only an example shape, not a mandatory exact schema.

---

## Recording Current Point

When the user presses "Record Current Point", the system should store:

### Required
- current joint positions
- current timestamp
- current mode context

### Recommended
- computed TCP pose
- current frame/tool context
- whether robot was armed
- optional label generated by UI

Why store joint values even if you also store TCP pose?
- joint values are reliable for reproducing exact robot posture
- TCP pose is useful for Cartesian editing and user readability

The backend can keep both.

---

## Point Editing

The teach editor should allow editing:

### Joint view
- joint_1 ~ joint_6

### Cartesian view
- x / y / z
- roll / pitch / yaw

### Motion parameters
- velocity scaling
- acceleration scaling
- blend radius (future)
- dwell time (through Wait)
- IO insertion
- motion type switch (MoveJ, MoveL, MoveC)

---

## Relationship with MoveIt

### Important rule
The Web task model is not equal to MoveIt requests.

Instead:

- MoveJ may be translated into joint-space planning / execution
- MoveL may be translated into linear planning logic
- MoveC may be translated into Pilz CIRC
- Wait stays in task executor
- SetIO goes to IO subsystem

This keeps the task model stable even if internal planning/execution implementation changes later.

---

## Why this is better than storing raw planner requests

If raw planner-specific requests are stored directly:
- the frontend becomes tightly coupled to one backend planner
- editing becomes harder
- IO and wait steps become awkward
- portability decreases
- future migration becomes painful

A custom task model avoids that.

---

## Speed / Acceleration Ownership

These should exist in two layers:

### Global/system limits
Defined by:
- model config
- MoveIt joint limits
- hardware constraints

### Per-step execution parameters
Defined by:
- task step
- velocity scaling
- acceleration scaling

This means:
- user can tune a specific step
- system still enforces safe upper bounds

---

## IO Placeholder Strategy

Even before IO is physically implemented, the system should reserve:

### UI
- an IO card in the right-side panel
- disabled "future" controls if needed
- visual placeholders for IO-capable task steps

### Backend/task model
- SetIO
- optional future WaitIO

### Model/config
- IO channel symbolic names can later live in model layer

This prevents future IO support from feeling bolted on.

---

## Circular Motion Strategy

For circular motion, recommended strategy is:

### Frontend
Store it as:
- MoveC
- with explicit arc parameters

### Backend
Translate to:
- Pilz CIRC when supported
- or fallback custom implementation later if necessary

This lets the task model remain stable even if planner backend changes.

---

## Product Mode vs Advanced Mode

The Web should eventually support two presentation styles:

### Product Mode
- clean
- guided
- minimal
- safe
- fewer visible low-level controls

### Advanced / Developer Mode
- more technical information
- more editable parameters
- optional card rearrangement
- diagnostics visibility
- joint-level details

This distinction matches the project’s overall product direction.

---

## Recommended Initial Web Scope

Before real hardware control is fully integrated, the Web can already implement:

- mode switcher
- main 3D layout
- passive state cards
- task sequence editor shell
- MoveJ / MoveL / MoveC / Wait / SetIO card types
- fake/mock robot state
- fake/mock point recording
- editable speed / acceleration / dwell fields

This provides high-value product progress without coupling too early to hardware bring-up.

---

## Summary

The Web layer should become a **robot task editor and product UI**, not a thin wrapper around low-level robot commands.

The key design choice is:

- store user intent as task steps
- let backend translate those steps to MoveIt, ros2_control, and hardware-specific execution

This is what allows:
- teach mode
- future IO
- circular motion
- future intelligent mode
- future portability
