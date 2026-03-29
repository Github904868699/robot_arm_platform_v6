
---

## `docs/architecture/02_startup_enable_logic.md`

```md
# 02_startup_enable_logic.md

## Purpose

This document defines the required startup behavior, enable/disable semantics, and runtime state machine.

This is one of the most important design constraints in the project.

The robot must feel:
- calm
- predictable
- safe
- non-aggressive on startup

---

## Core Principle

### Startup must not actively move the robot

The system must never do any of the following automatically at startup:
- move to home
- move to work
- re-zero by motion
- automatically hold with a stale target
- suddenly become stiff without user confirmation

---

## Required Startup Sequence

### Stage 1: Communication bring-up
- open communication interfaces
- open all expected buses
- start discovery only
- do not send motion commands

### Stage 2: Joint discovery
- detect expected joints
- detect whether current model mapping is consistent
- verify joint presence per expected bus

### Stage 3: Position synchronization
- read current position of all detected joints
- synchronize robot state
- publish current state
- synchronize model / joint state representation

### Stage 4: Ready but unarmed
Once all required joints are discovered and synchronized:
- enter `READY_UNARMED`
- remain readable
- remain visible
- remain non-actuated

No motion should be allowed yet.

---

## Enable Philosophy

### Discovery is not permission to take over
Even if all joints are found:
- do not auto-enable
- do not auto-lock
- do not auto-hold
- do not move toward a remembered target

### User confirmation is required
Only after explicit user confirmation should the robot enter a controlled hold state.

---

## Hold Strategy

When the user presses enable:

### Correct behavior
- use the currently read joint positions as hold targets
- smoothly enter holding state
- avoid jumps or discontinuities
- avoid snapping toward old targets

### Incorrect behavior
- move toward last command from previous run
- jump toward home
- jump toward work
- immediately apply unexpected position control

---

## Recommended Runtime States

### `DISCONNECTED`
Meaning:
- no valid bus connection
- no valid joint presence
- no synchronized robot state

### `DISCOVERING`
Meaning:
- communication opened
- looking for expected joints
- no synchronized 6-joint state yet

### `SYNCING_POSITION`
Meaning:
- joints are appearing
- current positions are being read
- state is not fully ready yet

### `READY_UNARMED`
Meaning:
- expected joints found
- current positions synchronized
- model state valid
- robot visible and readable
- still not enabled
- no motion commands allowed

### `ARMING`
Meaning:
- user requested enable
- system is transitioning into hold using current state as target

### `ARMED_HOLDING_CURRENT`
Meaning:
- robot is enabled
- robot is holding current pose
- motion commands may become available

### `EXECUTING`
Meaning:
- robot is executing:
  - jog
  - home
  - work
  - teach playback
  - MoveIt trajectory

### `FAULT`
Meaning:
- hardware fault
- bus fault
- missing joint
- protocol error
- invalid configuration mismatch
- any unsafe runtime issue

---

## Required Command Rules

### Before enable
The following must be rejected:
- joint motion commands
- cartesian jog commands
- trajectory execution
- home/work execution
- teach execution

Allowed:
- state reading
- UI display
- diagnostics
- discovery
- synchronization

### After enable
Allowed:
- hold current pose
- explicit motion commands
- planning execution
- teach playback

---

## Why this matters

This behavior is critical for:
- safety
- product quality
- user trust
- future teach mode
- future web UI design

The robot should feel like:
- it wakes up
- it knows where it is
- it waits for permission
- it does not seize control unexpectedly

---

## UI Mapping

### In `READY_UNARMED`
UI should show:
- robot online
- current pose known
- controls still disabled
- enable action available

### In `ARMED_HOLDING_CURRENT`
UI should show:
- robot enabled
- controls unlocked
- robot holding safely
- ready for jog/home/work/execute

---

## Non-Negotiable Rules

1. Startup must not actively move the robot
2. Discovery must not automatically enable the robot
3. Discovery must not automatically lock the robot
4. Enable must use current pose as hold target
5. Motion must only be allowed after explicit enable
6. Faults must force exit from executable states

---

## Long-Term Benefit

This startup logic supports future features cleanly:
- web UI enable flow
- teach mode
- safe remote control
- multi-device integration
- mixed hardware support
