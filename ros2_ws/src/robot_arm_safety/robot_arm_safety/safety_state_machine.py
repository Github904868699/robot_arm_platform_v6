"""Safety state machine prototype.

Focus: startup safety and explicit enable flow.
"""

from __future__ import annotations

from enum import Enum


class SafetyState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    DISCOVERING = "DISCOVERING"
    SYNCING_POSITION = "SYNCING_POSITION"
    READY_UNARMED = "READY_UNARMED"
    ARMING = "ARMING"
    ARMED_HOLDING_CURRENT = "ARMED_HOLDING_CURRENT"
    EXECUTING = "EXECUTING"
    FAULT = "FAULT"


class SafetyStateMachine:
    """Minimal guard-only state machine for command gating."""

    def __init__(self) -> None:
        self.state = SafetyState.DISCONNECTED

    def can_accept_motion(self) -> bool:
        return self.state in {SafetyState.ARMED_HOLDING_CURRENT, SafetyState.EXECUTING}

    def require_motion_enabled(self) -> None:
        if not self.can_accept_motion():
            raise RuntimeError("Motion rejected: explicit enable is required before movement.")
