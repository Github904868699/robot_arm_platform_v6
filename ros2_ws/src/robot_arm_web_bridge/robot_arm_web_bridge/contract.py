"""High-level state contract shared by mock bridge and web clients.

This contract intentionally avoids low-level protocol fields (bus/node/vendor/etc).
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Dict, Optional

ROBOT_STATES = {
    "DISCONNECTED",
    "DISCOVERING",
    "SYNCING_POSITION",
    "READY_UNARMED",
    "ARMING",
    "ARMED_HOLDING_CURRENT",
    "EXECUTING",
    "FAULT",
    "UNAVAILABLE",
}

CONNECTION_STATES = {"CONNECTED", "DISCONNECTED", "UNAVAILABLE"}
REQUEST_LIFECYCLE = {"idle", "in_progress", "success", "failed"}


@dataclass
class ControlRequestStatus:
    action: str = "none"
    status: str = "idle"  # idle | in_progress | success | failed
    message: Optional[str] = None


@dataclass
class RobotControlContract:
    robot_state: str = "READY_UNARMED"
    connection_state: str = "CONNECTED"
    armed: bool = False
    mode: str = "Control"
    cycle_ms: float = 8.3
    loss_percent: float = 0.0
    error_message: Optional[str] = None
    can_retry: bool = False
    request: ControlRequestStatus = field(default_factory=ControlRequestStatus)
    joints: Dict[str, float] = field(
        default_factory=lambda: {
            "joint_1": 0.02,
            "joint_2": -0.31,
            "joint_3": 0.88,
            "joint_4": -0.41,
            "joint_5": 0.11,
            "joint_6": 0.63,
        }
    )

    def to_dict(self) -> dict:
        return asdict(self)

    def normalize(self) -> None:
        if self.robot_state not in ROBOT_STATES:
            self.robot_state = "UNAVAILABLE"
        if self.connection_state not in CONNECTION_STATES:
            self.connection_state = "UNAVAILABLE"
        if self.request.status not in REQUEST_LIFECYCLE:
            self.request.status = "failed"
        if self.robot_state == "FAULT":
            self.armed = False
