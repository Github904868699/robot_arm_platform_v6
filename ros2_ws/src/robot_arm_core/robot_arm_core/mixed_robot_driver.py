"""Unified mixed-protocol robot driver prototype.

This is a minimal prototype layer, not a final ros2_control hardware plugin.
Upper layers should only see logical joints: joint_1 ~ joint_6.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


class DriverState(str, Enum):
    DISCOVERING = "DISCOVERING"
    SYNCING_POSITION = "SYNCING_POSITION"
    READY_UNARMED = "READY_UNARMED"
    ARMED_HOLDING_CURRENT = "ARMED_HOLDING_CURRENT"
    EXECUTING = "EXECUTING"
    FAULT = "FAULT"


@dataclass
class JointRoute:
    adapter: str
    node_id: int


class MixedRobotDriver:
    """Unified 6-joint driver facade over two protocol adapters.

    Notes:
    - Routing must come from model/config layer.
    - Core should not hardcode model-specific bus/node mapping.
    """

    def __init__(self, hightorque_adapter: Any, yiyou_adapter: Any, joint_routes: dict[str, JointRoute]) -> None:
        self.hightorque = hightorque_adapter
        self.yiyou = yiyou_adapter
        self.state = DriverState.DISCOVERING
        self._routes = joint_routes

    def connect_all(self) -> None:
        """Open both transports and initialize bridges."""
        self.hightorque.open()
        self.hightorque.initialize_bridge()
        self.yiyou.open()
        self.yiyou.initialize_bridge()
        self.state = DriverState.DISCOVERING

    def discover(self) -> dict[str, bool]:
        """Discover joints without enabling or locking robot."""
        self.state = DriverState.DISCOVERING
        ht_ids = set(self.hightorque.scan())
        discovered = {
            joint_name: (route.node_id in ht_ids if route.adapter == "hightorque" else True)
            for joint_name, route in self._routes.items()
        }
        self.state = DriverState.SYNCING_POSITION
        return discovered

    def read_joint_states(self) -> dict[str, dict]:
        """Read current state snapshot for all logical joints."""
        snapshot: dict[str, dict] = {}
        for joint_name, route in self._routes.items():
            if route.adapter == "hightorque":
                snapshot[joint_name] = self.hightorque.read_joint_state(route.node_id)
            else:
                snapshot[joint_name] = {
                    "node_id": route.node_id,
                    "position": self.yiyou.read_position().get("position"),
                    "velocity": self.yiyou.read_velocity().get("velocity"),
                }
        self.state = DriverState.READY_UNARMED
        return snapshot

    def enable_robot(self) -> None:
        """Explicitly enable robot and enter hold-current state."""
        for joint_name, route in self._routes.items():
            if route.adapter == "hightorque":
                self.hightorque.enable(route.node_id)
            elif route.adapter == "yiyou":
                self.yiyou.enable()
        self.hold_current()
        self.state = DriverState.ARMED_HOLDING_CURRENT

    def disable_robot(self) -> None:
        """Disable all joints and leave armed states."""
        for joint_name, route in self._routes.items():
            if route.adapter == "hightorque":
                self.hightorque.disable(route.node_id)
            elif route.adapter == "yiyou":
                self.yiyou.disable()
        self.state = DriverState.READY_UNARMED

    def hold_current(self) -> None:
        """Use current positions as holding targets after explicit enable.

        TODO: implement smooth hold transition based on synchronized states.
        """

    def move_single_joint(self, joint_name: str, target: float) -> None:
        """Move one logical joint in prototype mode.

        Reject motion unless robot is armed.
        """
        if self.state not in {DriverState.ARMED_HOLDING_CURRENT, DriverState.EXECUTING}:
            raise RuntimeError("Motion rejected: robot is not enabled (READY_UNARMED or earlier).")

        route = self._routes[joint_name]
        self.state = DriverState.EXECUTING
        if route.adapter == "hightorque":
            self.hightorque.send_position_velocity_acceleration(route.node_id, target, velocity=0.2, acceleration=0.2)
        else:
            self.yiyou.write_target_position(target)

    def stop_robot(self) -> None:
        """Stop all joints immediately and transition to safe state."""
        for joint_name, route in self._routes.items():
            if route.adapter == "hightorque":
                self.hightorque.stop(route.node_id)
            elif route.adapter == "yiyou":
                self.yiyou.disable()
        self.state = DriverState.FAULT
