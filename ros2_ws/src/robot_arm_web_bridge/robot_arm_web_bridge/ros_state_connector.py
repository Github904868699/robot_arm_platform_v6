"""ROS2 state + control connector for web bridge.

This module keeps web API stable while sourcing state from ROS2 when available.
"""

from __future__ import annotations

import json
import importlib.util
import threading
from copy import deepcopy
from dataclasses import asdict
from typing import Any

from .contract import RobotControlContract

if (
    importlib.util.find_spec("rclpy") is not None
    and importlib.util.find_spec("std_srvs") is not None
    and importlib.util.find_spec("std_msgs") is not None
):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    ROS2_AVAILABLE = True
else:  # pragma: no cover - optional dependency
    ROS2_AVAILABLE = False
    rclpy = None
    Node = object  # type: ignore[assignment]
    SingleThreadedExecutor = object  # type: ignore[assignment]
    String = object  # type: ignore[assignment]
    Trigger = object  # type: ignore[assignment]


CONTROL_SERVICE_NAMES = {
    "enable": "/robot_arm/hardware/enable",
    "disable": "/robot_arm/hardware/disable",
    "recover": "/robot_arm/hardware/recover",
    "stop": "/robot_arm/hardware/stop",
    "clear_fault": "/robot_arm/hardware/clear_fault",
}


class _BridgeRosNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_arm_web_bridge_state_client")
        self.state_lock = threading.Lock()
        self.latest_state = RobotControlContract()
        self.latest_state_source = "fallback"

        self.state_sub = self.create_subscription(
            String, "/robot_arm/control/state_json", self._on_state_json, 10
        )

        self.service_clients = {
            name: self.create_client(Trigger, srv_name) for name, srv_name in CONTROL_SERVICE_NAMES.items()
        }

    def _on_state_json(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        contract = RobotControlContract(
            robot_state=data.get("robot_state", "UNAVAILABLE"),
            connection_state=data.get("connection_state", "UNAVAILABLE"),
            armed=bool(data.get("armed", False)),
            mode=str(data.get("mode", "Control")),
            cycle_ms=float(data.get("cycle_ms", 0.0)),
            loss_percent=float(data.get("loss_percent", 0.0)),
            error_message=data.get("error_message"),
            can_retry=bool(data.get("can_retry", False)),
            joints=dict(data.get("joints", {})),
        )
        req = data.get("request", {}) if isinstance(data.get("request"), dict) else {}
        contract.request.action = str(req.get("action", "none"))
        contract.request.status = str(req.get("status", "idle"))
        contract.request.message = req.get("message")
        contract.normalize()

        with self.state_lock:
            self.latest_state = contract
            self.latest_state_source = "ros2"

    def snapshot(self) -> tuple[dict[str, Any], str]:
        with self.state_lock:
            return asdict(deepcopy(self.latest_state)), self.latest_state_source


class RosStateConnector:
    def __init__(self) -> None:
        self.enabled = ROS2_AVAILABLE
        self._thread: threading.Thread | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._node: _BridgeRosNode | None = None
        self._fallback = RobotControlContract()
        if self.enabled:
            self._start_ros()

    def _start_ros(self) -> None:
        assert rclpy is not None
        rclpy.init(args=None)
        self._node = _BridgeRosNode()
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        def spin() -> None:
            assert self._executor is not None
            self._executor.spin()

        self._thread = threading.Thread(target=spin, daemon=True)
        self._thread.start()

    def get_state(self) -> tuple[dict[str, Any], str]:
        if not self.enabled or self._node is None:
            self._fallback.normalize()
            return asdict(self._fallback), "fallback"
        state, source = self._node.snapshot()
        return state, source

    def get_joints(self) -> tuple[dict[str, float], str]:
        state, source = self.get_state()
        joints = state.get("joints", {})
        if not isinstance(joints, dict):
            joints = {}
        return joints, source

    def call_control(self, action: str) -> tuple[bool, str]:
        if not self.enabled or self._node is None:
            return False, "ros2_unavailable"
        client = self._node.service_clients.get(action)
        if client is None:
            return False, "unsupported_action"
        if not client.wait_for_service(timeout_sec=0.4):
            return False, "service_unavailable"
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=1.0)
        if not future.done():
            return False, "service_timeout"
        response = future.result()
        if response is None:
            return False, "service_error"
        return bool(response.success), str(response.message)
