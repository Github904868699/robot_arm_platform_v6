"""Minimal ROS2 control-entry server for enable/disable/recover/stop/clear_fault.

This node publishes high-level unified state for web bridge consumption.
It intentionally keeps protocol/bus/vendor details outside web-facing APIs.
"""

from __future__ import annotations

import json
import threading
from dataclasses import asdict
from typing import Callable

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .contract import RobotControlContract


class ControlEntryServer(Node):
    def __init__(self) -> None:
        super().__init__("robot_arm_control_entry_server")
        self._service_group = ReentrantCallbackGroup()
        self._client_group = ReentrantCallbackGroup()
        self.contract = RobotControlContract()
        self.contract.robot_state = "READY_UNARMED"
        self.contract.connection_state = "CONNECTED"
        self.contract.mode = "Control"

        self.state_pub = self.create_publisher(String, "/robot_arm/control/state_json", 10)
        self.joint_sub = self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

        self.hardware_clients = {
            "enable": self.create_client(Trigger, "/robot_arm/hardware/enable", callback_group=self._client_group),
            "disable": self.create_client(Trigger, "/robot_arm/hardware/disable", callback_group=self._client_group),
            "recover": self.create_client(Trigger, "/robot_arm/hardware/recover", callback_group=self._client_group),
            "stop": self.create_client(Trigger, "/robot_arm/hardware/stop", callback_group=self._client_group),
            "clear_fault": self.create_client(Trigger, "/robot_arm/hardware/clear_fault", callback_group=self._client_group),
        }

        self.srv_enable = self.create_service(
            Trigger, "/robot_arm/control/enable", self._wrap(self.handle_enable), callback_group=self._service_group
        )
        self.srv_disable = self.create_service(
            Trigger, "/robot_arm/control/disable", self._wrap(self.handle_disable), callback_group=self._service_group
        )
        self.srv_recover = self.create_service(
            Trigger, "/robot_arm/control/recover", self._wrap(self.handle_recover), callback_group=self._service_group
        )
        self.srv_stop = self.create_service(
            Trigger, "/robot_arm/control/stop", self._wrap(self.handle_stop), callback_group=self._service_group
        )
        self.srv_clear_fault = self.create_service(
            Trigger, "/robot_arm/control/clear_fault", self._wrap(self.handle_clear_fault), callback_group=self._service_group
        )

        self.state_timer = self.create_timer(0.1, self.publish_state)

    def _on_joint_states(self, msg: JointState) -> None:
        for idx, joint_name in enumerate(msg.name):
            if joint_name in self.contract.joints and idx < len(msg.position):
                self.contract.joints[joint_name] = float(msg.position[idx])

    def publish_state(self) -> None:
        payload = String()
        self.contract.normalize()
        payload.data = json.dumps(asdict(self.contract))
        self.state_pub.publish(payload)

    def _wrap(
        self, handler: Callable[[Trigger.Request, Trigger.Response], Trigger.Response]
    ) -> Callable[[Trigger.Request, Trigger.Response], Trigger.Response]:
        def wrapped(request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
            result = handler(request, response)
            self.publish_state()
            return result

        return wrapped

    def _set_request(self, action: str, status: str, message: str | None = None) -> None:
        self.contract.request.action = action
        self.contract.request.status = status
        self.contract.request.message = message
        self.contract.normalize()

    def _forward_hardware(self, action: str) -> tuple[bool, str]:
        client = self.hardware_clients[action]
        if not client.wait_for_service(timeout_sec=0.2):
            return False, f"hardware_service_unavailable:{action}"
        future = client.call_async(Trigger.Request())
        done_event = threading.Event()
        future.add_done_callback(lambda _: done_event.set())
        if not done_event.wait(timeout=0.6):
            return False, f"hardware_service_timeout:{action}"
        resp = future.result()
        if resp is None:
            return False, f"hardware_service_error:{action}"
        return bool(resp.success), str(resp.message)

    def _reply(self, response: Trigger.Response, success: bool, msg: str) -> Trigger.Response:
        response.success = success
        response.message = msg
        return response

    def handle_enable(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._set_request("enable", "in_progress")
        if self.contract.robot_state != "READY_UNARMED":
            self.contract.error_message = "cannot_enable_from_current_state"
            self.contract.can_retry = True
            self._set_request("enable", "failed", self.contract.error_message)
            return self._reply(response, False, self.contract.error_message)

        ok, message = self._forward_hardware("enable")
        if not ok:
            self.contract.error_message = message
            self.contract.can_retry = True
            self._set_request("enable", "failed", message)
            return self._reply(response, False, message)
        self.contract.robot_state = "ARMING"
        self.contract.armed = True
        self.contract.robot_state = "ARMED_HOLDING_CURRENT"
        self.contract.error_message = None
        self.contract.can_retry = False
        self._set_request("enable", "success")
        return self._reply(response, True, "enabled")

    def handle_disable(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._set_request("disable", "in_progress")
        if self.contract.robot_state == "FAULT":
            self.contract.error_message = "cannot_disable_in_fault"
            self.contract.can_retry = True
            self._set_request("disable", "failed", self.contract.error_message)
            return self._reply(response, False, self.contract.error_message)

        ok, message = self._forward_hardware("disable")
        if not ok:
            self.contract.error_message = message
            self.contract.can_retry = True
            self._set_request("disable", "failed", message)
            return self._reply(response, False, message)
        self.contract.robot_state = "READY_UNARMED"
        self.contract.armed = False
        self.contract.error_message = None
        self.contract.can_retry = False
        self._set_request("disable", "success")
        return self._reply(response, True, "disabled")

    def handle_stop(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._set_request("stop", "in_progress")
        ok, message = self._forward_hardware("stop")
        if not ok:
            self.contract.error_message = message
            self.contract.can_retry = True
            self._set_request("stop", "failed", message)
            return self._reply(response, False, message)
        self.contract.robot_state = "FAULT"
        self.contract.armed = False
        self.contract.error_message = "stopped_to_fault"
        self.contract.can_retry = True
        self._set_request("stop", "success", "stopped_to_fault")
        return self._reply(response, True, "stopped_to_fault")

    def handle_clear_fault(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._set_request("clear_fault", "in_progress")
        ok, message = self._forward_hardware("clear_fault")
        if not ok:
            self.contract.error_message = message
            self.contract.can_retry = True
            self._set_request("clear_fault", "failed", message)
            return self._reply(response, False, message)
        self.contract.error_message = None
        self.contract.can_retry = False
        if self.contract.robot_state == "FAULT":
            self.contract.robot_state = "READY_UNARMED"
        self._set_request("clear_fault", "success")
        return self._reply(response, True, "fault_cleared")

    def handle_recover(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._set_request("recover", "in_progress")
        if self.contract.robot_state != "FAULT":
            self.contract.error_message = "recover_requires_fault"
            self.contract.can_retry = True
            self._set_request("recover", "failed", self.contract.error_message)
            return self._reply(response, False, self.contract.error_message)

        ok, message = self._forward_hardware("recover")
        if not ok:
            self.contract.error_message = message
            self.contract.can_retry = True
            self._set_request("recover", "failed", message)
            return self._reply(response, False, message)
        self.contract.robot_state = "READY_UNARMED"
        self.contract.armed = False
        self.contract.error_message = None
        self.contract.can_retry = False
        self._set_request("recover", "success")
        return self._reply(response, True, "recovered")


def main() -> None:
    rclpy.init(args=None)
    node = ControlEntryServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
