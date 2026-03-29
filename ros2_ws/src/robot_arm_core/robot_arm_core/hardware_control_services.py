"""ROS2 hardware control services proxy for robot arm platform."""

from __future__ import annotations

import json
import threading
from dataclasses import asdict

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robot_arm_web_bridge.contract import RobotControlContract


class HardwareControlServices(Node):
    """Proxy/client layer: does not directly own serial/CAN hardware."""

    def __init__(self) -> None:
        super().__init__("robot_arm_hardware_control_services")
        self._service_group = ReentrantCallbackGroup()
        self._client_group = ReentrantCallbackGroup()
        self.contract = RobotControlContract()
        self.contract.robot_state = "READY_UNARMED"
        self.contract.connection_state = "CONNECTED"
        self.contract.can_retry = False

        self._backend_prefix = "/robot_arm/hardware_backend"
        self._backend_clients: dict[str, rclpy.client.Client] = {
            "enable": self.create_client(Trigger, f"{self._backend_prefix}/enable", callback_group=self._client_group),
            "disable": self.create_client(Trigger, f"{self._backend_prefix}/disable", callback_group=self._client_group),
            "stop": self.create_client(Trigger, f"{self._backend_prefix}/stop", callback_group=self._client_group),
            "clear_fault": self.create_client(Trigger, f"{self._backend_prefix}/clear_fault", callback_group=self._client_group),
            "recover": self.create_client(Trigger, f"{self._backend_prefix}/recover", callback_group=self._client_group),
            "write_single_joint_step": self.create_client(
                Trigger, f"{self._backend_prefix}/write_single_joint_step", callback_group=self._client_group
            ),
        }

        self.state_pub = self.create_publisher(String, "/robot_arm/control/state_json", 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_timer(0.1, self.publish_state)

        self.create_service(
            Trigger, "/robot_arm/hardware/enable", self.handle_enable, callback_group=self._service_group
        )
        self.create_service(
            Trigger, "/robot_arm/hardware/disable", self.handle_disable, callback_group=self._service_group
        )
        self.create_service(
            Trigger, "/robot_arm/hardware/stop", self.handle_stop, callback_group=self._service_group
        )
        self.create_service(
            Trigger, "/robot_arm/hardware/clear_fault", self.handle_clear_fault, callback_group=self._service_group
        )
        self.create_service(
            Trigger, "/robot_arm/hardware/recover", self.handle_recover, callback_group=self._service_group
        )
        self.create_service(
            Trigger,
            "/robot_arm/hardware/write_single_joint_step",
            self.handle_write_single_step,
            callback_group=self._service_group,
        )

    def _on_joint_states(self, msg: JointState) -> None:
        for idx, name in enumerate(msg.name):
            if name not in self.contract.joints:
                continue
            if idx < len(msg.position):
                self.contract.joints[name] = float(msg.position[idx])

    def _respond(self, response: Trigger.Response, success: bool, message: str) -> Trigger.Response:
        response.success = success
        response.message = message
        return response

    def _forward_trigger(self, op: str) -> tuple[bool, str]:
        client = self._backend_clients[op]
        if not client.wait_for_service(timeout_sec=0.05):
            return False, f"backend_service_unavailable:{op}"
        request = Trigger.Request()
        future = client.call_async(request)
        done_event = threading.Event()
        future.add_done_callback(lambda _: done_event.set())
        if not done_event.wait(timeout=0.2):
            return False, f"backend_service_timeout:{op}"
        result = future.result()
        if result is None:
            return False, f"backend_service_error:{op}"
        return bool(result.success), str(result.message)

    def publish_state(self) -> None:
        msg = String()
        self.contract.normalize()
        msg.data = json.dumps(asdict(self.contract))
        self.state_pub.publish(msg)

    def handle_enable(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        ok, message = self._forward_trigger("enable")
        if ok:
            self.contract.armed = True
            self.contract.robot_state = "ARMED_HOLDING_CURRENT"
        return self._respond(response, ok, message)

    def handle_disable(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        ok, message = self._forward_trigger("disable")
        if ok:
            self.contract.armed = False
            self.contract.robot_state = "READY_UNARMED"
        return self._respond(response, ok, message)

    def handle_stop(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        ok, message = self._forward_trigger("stop")
        if ok:
            self.contract.armed = False
            self.contract.robot_state = "FAULT"
            self.contract.can_retry = True
        return self._respond(response, ok, message)

    def handle_clear_fault(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        ok, message = self._forward_trigger("clear_fault")
        if ok:
            self.contract.error_message = None
            self.contract.can_retry = False
            if self.contract.robot_state == "FAULT":
                self.contract.robot_state = "READY_UNARMED"
        return self._respond(response, ok, message)

    def handle_recover(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        ok, message = self._forward_trigger("recover")
        if ok:
            self.contract.robot_state = "READY_UNARMED"
            self.contract.armed = False
            self.contract.error_message = None
            self.contract.can_retry = False
        return self._respond(response, ok, message)

    def handle_write_single_step(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        ok, message = self._forward_trigger("write_single_joint_step")
        return self._respond(response, ok, message)


def main() -> None:
    rclpy.init(args=None)
    node = HardwareControlServices()
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
