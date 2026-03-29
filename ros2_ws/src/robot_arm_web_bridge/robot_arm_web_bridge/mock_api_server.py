"""Minimal mock API server for robot_arm_web_bridge.

This server exposes only high-level robot semantics for frontend consumption.
No bus/node/protocol/vendor fields are exposed.
"""

from __future__ import annotations

import json
import os
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any

if __package__:
    from .contract import RobotControlContract
    from .ros_state_connector import RosStateConnector
else:  # direct script mode
    from contract import RobotControlContract
    from ros_state_connector import RosStateConnector


class BridgeState:
    def __init__(self) -> None:
        self.contract = RobotControlContract()
        self.state_connector = RosStateConnector()
        self.state_source = "fallback"
        self.allow_local_fallback = os.getenv("ROBOT_ARM_WEB_ALLOW_LOCAL_FALLBACK", "false").lower() == "true"
        self.teach_sequence: list[dict[str, Any]] = [
            {"id": "step_1", "type": "MoveJ", "name": "Go Home"},
            {"id": "step_2", "type": "Wait", "name": "Stabilize"},
            {"id": "step_3", "type": "MoveL", "name": "Approach"},
            {"id": "step_4", "type": "MoveC", "name": "Arc Align"},
            {"id": "step_5", "type": "SetIO", "name": "Clamp ON"},
        ]

    def mark_request(self, action: str, status: str, message: str | None = None) -> None:
        self.contract.request.action = action
        self.contract.request.status = status
        self.contract.request.message = message
        self.contract.normalize()

    def refresh_state_from_ros(self) -> None:
        state_dict, source = self.state_connector.get_state()
        self.state_source = source
        self.contract = RobotControlContract(
            robot_state=str(state_dict.get("robot_state", "UNAVAILABLE")),
            connection_state=str(state_dict.get("connection_state", "UNAVAILABLE")),
            armed=bool(state_dict.get("armed", False)),
            mode=str(state_dict.get("mode", "Control")),
            cycle_ms=float(state_dict.get("cycle_ms", 0.0)),
            loss_percent=float(state_dict.get("loss_percent", 0.0)),
            error_message=state_dict.get("error_message"),
            can_retry=bool(state_dict.get("can_retry", False)),
            joints=dict(state_dict.get("joints", {})),
        )
        req = state_dict.get("request", {}) if isinstance(state_dict.get("request"), dict) else {}
        self.contract.request.action = str(req.get("action", "none"))
        self.contract.request.status = str(req.get("status", "idle"))
        self.contract.request.message = req.get("message")
        self.contract.normalize()

    def finalize_success(self, action: str) -> None:
        self.contract.error_message = None
        self.contract.can_retry = False
        self.mark_request(action, "success")

    def reject(self, action: str, message: str) -> tuple[int, dict[str, Any]]:
        self.mark_request(action, "failed", message)
        self.contract.error_message = message
        self.contract.can_retry = True
        return 409, {"request": self.contract.request.__dict__, "state": self.contract.to_dict()}

    def apply_local_fallback_action(self, action: str) -> tuple[bool, str]:
        self.state_source = "fallback"
        self.mark_request(action, "in_progress")
        if action == "enable":
            if self.contract.connection_state != "CONNECTED" or self.contract.robot_state != "READY_UNARMED":
                return False, "fallback_enable_rejected"
            self.contract.robot_state = "ARMED_HOLDING_CURRENT"
            self.contract.armed = True
            self.finalize_success(action)
            return True, "ok"
        if action == "disable":
            if self.contract.robot_state == "FAULT":
                return False, "fallback_disable_rejected"
            self.contract.robot_state = "READY_UNARMED"
            self.contract.armed = False
            self.finalize_success(action)
            return True, "ok"
        if action == "recover":
            if self.contract.robot_state != "FAULT":
                return False, "fallback_recover_rejected"
            self.contract.robot_state = "READY_UNARMED"
            self.contract.armed = False
            self.finalize_success(action)
            return True, "ok"
        if action == "stop":
            self.contract.robot_state = "FAULT"
            self.contract.armed = False
            self.mark_request(action, "success", "fallback_stopped_to_fault")
            self.contract.can_retry = True
            return True, "ok"
        if action == "clear_fault":
            if self.contract.robot_state == "FAULT":
                self.contract.robot_state = "READY_UNARMED"
            self.contract.armed = False
            self.finalize_success(action)
            return True, "ok"
        return False, "fallback_unsupported_action"


STATE = BridgeState()


class Handler(BaseHTTPRequestHandler):
    def _write_json(self, code: int, payload: dict[str, Any], extra_headers: dict[str, str] | None = None) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        if extra_headers:
            for key, value in extra_headers.items():
                self.send_header(key, value)
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self) -> None:  # noqa: N802
        self._write_json(200, {"ok": True})

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/state":
            STATE.refresh_state_from_ros()
            self._write_json(200, STATE.contract.to_dict(), {"X-Bridge-State-Source": STATE.state_source})
            return
        if self.path == "/joints":
            joints, source = STATE.state_connector.get_joints()
            self._write_json(200, {"joints": joints}, {"X-Bridge-State-Source": source})
            return
        if self.path == "/mode":
            self._write_json(200, {"mode": STATE.contract.mode})
            return
        if self.path == "/teach/sequence":
            self._write_json(200, {"steps": STATE.teach_sequence})
            return
        self._write_json(404, {"error": "not_found"})

    def do_POST(self) -> None:  # noqa: N802
        if self.path == "/enable":
            ok, message = STATE.state_connector.call_control("enable")
            if STATE.allow_local_fallback and not ok and message in {"ros2_unavailable", "service_unavailable", "service_timeout"}:
                ok, message = STATE.apply_local_fallback_action("enable")
            else:
                STATE.refresh_state_from_ros()
            if not ok:
                code, payload = STATE.reject("enable", message)
                self._write_json(code, payload, {"X-Bridge-State-Source": STATE.state_source})
                return
            STATE.finalize_success("enable")
            self._write_json(
                200,
                {"request": STATE.contract.request.__dict__, "state": STATE.contract.to_dict()},
                {"X-Bridge-State-Source": STATE.state_source},
            )
            return

        if self.path == "/disable":
            ok, message = STATE.state_connector.call_control("disable")
            if STATE.allow_local_fallback and not ok and message in {"ros2_unavailable", "service_unavailable", "service_timeout"}:
                ok, message = STATE.apply_local_fallback_action("disable")
            else:
                STATE.refresh_state_from_ros()
            if not ok:
                code, payload = STATE.reject("disable", message)
                self._write_json(code, payload, {"X-Bridge-State-Source": STATE.state_source})
                return
            STATE.finalize_success("disable")
            self._write_json(
                200,
                {"request": STATE.contract.request.__dict__, "state": STATE.contract.to_dict()},
                {"X-Bridge-State-Source": STATE.state_source},
            )
            return

        if self.path == "/fault":
            STATE.mark_request("fault", "in_progress")
            STATE.contract.robot_state = "FAULT"
            STATE.contract.armed = False
            STATE.contract.error_message = "mock_fault_injected"
            STATE.contract.can_retry = True
            STATE.mark_request("fault", "success", "mock_fault_injected")
            self._write_json(200, {"request": STATE.contract.request.__dict__, "state": STATE.contract.to_dict()})
            return

        if self.path == "/recover":
            ok, message = STATE.state_connector.call_control("recover")
            if STATE.allow_local_fallback and not ok and message in {"ros2_unavailable", "service_unavailable", "service_timeout"}:
                ok, message = STATE.apply_local_fallback_action("recover")
            else:
                STATE.refresh_state_from_ros()
            if not ok:
                code, payload = STATE.reject("recover", message)
                self._write_json(code, payload, {"X-Bridge-State-Source": STATE.state_source})
                return
            STATE.finalize_success("recover")
            self._write_json(
                200,
                {"request": STATE.contract.request.__dict__, "state": STATE.contract.to_dict()},
                {"X-Bridge-State-Source": STATE.state_source},
            )
            return

        if self.path == "/stop":
            ok, message = STATE.state_connector.call_control("stop")
            if STATE.allow_local_fallback and not ok and message in {"ros2_unavailable", "service_unavailable", "service_timeout"}:
                ok, message = STATE.apply_local_fallback_action("stop")
            else:
                STATE.refresh_state_from_ros()
            if not ok:
                code, payload = STATE.reject("stop", message)
                self._write_json(code, payload, {"X-Bridge-State-Source": STATE.state_source})
                return
            STATE.mark_request("stop", "success")
            self._write_json(
                200,
                {"request": STATE.contract.request.__dict__, "state": STATE.contract.to_dict()},
                {"X-Bridge-State-Source": STATE.state_source},
            )
            return

        if self.path == "/clear_fault":
            ok, message = STATE.state_connector.call_control("clear_fault")
            if STATE.allow_local_fallback and not ok and message in {"ros2_unavailable", "service_unavailable", "service_timeout"}:
                ok, message = STATE.apply_local_fallback_action("clear_fault")
            else:
                STATE.refresh_state_from_ros()
            if not ok:
                code, payload = STATE.reject("clear_fault", message)
                self._write_json(code, payload, {"X-Bridge-State-Source": STATE.state_source})
                return
            STATE.mark_request("clear_fault", "success")
            self._write_json(
                200,
                {"request": STATE.contract.request.__dict__, "state": STATE.contract.to_dict()},
                {"X-Bridge-State-Source": STATE.state_source},
            )
            return

        if self.path == "/teach/sequence":
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length) if length > 0 else b"{}"
            data = json.loads(raw.decode("utf-8"))
            steps = data.get("steps", [])
            if isinstance(steps, list):
                STATE.teach_sequence = steps
                self._write_json(200, {"steps": STATE.teach_sequence})
                return
            self._write_json(400, {"error": "invalid_steps"})
            return

        self._write_json(404, {"error": "not_found"})


def run_mock_server(host: str = "0.0.0.0", port: int = 8090) -> None:
    server = HTTPServer((host, port), Handler)
    print(f"robot_arm_web_bridge mock api listening on http://{host}:{port}")
    server.serve_forever()


def main() -> None:
    run_mock_server()


if __name__ == "__main__":
    main()
