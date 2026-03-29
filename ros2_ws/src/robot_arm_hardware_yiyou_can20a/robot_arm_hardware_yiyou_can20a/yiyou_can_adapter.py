"""Yiyou CAN 2.0A adapter.

Thin adapter layer over :mod:`yiyou_low_level`.
- Adapter owns route/node validation and error normalization.
- Low-level owns transport and protocol frame details.
"""

from __future__ import annotations

from typing import Any


class YiyouAdapterError(RuntimeError):
    """Adapter-level error with protocol details hidden from upper layers."""


class YiyouCanAdapter:
    """Thin high-level actions for one Yiyou joint on CAN 2.0A."""

    def __init__(self, serial_device: str = "/dev/ttyACM1", bitrate: int = 115200, node_id: int = 2) -> None:
        self.serial_device = serial_device
        self.bitrate = bitrate
        self.node_id = node_id
        self._ll = self._build_low_level(serial_device, bitrate)
        self._connected = False

    def open(self) -> None:
        self._validate_node_id(self.node_id)
        try:
            self._ll.open()
            self._connected = True
        except Exception as exc:  # noqa: BLE001
            raise YiyouAdapterError(f"failed_to_open_yiyou_bridge:{exc}") from exc

    def close(self) -> None:
        self._ll.close()
        self._connected = False

    def initialize_bridge(self) -> None:
        self._require_open()
        try:
            self._ll.initialize_bridge()
        except Exception as exc:  # noqa: BLE001
            raise YiyouAdapterError(f"failed_to_initialize_yiyou_bridge:{exc}") from exc

    def read_mode(self) -> dict:
        parsed = self._parsed_reply(self._ll.read_mode(self.node_id), op="read_mode")
        return {"mode": parsed.value}

    def read_position(self) -> dict:
        parsed = self._parsed_reply(self._ll.read_position(self.node_id), op="read_position")
        turns = self._ll.position_turns_from_reply(parsed)
        return {"position": turns, "raw_value": parsed.value}

    def read_velocity(self) -> dict:
        parsed = self._parsed_reply(self._ll.read_velocity(self.node_id), op="read_velocity")
        rpm = self._ll.velocity_rpm_from_reply(parsed)
        return {"velocity": rpm, "raw_value": parsed.value}

    def read_enable_state(self) -> dict:
        parsed = self._parsed_reply(self._ll.read_enable_state(self.node_id), op="read_enable_state")
        return {"enabled": bool(parsed.value) if parsed.value is not None else None, "raw_value": parsed.value}

    def enable(self) -> None:
        self._parsed_reply(self._ll.enable(self.node_id), op="enable")

    def disable(self) -> None:
        self._parsed_reply(self._ll.disable(self.node_id), op="disable")

    def stop(self) -> None:
        self._parsed_reply(self._ll.stop(self.node_id), op="stop")

    def write_target_position(self, position: float) -> None:
        raw = int(round(float(position) * 65536.0))
        self._parsed_reply(self._ll.write_target_position(self.node_id, raw), op="write_target_position")

    def write_target_speed(self, speed: float) -> None:
        raw = int(round(float(speed) * 65536.0 / 60.0))
        self._parsed_reply(self._ll.write_target_speed(self.node_id, raw), op="write_target_speed")

    def _parsed_reply(self, reply, op: str):
        self._require_open()
        if len(reply.raw) == 0 and reply.text.strip() == "":
            raise YiyouAdapterError(f"empty_reply:{op}:tx_sent_but_no_rx")
        try:
            parsed = self._ll.parse_reply(reply)
        except Exception as exc:  # noqa: BLE001
            raise YiyouAdapterError(f"failed_to_parse_reply:{op}:{exc}") from exc
        if parsed is None:
            rx_hex = reply.raw.hex().upper()
            raise YiyouAdapterError(f"parse_failed:{op}:unrecognized_frame:rx_text={reply.text!r}:rx_hex={rx_hex}")
        return parsed

    def _require_open(self) -> None:
        if not self._connected:
            raise YiyouAdapterError("yiyou_bridge_not_open")

    @staticmethod
    def _validate_node_id(node_id: int) -> None:
        if not isinstance(node_id, int) or not (0 <= node_id <= 0x7FF):
            raise YiyouAdapterError(f"invalid_node_id:{node_id}")

    @staticmethod
    def _build_low_level(serial_device: str, bitrate: int) -> Any:
        try:
            from .yiyou_low_level import YiyouLowLevel
        except Exception as exc:  # noqa: BLE001
            raise YiyouAdapterError(f"failed_to_import_yiyou_low_level:{exc}") from exc
        return YiyouLowLevel(port=serial_device, baudrate=bitrate)


def smoke_check_yiyou_adapter_interface() -> dict:
    """Import-level smoke check without hardware access."""
    return {
        "class": YiyouCanAdapter.__name__,
        "public_methods": [
            name
            for name in (
                "open",
                "close",
                "initialize_bridge",
                "read_mode",
                "read_position",
                "read_velocity",
                "read_enable_state",
                "enable",
                "disable",
                "stop",
                "write_target_position",
                "write_target_speed",
            )
            if callable(getattr(YiyouCanAdapter, name, None))
        ],
    }
