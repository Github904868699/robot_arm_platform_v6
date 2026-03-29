#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Yiyou CAN 2.0A low-level transport for current serial-ASCII bridge.

Design goals
------------
- Keep this file ROS-independent.
- Keep only transport/protocol details here.
- Prefer board-proven behavior over speculative register assumptions.

Read command format (board-observed)
------------------------------------
Current board behavior is consistent with short read command payload:
    data = [0x03, reg_addr]
which maps to ASCII frame:
    t + 3-hex CAN ID + 1-hex DLC + data_hex + \r
example for node 0x002 reading position(0x07):
    t00220307\r

Read reply format (board-observed)
----------------------------------
Typical read reply payload starts with 0x04:
    [0x04, reg_addr, value_be32]
example:
    t00260407FFFFE61C\r

Write command path
------------------
Write path remains unchanged and board-proven for enable/disable:
    [0x01, reg_addr, value_be32]
example:
    t0026011000000001\r
"""

from __future__ import annotations

import os
import struct
import time
from dataclasses import dataclass
from typing import Any, Optional


@dataclass
class SerialReply:
    raw: bytes
    text: str


@dataclass
class YiyouReply:
    can_id: int
    dlc: int
    data: bytes
    kind: str
    addr: Optional[int]
    value: Optional[int]


def _import_pyserial():
    try:
        import serial  # type: ignore
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(f"pyserial_not_installed:{exc}") from exc
    return serial


class _SerialAsciiBridge:
    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[Any] = None

    def open(self) -> None:
        serial_module = _import_pyserial()
        self.ser = serial_module.Serial(
            self.port,
            self.baudrate,
            timeout=0,
            write_timeout=1,
            bytesize=serial_module.EIGHTBITS,
            parity=serial_module.PARITY_NONE,
            stopbits=serial_module.STOPBITS_ONE,
        )

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _require(self) -> Any:
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("bridge not opened")
        return self.ser

    def reset_input(self) -> None:
        self._require().reset_input_buffer()

    def send_ascii(self, text: str) -> None:
        self._require().write(text.encode("ascii"))
        self._require().flush()

    def read_idle(self, total_timeout: float = 0.8, idle_gap: float = 0.03) -> SerialReply:
        ser = self._require()
        start = time.monotonic()
        last_rx = start
        buf = bytearray()

        while time.monotonic() - start < total_timeout:
            n = ser.in_waiting
            if n > 0:
                chunk = ser.read(n)
                if chunk:
                    buf.extend(chunk)
                    last_rx = time.monotonic()
            else:
                if buf and (time.monotonic() - last_rx) >= idle_gap:
                    break
                time.sleep(0.005)

        raw = bytes(buf)
        return SerialReply(raw=raw, text=raw.decode("ascii", errors="replace"))


class YiyouLowLevel:
    INIT_CMDS = ["C\r", "S8\r", "M0\r", "A1\r", "O\r"]

    OP_READ = 0x03
    OP_WRITE = 0x01
    OP_WRITE_ACK = 0x02
    OP_READ_REPLY = 0x04

    ADDR_VELOCITY = 0x06
    ADDR_POSITION = 0x07
    ADDR_MODE = 0x0F
    ADDR_ENABLE = 0x10
    ADDR_STOP = 0x11
    ADDR_ALARM = 0x15
    ADDR_TARGET_SPEED = 0x09
    ADDR_TARGET_POSITION = 0x0A

    def __init__(self, port: str = "/dev/ttyACM1", baudrate: int = 115200) -> None:
        self.bridge = _SerialAsciiBridge(port, baudrate)
        self._debug_io = os.getenv("ROBOT_ARM_YIYOU_DEBUG_IO", "0") == "1"

    def open(self) -> None:
        self.bridge.open()

    def close(self) -> None:
        self.bridge.close()

    def initialize_bridge(self) -> None:
        self.bridge.reset_input()
        for cmd in self.INIT_CMDS:
            self.bridge.send_ascii(cmd)
            self._debug_log_tx(cmd.encode("ascii"), cmd)
            time.sleep(0.03)
        time.sleep(0.10)
        self.bridge.reset_input()

    @staticmethod
    def _build_standard_frame(can_id: int, data: bytes) -> str:
        if not (0 <= can_id <= 0x7FF):
            raise ValueError("standard CAN ID out of range")
        if len(data) > 8:
            raise ValueError("CAN 2.0A data too long")
        return f"t{can_id:03X}{len(data):X}{data.hex().upper()}\r"

    def _transact(self, can_id: int, data: bytes, timeout: float = 0.8) -> SerialReply:
        frame = self._build_standard_frame(can_id, data)
        frame_bytes = frame.encode("ascii")
        self.bridge.send_ascii(frame)
        self._debug_log_tx(frame_bytes, frame)
        reply = self.bridge.read_idle(total_timeout=timeout)
        self._debug_log_rx(reply)
        return reply

    @staticmethod
    def _be32(value: int) -> bytes:
        return struct.pack(">i", value)

    @staticmethod
    def _hex(data: bytes) -> str:
        return data.hex().upper()

    def _debug_log_tx(self, payload: bytes, ascii_text: str) -> None:
        if not self._debug_io:
            return
        print(f"[YiyouLowLevel][TX] ascii={ascii_text!r} hex={self._hex(payload)}")

    def _debug_log_rx(self, reply: SerialReply) -> None:
        if not self._debug_io:
            return
        print(f"[YiyouLowLevel][RX] text={reply.text!r} hex={self._hex(reply.raw)}")

    def read_register(self, node_id: int, addr: int, timeout: float = 0.8) -> SerialReply:
        # Board-observed read command family: [0x03, reg_addr]
        payload = bytes([self.OP_READ, addr])
        return self._transact(node_id, payload, timeout)

    def write_register(self, node_id: int, addr: int, value: int, timeout: float = 0.8) -> SerialReply:
        payload = bytes([self.OP_WRITE, addr]) + self._be32(value)
        return self._transact(node_id, payload, timeout)

    def read_position(self, node_id: int, timeout: float = 0.8) -> SerialReply:
        return self.read_register(node_id, self.ADDR_POSITION, timeout)

    def read_velocity(self, node_id: int, timeout: float = 0.8) -> SerialReply:
        return self.read_register(node_id, self.ADDR_VELOCITY, timeout)

    def read_mode(self, node_id: int, timeout: float = 0.8) -> SerialReply:
        return self.read_register(node_id, self.ADDR_MODE, timeout)

    def read_enable_state(self, node_id: int, timeout: float = 0.8) -> SerialReply:
        return self.read_register(node_id, self.ADDR_ENABLE, timeout)

    def enable(self, node_id: int, timeout: float = 0.8) -> SerialReply:
        return self.write_register(node_id, self.ADDR_ENABLE, 1, timeout)

    def disable(self, node_id: int, timeout: float = 0.8) -> SerialReply:
        return self.write_register(node_id, self.ADDR_ENABLE, 0, timeout)

    def stop(self, node_id: int, timeout: float = 0.8) -> SerialReply:
        return self.write_register(node_id, self.ADDR_STOP, 1, timeout)

    def write_target_speed(self, node_id: int, value: int, timeout: float = 0.8) -> SerialReply:
        return self.write_register(node_id, self.ADDR_TARGET_SPEED, value, timeout)

    def write_target_position(self, node_id: int, value: int, timeout: float = 0.8) -> SerialReply:
        return self.write_register(node_id, self.ADDR_TARGET_POSITION, value, timeout)

    @staticmethod
    def _extract_last_frame(text: str) -> str:
        cleaned = text.strip()
        if not cleaned:
            return ""
        lines = [line.strip() for line in cleaned.splitlines() if line.strip()]
        for line in reversed(lines):
            idx_t = max(line.rfind("t"), line.rfind("T"))
            if idx_t >= 0:
                return line[idx_t:]
        idx_t = max(cleaned.rfind("t"), cleaned.rfind("T"))
        return cleaned[idx_t:] if idx_t >= 0 else ""

    @staticmethod
    def parse_reply(reply: SerialReply) -> Optional[YiyouReply]:
        frame_text = YiyouLowLevel._extract_last_frame(reply.text).upper()
        if not frame_text.startswith("T"):
            return None
        if len(frame_text) < 5:
            return None
        can_id = int(frame_text[1:4], 16)
        dlc = int(frame_text[4], 16)
        data_hex = "".join(ch for ch in frame_text[5:] if ch in "0123456789ABCDEF")
        if len(data_hex) < dlc * 2:
            return None
        data = bytes.fromhex(data_hex[: dlc * 2])
        if not data:
            return YiyouReply(can_id, dlc, data, "empty", None, None)

        cmd = data[0]
        if cmd == YiyouLowLevel.OP_READ_REPLY and len(data) >= 6:
            addr = data[1]
            value = int.from_bytes(data[2:6], byteorder="big", signed=True)
            return YiyouReply(can_id, dlc, data, "read_reply", addr, value)

        if cmd == YiyouLowLevel.OP_WRITE_ACK:
            addr = data[1] if len(data) >= 2 else None
            value = data[2] if len(data) >= 3 else None
            return YiyouReply(can_id, dlc, data, "write_ack", addr, value)

        return YiyouReply(can_id, dlc, data, "unknown", None, None)

    @staticmethod
    def position_turns_from_reply(parsed: YiyouReply) -> Optional[float]:
        if parsed.kind != "read_reply" or parsed.addr != YiyouLowLevel.ADDR_POSITION or parsed.value is None:
            return None
        return parsed.value / 65536.0

    @staticmethod
    def velocity_rpm_from_reply(parsed: YiyouReply) -> Optional[float]:
        if parsed.kind != "read_reply" or parsed.addr != YiyouLowLevel.ADDR_VELOCITY or parsed.value is None:
            return None
        return parsed.value * 60.0 / 65536.0


if __name__ == "__main__":
    drv = YiyouLowLevel()
    drv.open()
    try:
        drv.initialize_bridge()
        rep = drv.read_position(2)
        print(rep.text.strip())
        parsed = drv.parse_reply(rep)
        print(parsed)
        if parsed is not None:
            print("turns =", drv.position_turns_from_reply(parsed))
    finally:
        drv.close()
