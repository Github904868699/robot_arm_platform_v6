#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HighTorque CAN-FD low-level transport for the current serial-ASCII bridge.

Design goals
------------
- Keep this file ROS-independent.
- Keep only protocol/transport details here.
- Use only the command family already verified on the real robot.
- Be conservative: do not invent unverified registers or frame layouts.

Current verified path
---------------------
Bridge init:
    C\r S8\r Y5\r M0\r A1\r O\r

Board frame family:
    D + 8-hex CAN-FD ID + 1-char DLC + payload_hex + \r

Verified operations:
- query_tint16_state(node_id)
- stop_tint16(node_id)
- brake_tint16(node_id)
- move_mode2_position_unlimited(node_id, rps)

Protocol notes
--------------
- HighTorque CAN-FD protocol uses 16-bit logical ID, little-endian payload fields,
  and 0x50 as NOP padding byte.
- The verified motion family here is the mode2 / position unlimited / speed family.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Optional

import serial


@dataclass
class SerialReply:
    raw: bytes
    text: str


@dataclass
class HighTorqueTint16State:
    mode: int
    position_turns: float
    position_deg: float
    velocity_rps: float
    torque_raw: float
    fault: int
    raw_payload_hex: str


class _SerialAsciiBridge:
    def __init__(self, port: str, baudrate: int = 115200) -> None:
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None

    def open(self) -> None:
        self.ser = serial.Serial(
            self.port,
            self.baudrate,
            timeout=0,
            write_timeout=1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _require(self) -> serial.Serial:
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


class HighTorqueLowLevel:
    INIT_CMDS = ["C\r", "S8\r", "Y5\r", "M0\r", "A1\r", "O\r"]

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 115200) -> None:
        self.bridge = _SerialAsciiBridge(port, baudrate)

    def open(self) -> None:
        self.bridge.open()

    def close(self) -> None:
        self.bridge.close()

    def initialize_bridge(self) -> None:
        self.bridge.reset_input()
        for cmd in self.INIT_CMDS:
            self.bridge.send_ascii(cmd)
            time.sleep(0.03)
        time.sleep(0.10)
        self.bridge.reset_input()

    @staticmethod
    def _dlc_char_for_size(size: int) -> str:
        if size == 0:
            return "0"
        if size <= 8:
            return f"{size:X}"
        if size <= 12:
            return "9"
        if size <= 16:
            return "A"
        if size <= 20:
            return "B"
        if size <= 24:
            return "C"
        if size <= 32:
            return "D"
        if size <= 48:
            return "E"
        if size <= 64:
            return "F"
        raise ValueError("payload too long")

    @classmethod
    def _build_board_frame(cls, can_id: int, payload: bytes) -> str:
        return f"D{can_id:08X}{cls._dlc_char_for_size(len(payload))}{payload.hex().upper()}\r"

    @staticmethod
    def _reply_enabled_can_id(node_id: int, source_id: int = 0) -> int:
        if not (0 <= node_id <= 0x7F):
            raise ValueError("node_id out of range")
        if not (0 <= source_id <= 0x7F):
            raise ValueError("source_id out of range")
        return (0x80 << 8) | (source_id << 8) | node_id

    def _transact(self, node_id: int, payload: bytes, timeout: float = 0.8) -> SerialReply:
        frame = self._build_board_frame(self._reply_enabled_can_id(node_id), payload)
        self.bridge.send_ascii(frame)
        return self.bridge.read_idle(total_timeout=timeout)

    @staticmethod
    def _payload_query_tint16_state() -> bytes:
        return bytes.fromhex("140400110F")

    @staticmethod
    def _payload_stop_tint16() -> bytes:
        return bytes.fromhex("010000140400110F")

    @staticmethod
    def _payload_brake_tint16() -> bytes:
        return bytes.fromhex("01000F140400110F")

    @staticmethod
    def _payload_move_mode2_position_unlimited(rps: float) -> bytes:
        vel_i32 = int(round(rps * 100000.0))
        if vel_i32 < -2147483648 or vel_i32 > 2147483647:
            raise ValueError("rps out of int32 range")
        payload = bytearray.fromhex("01000A08022000000080102700005050")
        payload[10:14] = struct.pack("<i", vel_i32)
        return bytes(payload)

    def query_tint16_state(self, node_id: int, timeout: float = 0.8) -> SerialReply:
        return self._transact(node_id, self._payload_query_tint16_state(), timeout)

    def stop_tint16(self, node_id: int, timeout: float = 0.8) -> SerialReply:
        return self._transact(node_id, self._payload_stop_tint16(), timeout)

    def brake_tint16(self, node_id: int, timeout: float = 0.8) -> SerialReply:
        return self._transact(node_id, self._payload_brake_tint16(), timeout)

    def move_mode2_position_unlimited(self, node_id: int, rps: float, timeout: float = 0.8) -> SerialReply:
        return self._transact(node_id, self._payload_move_mode2_position_unlimited(rps), timeout)

    @staticmethod
    def parse_tint16_state(reply: SerialReply) -> Optional[HighTorqueTint16State]:
        text = reply.text.strip().upper()
        idx = text.find("240400")
        if idx < 0:
            return None
        payload_hex = "".join(ch for ch in text[idx:] if ch in "0123456789ABCDEF")
        if len(payload_hex) < 28:
            return None
        payload = bytes.fromhex(payload_hex)
        if len(payload) < 14:
            return None
        if not (payload[0] == 0x24 and payload[1] == 0x04 and payload[2] == 0x00):
            return None
        if not (payload[11] == 0x21 and payload[12] == 0x0F):
            return None

        mode = payload[3]
        pos = struct.unpack("<h", payload[5:7])[0]
        vel = struct.unpack("<h", payload[7:9])[0]
        tqe = struct.unpack("<h", payload[9:11])[0]
        fault = payload[13]

        position_turns = pos / 10000.0
        velocity_rps = vel / 4000.0
        torque_raw = tqe / 100.0

        return HighTorqueTint16State(
            mode=mode,
            position_turns=position_turns,
            position_deg=position_turns * 360.0,
            velocity_rps=velocity_rps,
            torque_raw=torque_raw,
            fault=fault,
            raw_payload_hex=payload[:14].hex().upper(),
        )


if __name__ == "__main__":
    drv = HighTorqueLowLevel()
    drv.open()
    try:
        drv.initialize_bridge()
        rep = drv.query_tint16_state(6)
        print(rep.text.strip())
        print(drv.parse_tint16_state(rep))
    finally:
        drv.close()
