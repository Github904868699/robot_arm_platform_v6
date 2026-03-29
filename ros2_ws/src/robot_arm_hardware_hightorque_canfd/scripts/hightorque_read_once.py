#!/usr/bin/env python3
"""Minimal standalone Hightorque serial probe for real-hardware debugging."""

from __future__ import annotations

import argparse
import os
import select
import termios
import time
import tty


def hex_digest(payload: bytes) -> str:
    return " ".join(f"{b:02X}" for b in payload[:128])


def ascii_escaped(payload: bytes) -> str:
    return payload.decode("utf-8", errors="ignore").replace("\r", "\\r").replace("\n", "\\n")


def set_raw_115200(fd: int) -> None:
    attrs = termios.tcgetattr(fd)
    if hasattr(termios, "cfmakeraw"):
        termios.cfmakeraw(attrs)
    else:
        # Python compatibility fallback (e.g. envs without termios.cfmakeraw).
        attrs[0] = attrs[0] & ~(termios.BRKINT | termios.ICRNL | termios.INPCK | termios.ISTRIP | termios.IXON)
        attrs[1] = attrs[1] & ~(termios.OPOST)
        attrs[2] = attrs[2] | (termios.CS8)
        attrs[3] = attrs[3] & ~(termios.ECHO | termios.ICANON | termios.IEXTEN | termios.ISIG)
    attrs[2] |= termios.CREAD | termios.CLOCAL
    attrs[2] &= ~termios.CRTSCTS
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    if hasattr(termios, "cfsetispeed") and hasattr(termios, "cfsetospeed"):
        termios.cfsetispeed(attrs, termios.B115200)
        termios.cfsetospeed(attrs, termios.B115200)
    else:
        tty.setraw(fd)
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def read_window(fd: int, total_s: float = 1.0) -> bytes:
    deadline = time.monotonic() + total_s
    last_data_ts: float | None = None
    out = bytearray()
    while time.monotonic() < deadline:
        r, _, _ = select.select([fd], [], [], 0)
        if r:
            chunk = os.read(fd, 512)
            if chunk:
                out.extend(chunk)
                last_data_ts = time.monotonic()
                print(f"chunk_len={len(chunk)} chunk_hex={hex_digest(chunk)} chunk_ascii='{ascii_escaped(chunk)}'")
                continue
        if last_data_ts is not None and (time.monotonic() - last_data_ts) > 0.05:
            break
        time.sleep(0.005)
    return bytes(out)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--node-id", type=int, default=1)
    args = parser.parse_args()

    can_id = 0x8000 | (args.node_id & 0xFF)
    query = f"D{can_id:08X}A010000140400110F0000000000005050\r"

    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        set_raw_115200(fd)
        termios.tcflush(fd, termios.TCIOFLUSH)
        for cmd in ("C\n", "S8\n", "Y5\n", "M0\n", "A1\n", "O\n"):
            os.write(fd, cmd.encode("utf-8"))
            termios.tcdrain(fd)
            print(f"TX init ascii='{cmd.strip()}' hex={hex_digest(cmd.encode())}")
            time.sleep(0.015)
            init_rx = read_window(fd, total_s=0.08)
            if init_rx:
                print(f"RX init raw_hex={hex_digest(init_rx)} raw_ascii='{ascii_escaped(init_rx)}'")
        time.sleep(0.05)
        termios.tcflush(fd, termios.TCIFLUSH)

        os.write(fd, query.encode("utf-8"))
        termios.tcdrain(fd)
        print(f"TX query ascii='{query.replace(chr(13), '\\r')}' hex={hex_digest(query.encode())}")
        raw = read_window(fd, total_s=1.0)
        print(f"raw_len={len(raw)} raw_hex={hex_digest(raw)} raw_ascii='{ascii_escaped(raw)}'")
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
