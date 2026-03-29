"""HighTorque CAN-FD adapter.

Thin adapter layer over :mod:`hightorque_low_level`.
- Adapter owns route/node validation and error normalization.
- Low-level owns serial bridge I/O and protocol frames.

Compatibility note
------------------
This adapter preserves the historical public method names used by upper layers.
Some methods are intentionally compatibility shims and are documented explicitly.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable


class HightorqueAdapterError(RuntimeError):
    """Adapter-level error with protocol details hidden from upper layers."""


@dataclass(frozen=True)
class HightorqueJointRoute:
    """Route metadata for one logical joint.

    Attributes:
        node_id: CAN logical node id in range [0, 0x7F].
    """

    node_id: int


class HightorqueFdcanAdapter:
    """Thin high-level actions for HighTorque joints on CAN-FD."""

    def __init__(
        self,
        serial_device: str = "/dev/ttyACM0",
        baudrate: int = 115200,
        known_node_ids: Iterable[int] | None = None,
    ) -> None:
        """Create adapter.

        Args:
            serial_device: Serial bridge device path.
            baudrate: Serial bridge baudrate.
            known_node_ids: Controlled/known node list for `scan()`.
                This is *not* generic active bus discovery.
        """
        self.serial_device = serial_device
        self.baudrate = baudrate
        self._known_node_ids = sorted(set(known_node_ids or []))
        self._ll = self._build_low_level(serial_device, baudrate)
        self._connected = False

    def open(self) -> None:
        """Open transport.

        Raises:
            HightorqueAdapterError: if transport open fails.
        """
        try:
            self._ll.open()
            self._connected = True
        except Exception as exc:  # noqa: BLE001
            raise HightorqueAdapterError(f"failed_to_open_hightorque_bridge:{exc}") from exc

    def close(self) -> None:
        """Close transport."""
        self._ll.close()
        self._connected = False

    def initialize_bridge(self) -> None:
        """Initialize serial/CAN bridge with verified init sequence."""
        self._require_open()
        try:
            self._ll.initialize_bridge()
        except Exception as exc:  # noqa: BLE001
            raise HightorqueAdapterError(f"failed_to_initialize_hightorque_bridge:{exc}") from exc

    def scan(self) -> list[int]:
        """Return online node IDs from the controlled known-node list.

        Behavior:
        - Only probes `known_node_ids` passed at construction.
        - Does *not* claim generic bus-wide auto-discovery.

        Returns:
            Sorted list of known node ids that replied to state query.
        """
        self._require_open()
        if not self._known_node_ids:
            return []
        online: list[int] = []
        for node_id in self._known_node_ids:
            if self._query_state(node_id) is not None:
                online.append(node_id)
        return online

    def read_joint_state(self, node_id: int) -> dict:
        """Read one joint state snapshot.

        Args:
            node_id: HighTorque node id [0, 0x7F].

        Returns:
            Dict with stable keys:
            - node_id: int
            - position: float | None (unit: turns)
            - velocity: float | None (unit: rps)
            - online: bool
            - fault: int (only when online=True)
            - mode: int (only when online=True)
            - raw_frame: str (only when online=True)
        """
        self._validate_node_id(node_id)
        self._require_open()
        parsed = self._query_state(node_id)
        if parsed is None:
            return {"node_id": node_id, "position": None, "velocity": None, "online": False}
        return {
            "node_id": node_id,
            "position": parsed.position_turns,
            "velocity": parsed.velocity_rps,
            "online": True,
            "fault": parsed.fault,
            "mode": parsed.mode,
            "raw_frame": parsed.raw_payload_hex,
        }

    def enable(self, node_id: int) -> None:
        """Compatibility-preserved method.

        Current status:
        - Kept for upper-layer API stability.
        - No verified dedicated HighTorque "enable" frame in current low-level set.
        - Therefore currently acts as a validated no-op placeholder.
        """
        self._validate_node_id(node_id)
        self._require_open()

    def disable(self, node_id: int) -> None:
        """Disable via brake command (current verified behavior)."""
        self.brake(node_id)

    def stop(self, node_id: int) -> None:
        """Send verified quick-stop frame for one node."""
        self._validate_node_id(node_id)
        self._require_open()
        try:
            self._ll.stop_tint16(node_id)
        except Exception as exc:  # noqa: BLE001
            raise HightorqueAdapterError(f"failed_to_stop_node:{node_id}:{exc}") from exc

    def brake(self, node_id: int) -> None:
        """Send verified brake frame for one node."""
        self._validate_node_id(node_id)
        self._require_open()
        try:
            self._ll.brake_tint16(node_id)
        except Exception as exc:  # noqa: BLE001
            raise HightorqueAdapterError(f"failed_to_brake_node:{node_id}:{exc}") from exc

    def query(self, node_id: int) -> dict:
        """Compatibility alias to `read_joint_state(node_id)`."""
        return self.read_joint_state(node_id)

    def send_position_velocity_acceleration(
        self,
        node_id: int,
        position: float,
        velocity: float,
        acceleration: float,
    ) -> None:
        """Compatibility-preserved legacy signature.

        Units:
            position: turns (currently not applied in low-level command)
            velocity: rps (currently the only motion parameter that is effective)
            acceleration: turns/s^2 (currently not applied)

        Current real behavior:
        - Calls low-level `move_mode2_position_unlimited(node_id, rps=velocity)`.
        - `position` and `acceleration` are retained only to keep call compatibility,
          and are intentionally ignored at this stage.
        """
        del position
        del acceleration
        self._validate_node_id(node_id)
        self._require_open()
        try:
            self._ll.move_mode2_position_unlimited(node_id, rps=float(velocity))
        except Exception as exc:  # noqa: BLE001
            raise HightorqueAdapterError(f"failed_to_move_node:{node_id}:{exc}") from exc

    def _query_state(self, node_id: int):
        try:
            reply = self._ll.query_tint16_state(node_id)
            return self._ll.parse_tint16_state(reply)
        except Exception as exc:  # noqa: BLE001
            raise HightorqueAdapterError(f"failed_to_query_node:{node_id}:{exc}") from exc

    def _require_open(self) -> None:
        if not self._connected:
            raise HightorqueAdapterError("hightorque_bridge_not_open")

    @staticmethod
    def _validate_node_id(node_id: int) -> None:
        if not isinstance(node_id, int) or not (0 <= node_id <= 0x7F):
            raise HightorqueAdapterError(f"invalid_node_id:{node_id}")

    @staticmethod
    def _build_low_level(serial_device: str, baudrate: int) -> Any:
        try:
            from .hightorque_low_level import HighTorqueLowLevel
        except Exception as exc:  # noqa: BLE001
            raise HightorqueAdapterError(f"failed_to_import_hightorque_low_level:{exc}") from exc
        return HighTorqueLowLevel(port=serial_device, baudrate=baudrate)


def smoke_check_hightorque_adapter_interface() -> dict:
    """Import-level smoke check without hardware access.

    This function intentionally avoids opening serial transport and only validates
    basic constructor/import/public-method presence.
    """
    return {
        "class": HightorqueFdcanAdapter.__name__,
        "public_methods": [
            name
            for name in (
                "open",
                "close",
                "initialize_bridge",
                "scan",
                "read_joint_state",
                "enable",
                "disable",
                "stop",
                "brake",
                "query",
                "send_position_velocity_acceleration",
            )
            if callable(getattr(HightorqueFdcanAdapter, name, None))
        ],
    }
