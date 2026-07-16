"""Strict parser for the small legacy Flutter command surface.

The WebSocket layer bounds the encoded message before this parser is called.
This module remains ROS-independent so protocol behaviour can be unit tested on
development hosts without ROS installed.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Union


class CommandError(ValueError):
    """A client-visible command parsing error."""

    def __init__(self, code: str, command: str = "COMMAND") -> None:
        super().__init__(code)
        self.code = code
        self.command = command

    def response(self) -> str:
        return f"ERR,{self.command},{self.code}"


@dataclass(frozen=True)
class ManualDrive:
    left_percent: float
    right_percent: float


@dataclass(frozen=True)
class Stop:
    pass


@dataclass(frozen=True)
class Ping:
    """Transport liveness probe; it has no motion semantics."""


@dataclass(frozen=True)
class RouteBegin:
    expected_count: int
    origin_latitude: Optional[float] = None
    origin_longitude: Optional[float] = None

    @property
    def legacy_local_mode(self) -> bool:
        return self.origin_latitude is not None


@dataclass(frozen=True)
class RouteWaypoint:
    index: int
    first: float
    second: float


@dataclass(frozen=True)
class RouteEnd:
    pass


@dataclass(frozen=True)
class NavigationCommand:
    action: str


Command = Union[
    ManualDrive,
    Stop,
    Ping,
    RouteBegin,
    RouteWaypoint,
    RouteEnd,
    NavigationCommand,
]


def _finite_float(raw: str, command: str, field: str) -> float:
    try:
        value = float(raw.strip())
    except (TypeError, ValueError) as exc:
        raise CommandError(f"INVALID_{field}", command) from exc
    if not math.isfinite(value):
        raise CommandError(f"INVALID_{field}", command)
    return value


def _integer(raw: str, command: str, field: str) -> int:
    token = raw.strip()
    if not token or (token[0] in "+-" and len(token) == 1):
        raise CommandError(f"INVALID_{field}", command)
    digits = token[1:] if token[0] in "+-" else token
    if not digits or any(character < "0" or character > "9" for character in digits):
        raise CommandError(f"INVALID_{field}", command)
    try:
        return int(token, 10)
    except ValueError as exc:  # Defensive; digit and size checks above are bounded.
        raise CommandError(f"INVALID_{field}", command) from exc


def parse_command(text: str, *, max_message_bytes: int = 4096) -> Command:
    """Parse one complete text command.

    The grammar is deliberately closed. Unknown commands, extra fields, binary
    data (rejected by the server), NaN, and infinities never reach motion code.
    """

    if not isinstance(text, str):
        raise CommandError("TEXT_REQUIRED")
    try:
        encoded_size = len(text.encode("utf-8", errors="strict"))
    except UnicodeError as exc:
        raise CommandError("INVALID_UTF8") from exc
    if encoded_size == 0:
        raise CommandError("EMPTY")
    if encoded_size > max_message_bytes:
        raise CommandError("MESSAGE_TOO_LARGE")
    if "\x00" in text or "\r" in text or "\n" in text:
        raise CommandError("CONTROL_CHARACTER")

    line = text.strip()
    if not line:
        raise CommandError("EMPTY")
    fields = line.split(",")
    verb = fields[0]

    if verb == "M":
        if len(fields) != 3:
            raise CommandError("FORMAT", "M")
        return ManualDrive(
            _finite_float(fields[1], "M", "LEFT"),
            _finite_float(fields[2], "M", "RIGHT"),
        )

    if verb == "STOP":
        if len(fields) != 1:
            raise CommandError("FORMAT", "STOP")
        return Stop()

    if verb == "PING":
        if len(fields) != 1:
            raise CommandError("FORMAT", "PING")
        return Ping()

    if verb == "ROUTE_BEGIN":
        if len(fields) not in (2, 4):
            raise CommandError("FORMAT", "ROUTE_BEGIN")
        count = _integer(fields[1], "ROUTE_BEGIN", "COUNT")
        if len(fields) == 2:
            return RouteBegin(count)
        return RouteBegin(
            count,
            _finite_float(fields[2], "ROUTE_BEGIN", "LATITUDE"),
            _finite_float(fields[3], "ROUTE_BEGIN", "LONGITUDE"),
        )

    if verb == "ROUTE_WP":
        if len(fields) != 4:
            raise CommandError("FORMAT", "ROUTE_WP")
        return RouteWaypoint(
            _integer(fields[1], "ROUTE_WP", "INDEX"),
            _finite_float(fields[2], "ROUTE_WP", "COORDINATE_1"),
            _finite_float(fields[3], "ROUTE_WP", "COORDINATE_2"),
        )

    if verb == "ROUTE_END":
        if len(fields) != 1:
            raise CommandError("FORMAT", "ROUTE_END")
        return RouteEnd()

    if verb in {"NAV_START", "NAV_PAUSE", "NAV_RESUME", "NAV_STOP"}:
        if len(fields) != 1:
            raise CommandError("FORMAT", verb)
        return NavigationCommand(verb)

    raise CommandError("UNKNOWN_COMMAND", verb[:32] or "COMMAND")
