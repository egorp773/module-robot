"""Pure, monotonic-time safety policy primitives.

This module intentionally has no ROS imports.  The node uses these primitives for
the command arbitration and the unit tests exercise the same code.  Freshness is
always based on local receipt time; ROS header stamps are informational only.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
from typing import Optional


class Mode(IntEnum):
    DISARMED = 0
    MANUAL = 1
    AUTO = 2


class State(IntEnum):
    DISCONNECTED = 0
    DISARMED = 1
    MANUAL = 2
    AUTO = 3
    FAULT = 4
    ESTOP = 5


class Source(IntEnum):
    """Ordered only for serialization; priority is explicit in ``selected_source``."""

    ZERO = 0
    AUTO = 1
    MANUAL = 2
    STOP = 3
    ESTOP = 4


@dataclass(frozen=True)
class Conditions:
    connected: bool = False
    esp32_armed: bool = False
    esp32_fault_free: bool = True
    motor_feedback_fresh: bool = False
    motor_fault_free: bool = False
    estop_clear: bool = True
    operator_armed: bool = False
    explicit_auto_arm: bool = False
    rtk_fresh: bool = False
    rtk_fixed: bool = False
    rtk_accuracy_ok: bool = False
    imu_fresh: bool = False
    gnss_fresh: bool = False
    heading_initialized: bool = False
    localization_valid: bool = False
    route_valid: bool = False
    nav2_active: bool = False


@dataclass(frozen=True)
class GateRequirements:
    manual_connected: bool = True
    manual_motor_feedback_fresh: bool = True
    manual_no_estop: bool = True
    manual_no_motor_fault: bool = True
    manual_operator_arm: bool = True
    auto_rtk_fixed: bool = True
    auto_rtk_accuracy: bool = True
    auto_imu_fresh: bool = True
    auto_gnss_fresh: bool = True
    auto_heading_initialized: bool = True
    auto_localization_valid: bool = True
    auto_route_valid: bool = True
    auto_nav2_active: bool = True
    auto_explicit_arm: bool = True


def manual_rejection(
    conditions: Conditions,
    requirements: GateRequirements = GateRequirements(),
) -> Optional[str]:
    """Return the first unmet manual gate, in safety-priority order."""

    checks = (
        (not requirements.manual_no_estop or conditions.estop_clear, "ESTOP_ACTIVE"),
        (conditions.esp32_fault_free, "ESP32_FAULT"),
        (not requirements.manual_connected or conditions.connected, "ESP32_DISCONNECTED"),
        (
            not requirements.manual_motor_feedback_fresh
            or conditions.motor_feedback_fresh,
            "MOTOR_FEEDBACK_STALE",
        ),
        (
            not requirements.manual_no_motor_fault or conditions.motor_fault_free,
            "MOTOR_FAULT",
        ),
        (
            not requirements.manual_operator_arm or conditions.operator_armed,
            "OPERATOR_NOT_ARMED",
        ),
        (conditions.esp32_armed, "ESP32_NOT_ARMED"),
    )
    return next((reason for passed, reason in checks if not passed), None)


def auto_rejection(
    conditions: Conditions,
    requirements: GateRequirements = GateRequirements(),
) -> Optional[str]:
    """Return the first unmet AUTO gate; all manual gates apply to AUTO."""

    manual = manual_rejection(conditions, requirements)
    if manual:
        return manual
    checks = (
        (
            not requirements.auto_explicit_arm or conditions.explicit_auto_arm,
            "AUTO_NOT_EXPLICITLY_ARMED",
        ),
        (conditions.rtk_fresh, "RTK_STATUS_STALE"),
        (not requirements.auto_rtk_fixed or conditions.rtk_fixed, "RTK_NOT_FIXED"),
        (
            not requirements.auto_rtk_accuracy or conditions.rtk_accuracy_ok,
            "RTK_ACCURACY_BAD",
        ),
        (not requirements.auto_imu_fresh or conditions.imu_fresh, "IMU_STALE"),
        (not requirements.auto_gnss_fresh or conditions.gnss_fresh, "GNSS_STALE"),
        (
            not requirements.auto_heading_initialized
            or conditions.heading_initialized,
            "HEADING_NOT_INITIALIZED",
        ),
        (
            not requirements.auto_localization_valid
            or conditions.localization_valid,
            "LOCALIZATION_INVALID",
        ),
        (not requirements.auto_route_valid or conditions.route_valid, "ROUTE_INVALID"),
        (not requirements.auto_nav2_active or conditions.nav2_active, "NAV2_INACTIVE"),
    )
    return next((reason for passed, reason in checks if not passed), None)


def selected_source(
    state: State,
    stop_requested: bool,
    manual_fresh: bool,
    auto_fresh: bool,
) -> str:
    """Select exactly one source: ESTOP > STOP > MANUAL > AUTO > ZERO.

    A fresh manual command may override AUTO.  AUTO is never eligible while the
    requested state is MANUAL.  Callers are responsible for passing freshness
    only after the relevant safety gates have passed.
    """

    if state == State.ESTOP:
        return "ESTOP"
    if stop_requested or state in (State.DISCONNECTED, State.DISARMED, State.FAULT):
        return "STOP"
    if state in (State.MANUAL, State.AUTO) and manual_fresh:
        return "MANUAL"
    if state == State.AUTO and auto_fresh:
        return "AUTO"
    return "ZERO"


def is_fresh(received_monotonic_s: float, now_monotonic_s: float, timeout_s: float) -> bool:
    """Check a local receipt timestamp, rejecting future and non-finite values."""

    if timeout_s <= 0.0:
        return False
    if not math.isfinite(received_monotonic_s) or not math.isfinite(now_monotonic_s):
        return False
    age = now_monotonic_s - received_monotonic_s
    return 0.0 <= age <= timeout_s


@dataclass
class CommandSlot:
    """Latest validated planar command from one upstream source."""

    linear_m_s: float = 0.0
    angular_rad_s: float = 0.0
    received_monotonic_s: float = float("-inf")
    generation: int = 0
    valid: bool = False

    def update(
        self,
        linear_m_s: float,
        angular_rad_s: float,
        received_monotonic_s: float,
        generation: int,
    ) -> None:
        self.linear_m_s = linear_m_s
        self.angular_rad_s = angular_rad_s
        self.received_monotonic_s = received_monotonic_s
        self.generation = generation
        self.valid = True

    def invalidate(self) -> None:
        self.linear_m_s = 0.0
        self.angular_rad_s = 0.0
        self.received_monotonic_s = float("-inf")
        self.valid = False

    def fresh(self, now_monotonic_s: float, timeout_s: float, minimum_generation: int = 0) -> bool:
        return (
            self.valid
            and self.generation >= minimum_generation
            and is_fresh(self.received_monotonic_s, now_monotonic_s, timeout_s)
        )


@dataclass
class FaultLatch:
    """Explicitly resettable fault storage, separate from live candidate status."""

    active: bool = False
    code: int = 0
    reason: str = ""
    occurrence_count: int = 0

    def latch(self, code: int, reason: str, enabled: bool = True) -> bool:
        """Latch once and count repeated observations; return True on transition."""

        self.occurrence_count += 1
        if not enabled or self.active:
            return False
        self.active = True
        self.code = int(code) & 0xFFFF
        self.reason = str(reason)
        return True

    def reset(self) -> None:
        self.active = False
        self.code = 0
        self.reason = ""


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)
