"""Small deterministic helpers kept separate from ROS and serial I/O."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class CommandGate:
    stale_after_s: float
    armed_confirmed: bool = False
    last_command_s: Optional[float] = None
    linear_mps: float = 0.0
    angular_rad_s: float = 0.0

    def update(self, now_s: float, linear_mps: float, angular_rad_s: float) -> None:
        self.last_command_s = now_s
        self.linear_mps = linear_mps
        self.angular_rad_s = angular_rad_s

    def invalidate(self) -> None:
        self.last_command_s = None
        self.linear_mps = 0.0
        self.angular_rad_s = 0.0

    def is_fresh(self, now_s: float) -> bool:
        return self.last_command_s is not None and 0.0 <= now_s - self.last_command_s <= self.stale_after_s

    def output(self, now_s: float) -> tuple[float, float]:
        if not self.armed_confirmed or not self.is_fresh(now_s):
            return 0.0, 0.0
        return self.linear_mps, self.angular_rad_s


@dataclass
class ReconnectBackoff:
    delay_s: float
    next_attempt_s: float = 0.0

    def failed(self, now_s: float) -> None:
        self.next_attempt_s = now_s + self.delay_s

    def ready(self, now_s: float) -> bool:
        return now_s >= self.next_attempt_s
