"""Absolute-deadline schedulers for production and deterministic tests."""

from __future__ import annotations

import time
from abc import ABC, abstractmethod
from typing import Callable


class Scheduler(ABC):
    """Clock and interruptible absolute-wait boundary."""

    @abstractmethod
    def now(self) -> float:
        """Return scheduler monotonic time."""

    @abstractmethod
    def wait_until(
        self,
        deadline_monotonic_s: float,
        cancellation_requested: Callable[[], bool],
    ) -> float:
        """Wait until an absolute deadline and return signed scheduler jitter."""


class DeadlineScheduler(Scheduler):
    """Production monotonic scheduler with bounded cancel polling."""

    def __init__(
        self,
        monotonic_clock: Callable[[], float] = time.monotonic,
        sleeper: Callable[[float], None] = time.sleep,
        cancellation_poll_s: float = 0.01,
    ) -> None:
        if cancellation_poll_s <= 0.0:
            raise ValueError("cancellation_poll_s must be positive.")
        self._clock = monotonic_clock
        self._sleeper = sleeper
        self._cancellation_poll_s = float(cancellation_poll_s)

    def now(self) -> float:
        """Return process monotonic time."""
        return self._clock()

    def wait_until(
        self,
        deadline_monotonic_s: float,
        cancellation_requested: Callable[[], bool],
    ) -> float:
        """Sleep in short bounded intervals until deadline or cancellation."""
        while not cancellation_requested():
            remaining_s = deadline_monotonic_s - self._clock()
            if remaining_s <= 0.0:
                break
            self._sleeper(min(remaining_s, self._cancellation_poll_s))
        return self._clock() - deadline_monotonic_s


class VirtualScheduler(Scheduler):
    """Deterministic no-sleep scheduler intended only for tests."""

    def __init__(self, initial_monotonic_s: float = 0.0) -> None:
        self._now_s = float(initial_monotonic_s)

    def now(self) -> float:
        """Return virtual monotonic time."""
        return self._now_s

    def wait_until(
        self,
        deadline_monotonic_s: float,
        cancellation_requested: Callable[[], bool],
    ) -> float:
        """Advance virtual time immediately unless canceled."""
        if not cancellation_requested():
            self._now_s = max(self._now_s, float(deadline_monotonic_s))
        return self._now_s - deadline_monotonic_s
