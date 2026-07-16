"""Shared deterministic test clocks for novapolicy unit tests."""

from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace

import pytest

import novapolicy.jogging.clock as clock_module


@dataclass
class ManualTime:
    """Controllable replacement for the monotonic clock used by jogging."""

    now: float = 10.0

    def monotonic(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


@pytest.fixture
def manual_time(monkeypatch: pytest.MonkeyPatch) -> ManualTime:
    clock = ManualTime()
    monkeypatch.setattr(clock_module, "time", SimpleNamespace(monotonic=clock.monotonic))
    return clock
