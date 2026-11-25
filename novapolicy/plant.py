"""A plant model between a setpoint policy and the waypoint interface.

Some policies are trained as closed loops around a compliant joint servo: every
step they emit one absolute joint setpoint and the servo pulls toward it, never
arriving before the next one lands. NOVA's action-chunk streaming consumes
waypoints with deadlines instead: a lone target is a terminal point the server
brakes to a standstill on, so the lead time caps the speed, and padding it with
a hold makes the arm stop and go.

A :class:`VirtualPlant` closes that gap. It carries the servo the policy was
trained against and steps it with each setpoint. The policy observes the plant,
exactly as in training, and the robot follows the plant's measured path through
the executor's ring buffer (``buffer_window_ms``), which replays it as a smooth
horizon one window late.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol, runtime_checkable

if TYPE_CHECKING:
    from nova.types import RobotState
    from novapolicy.types import ActionChunk


@runtime_checkable
class VirtualPlant(Protocol):
    """What :class:`novapolicy.executor.PolicyExecutor` needs from a plant."""

    def reset(self, states: dict[str, RobotState]) -> None:
        """Place the plant on the robot's measured state before the first step."""

    def observe(self, states: dict[str, RobotState]) -> dict[str, RobotState]:
        """The states the policy should see; joints replaced by the plant's."""

    def shape(self, chunk: ActionChunk) -> ActionChunk:
        """Advance the plant one step on the chunk's first targets.

        Returns a one-step chunk holding the plant's joint positions after this
        step: the point the robot is asked to pass through next.
        """
