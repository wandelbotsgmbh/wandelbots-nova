"""A Zimmer gripper on its own bus, driven by a policy through ``Observation.computed`` and
``Action.computed``.

The gripper is a Zimmer GEH6060IL on a Turck IO-Link master, driven by the optional
``zimmer-gripper-controller`` package over Modbus TCP. :class:`ZimmerGripper` maps the jaw
gap onto the state values the policy was trained with and executes its binary gripper
action; :class:`HoldWhileGripping` keeps the arm still while the jaws move.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import importlib
import logging
from typing import TYPE_CHECKING, Any

import numpy as np

from novapolicy import ActionChunk, PolicyClient

if TYPE_CHECKING:
    from novapolicy import Gr00tPolicyClient, PolicySchema

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class GripperStates:
    """The policy's gripper state values: fully open, fully closed, holding the object."""

    open: float
    closed: float
    holding: float


def open_gripper_session(host: str, *, force_percent: int, velocity_percent: int) -> Any:
    """Connect to the gripper, home it and open it (moves the jaws). Blocking.

    In force mode the grip force also sets the closing speed; ``velocity_percent`` applies
    to positioning moves (``open_and_wait``) only.
    """
    # Imported here so dry runs work without the (optional) gripper package installed.
    zimmer = importlib.import_module("zimmer_gripper_controller")
    session = zimmer.GripperSession(zimmer.SessionConfig(modbus=zimmer.ModbusConfig(host=host)))
    session.connect()
    session.wait_until_ready()
    session.set_force_percent(force_percent)
    session.set_velocity_percent(velocity_percent)
    return session


class ZimmerGripper:
    """Feeds the jaw gap to the policy and executes its ``gripper`` action.

    ``GripperSession`` is blocking and thread based, so commands run in a worker thread.
    A command is only sent when the open/closed decision changes, at the step of the
    chunk that asks for it (a negative command closes). While a command runs,
    :class:`HoldWhileGripping` keeps the arm still and the policy is not asked, so its
    next observation is taken after the jaws have settled.

    Args:
        client: The GR00T client; its ``last_raw_action["gripper"]`` is the command.
        host: The IO-Link master's address.
        states: The policy's gripper state values.
        open_gap_mm: The jaw gap after homing; the gap maps linearly onto
            ``states.closed`` .. ``states.open``.
        lookahead_steps: Steps of each answer a switch may come from.
        dt_ms: Step spacing, to delay a switch to its step.
    """

    def __init__(
        self,
        client: Gr00tPolicyClient,
        host: str,
        *,
        states: GripperStates,
        open_gap_mm: float,
        lookahead_steps: int,
        dt_ms: float,
        force_percent: int = 50,
        velocity_percent: int = 100,
    ) -> None:
        self._client = client
        self._host = host
        self._states = states
        self._open_gap_mm = open_gap_mm
        self._lookahead_steps = lookahead_steps
        self._dt_ms = dt_ms
        self._force_percent = force_percent
        self._velocity_percent = velocity_percent
        self._session: Any = None  # created in connect(); dry runs never connect
        self._task: asyncio.Task[None] | None = None
        self._task_started = False  # the scheduled grip()/open() has left its delay
        self._closed: bool | None = None
        self._holding = False

    async def connect(self) -> None:
        """Start the controller, home, and open (moves the jaws)."""
        self._session = await asyncio.to_thread(
            open_gripper_session,
            self._host,
            force_percent=self._force_percent,
            velocity_percent=self._velocity_percent,
        )
        self._closed = False

    async def close(self) -> None:
        """Let a running grip()/open() finish; drop one still waiting for its step."""
        if self._task is not None and not self._task.done():
            if not self._task_started:
                self._task.cancel()  # the run ended before the command's step came
            await asyncio.gather(self._task, return_exceptions=True)
        if self._session is not None:
            self._session.close()

    async def observe(self, _obs: dict[str, Any]) -> dict[str, float]:
        """``Observation.computed``: the jaw gap mapped onto the policy's state values."""
        s = self._states
        if self._session is None:  # dry run: jaws assumed open
            return {"gripper": s.open}
        state = self._session.state()
        if state.has_error:
            msg = f"Gripper error: diag=0x{state.diagnosis:04X} {state.last_error_text}"
            raise RuntimeError(msg)
        if self._closed and self._holding:
            return {"gripper": s.holding}
        opening = min(max(state.jaw_gap_m * 1000.0 / self._open_gap_mm, 0.0), 1.0)
        return {"gripper": s.closed + opening * (s.open - s.closed)}

    @property
    def busy(self) -> bool:
        """A grip()/open_and_wait() is scheduled or running."""
        return self._task is not None and not self._task.done()

    def next_switch(self, *, closed: bool | None) -> tuple[int, bool] | None:
        """First step of the last answer whose decision differs from ``closed``."""
        command = self._client.last_raw_action.get("gripper")
        values = [] if command is None else np.asarray(command).reshape(-1).tolist()
        for index, value in enumerate(values[: self._lookahead_steps]):
            if (value < 0.0) != closed:
                return index, value < 0.0
        return None

    async def act(self, _chunk: ActionChunk) -> None:
        """``Action.computed``, fired as the arm reaches the chunk's first step."""
        if self._session is None or self.busy:
            return
        switch = self.next_switch(closed=self._closed)
        if switch is None:
            return
        index, close = switch
        self._closed = close
        self._task_started = False
        delay_s = index * self._dt_ms / 1000.0
        self._task = asyncio.create_task(self._command(close=close, delay_s=delay_s))

    async def _command(self, *, close: bool, delay_s: float) -> None:
        await asyncio.sleep(delay_s)
        self._task_started = True
        try:  # ruff: ignore[too-many-statements-in-try-clause]
            if close:
                result = await asyncio.to_thread(self._session.grip)
                self._holding = result.holding
                print(f"Gripper grip: holding={result.holding} gap={result.jaw_gap_m:.4f} m")
            else:
                await asyncio.to_thread(self._session.open_and_wait)
                self._holding = False
        except Exception:
            self._closed = None  # unknown; the next chunk re-sends its decision
            logger.exception("Gripper command failed")


class HoldWhileGripping(PolicyClient):
    """Stand still instead of asking the policy while the gripper moves.

    A zero relative TCP delta holds the pose the arm is at (for ``Action.tcp(mode=
    "relative")``). The policy's next observation is then taken after the jaws have
    settled, with images and joints that show it.
    """

    def __init__(
        self,
        policy: Gr00tPolicyClient,
        gripper: ZimmerGripper,
        mg_id: str,
        *,
        dt_ms: float,
        hold_steps: int = 5,
    ) -> None:
        self._policy = policy
        self._gripper = gripper
        self._mg_id = mg_id
        self._dt_ms = dt_ms
        self._hold_steps = hold_steps

    async def connect(self, motion_group_ids: list[str]) -> None:
        await self._policy.connect(motion_group_ids)

    async def validate_schema(self, schema: PolicySchema) -> None:
        await self._policy.validate_schema(schema)

    async def close(self) -> None:
        await self._policy.close()

    async def get_actions(
        self,
        states: dict[str, Any],
        schema: PolicySchema,
        images: dict[str, Any] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        if not self._gripper.busy:
            return await self._policy.get_actions(states, schema, images, io_values)
        await asyncio.sleep(self._dt_ms / 1000.0)  # no inference; don't spin the executor loop
        self._policy.last_raw_action = {}  # nothing new for the gripper hook to act on
        hold = [[0.0] * 6] * self._hold_steps
        return ActionChunk(tcp={self._mg_id: hold}, dt_ms=self._dt_ms)
