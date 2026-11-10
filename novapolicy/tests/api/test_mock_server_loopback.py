"""``NatsPolicyClient`` against the mock policy service over a real broker.

The unit tests cover each side on its own; this is the one that proves they agree
on the wire, including the msgpack numpy encoding and the block binding the client
derives from the advertised spaces. It needs a broker, so it skips without one:

    docker run -d --rm -p 4222:4222 nats:2-alpine
    uv run --no-sync pytest novapolicy/tests/api/test_mock_server_loopback.py

``NOVAPOLICY_TEST_BROKER`` overrides ``nats://localhost:4222``. Each test serves
on its own subject prefix, so a broker shared with anything else is safe.
"""

from __future__ import annotations

import asyncio
import contextlib
from itertools import pairwise
import math
import os
from typing import TYPE_CHECKING
from unittest.mock import MagicMock

import numpy as np
import pytest

from novapolicy.nats.client import NatsPolicyClient
from novapolicy.nats.mock_server import MockPolicyServer, build_policies

if TYPE_CHECKING:
    from collections.abc import AsyncIterator

BROKER = os.environ.get("NOVAPOLICY_TEST_BROKER", "nats://localhost:4222")
DOF = 7
BLOCKS = ["left", "right"]
LEFT_ID, RIGHT_ID = "1@mock", "2@mock"
HOME = [0.1, 0.2, -0.3, 0.4, 0.5, 0.0, -0.1]
AMPLITUDE_DEG = 10.0
PERIOD_S = 4.0
DT_MS = 20.0
MOVING_JOINT = 1
QUARTER_PERIOD_STEPS = int(PERIOD_S * 1000 / DT_MS / 4)

pytestmark = pytest.mark.asyncio


class _FakeState:
    """Stands in for a Nova ``RobotState``; the client only reads ``joints``."""

    def __init__(self, joints: list[float]) -> None:
        self.joints = tuple(joints)


def _states(joints: list[float] | None = None) -> dict[str, _FakeState]:
    state = _FakeState(joints or HOME)
    return {LEFT_ID: state, RIGHT_ID: state}


async def _require_broker() -> None:
    try:
        import nats

        connection = await nats.connect(servers=[BROKER], connect_timeout=1, allow_reconnect=False)
    except Exception:  # ruff: ignore[blind-except]
        pytest.skip(f"No NATS broker at {BROKER}")
    else:
        await connection.close()


@contextlib.asynccontextmanager
async def _serving(
    pattern: str, *, prefix: str, horizon: int = 1, joints: list[int] | None = None
) -> AsyncIterator[MockPolicyServer]:
    """Serve one pattern on its own subject prefix for the duration of a test."""
    await _require_broker()
    server = MockPolicyServer(
        build_policies(
            blocks=BLOCKS,
            dof=DOF,
            horizon=horizon,
            dt_ms=DT_MS,
            amplitude_deg=AMPLITUDE_DEG,
            period_s=PERIOD_S,
            joints=joints or [1, 3],
            patterns=[pattern],
        ),
        servers=[BROKER],
        subject_prefix=prefix,
    )
    await server.start()
    try:
        yield server
    finally:
        await server.stop()


@contextlib.asynccontextmanager
async def _connected(pattern: str, *, prefix: str, **kwargs) -> AsyncIterator[NatsPolicyClient]:
    """A mock serving ``pattern`` with a client already bound to both blocks."""
    async with _serving(pattern, prefix=prefix, **kwargs):
        client = NatsPolicyClient(
            policy_id=f"mock-{pattern}", servers=BROKER, subject_prefix=prefix
        )
        await client.connect([LEFT_ID, RIGHT_ID])
        try:
            yield client
        finally:
            await client.close()


async def test_client_binds_both_blocks_and_reads_the_declared_rate() -> None:
    async with _connected("circle", prefix="loopback-bind") as client:
        assert [name for name, _ in client.bound_blocks] == [LEFT_ID, RIGHT_ID]
        specs = [spec for _, spec in client.bound_blocks]
        assert {spec.dof for spec in specs} == {DOF}
        assert {spec.dt_ms for spec in specs} == {DT_MS}
        assert {spec.horizon for spec in specs} == {1}


async def test_first_chunk_holds_the_observed_pose() -> None:
    """Step zero is the opening pose, so homing to the policy's start is a no-op
    and no episode begins with a jump."""
    async with _connected("circle", prefix="loopback-first") as client:
        chunk = await client.get_actions(_states(), schema=None)
        for motion_group_id in (LEFT_ID, RIGHT_ID):
            np.testing.assert_allclose(chunk.joints[motion_group_id][0], HOME, atol=1e-5)
        assert chunk.dt_ms == DT_MS


async def test_the_pattern_advances_with_the_client_step() -> None:
    """A quarter period of predicts reaches the sine axis's peak, which is what
    makes the commanded path knowable in advance from the tick count alone."""
    async with _connected("circle", prefix="loopback-advance") as client:
        first = await client.get_actions(_states(), schema=None)
        assert first.joints[LEFT_ID][0][MOVING_JOINT] == pytest.approx(HOME[MOVING_JOINT], abs=1e-3)

        for _ in range(QUARTER_PERIOD_STEPS):
            latest = await client.get_actions(_states(), schema=None)
        expected = HOME[MOVING_JOINT] + math.radians(AMPLITUDE_DEG)
        assert latest.joints[LEFT_ID][0][MOVING_JOINT] == pytest.approx(expected, abs=1e-3)


async def test_hold_commands_the_opening_pose_forever() -> None:
    async with _connected("hold", prefix="loopback-hold") as client:
        for _ in range(20):
            chunk = await client.get_actions(_states(), schema=None)
            np.testing.assert_allclose(chunk.joints[LEFT_ID][0], HOME, atol=1e-5)


async def test_a_dof_mismatch_fails_before_any_motion() -> None:
    """The client checks widths per step, so a wrong axis count raises instead of
    building a chunk from the wrong joints."""
    async with _connected("hold", prefix="loopback-dof") as client:
        with pytest.raises(Exception, match=r"axes|dof"):
            await client.get_actions(_states([0.0] * (DOF - 1)), schema=None)


async def test_schema_validation_accepts_two_bound_motion_groups() -> None:
    from novapolicy.schema import Observation, PolicySchema

    def _mg(motion_group_id: str) -> MagicMock:
        mg = MagicMock()
        mg.id = motion_group_id
        return mg

    schema = PolicySchema(
        observations=[
            Observation.joint_positions("left", source=_mg(LEFT_ID)),
            Observation.joint_positions("right", source=_mg(RIGHT_ID)),
        ]
    )
    async with _connected("hold", prefix="loopback-schema") as client:
        await client.validate_schema(schema)


async def test_a_horizon_arrives_as_a_multi_step_chunk() -> None:
    """The point of ``--horizon``: this is how the executor's chunk path gets a
    horizon of real future motion instead of one target padded out."""
    horizon = 25
    async with _connected(
        "updown", prefix="loopback-horizon", horizon=horizon, joints=[MOVING_JOINT]
    ) as client:
        chunk = await client.get_actions(_states(), schema=None)
        steps = chunk.joints[LEFT_ID]
        assert len(steps) == horizon
        moving = [step[MOVING_JOINT] for step in steps]
        assert all(b > a for a, b in pairwise(moving))


async def test_closing_twice_is_harmless() -> None:
    """An episode that fails mid-run closes on the way out and again in cleanup."""
    async with _serving("hold", prefix="loopback-close"):
        client = NatsPolicyClient(
            policy_id="mock-hold", servers=BROKER, subject_prefix="loopback-close"
        )
        await client.connect([LEFT_ID, RIGHT_ID])
        await client.get_actions(_states(), schema=None)
        await client.close()
        await client.close()


async def test_concurrent_clients_keep_separate_episodes() -> None:
    """Two clients on one mock must not share a centre or a step counter."""
    other_home = [value + 0.5 for value in HOME]
    async with _serving("hold", prefix="loopback-multi"):
        first = NatsPolicyClient(
            policy_id="mock-hold", servers=BROKER, subject_prefix="loopback-multi"
        )
        second = NatsPolicyClient(
            policy_id="mock-hold", servers=BROKER, subject_prefix="loopback-multi"
        )
        try:
            await asyncio.gather(
                first.connect([LEFT_ID, RIGHT_ID]), second.connect([LEFT_ID, RIGHT_ID])
            )
            one = await first.get_actions(_states(), schema=None)
            two = await second.get_actions(_states(other_home), schema=None)
            np.testing.assert_allclose(one.joints[LEFT_ID][0], HOME, atol=1e-5)
            np.testing.assert_allclose(two.joints[LEFT_ID][0], other_home, atol=1e-5)
        finally:
            await first.close()
            await second.close()
