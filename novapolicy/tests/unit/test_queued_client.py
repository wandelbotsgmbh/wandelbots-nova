"""Tests for merging chunk-per-observation policies into one continuous plan."""

from __future__ import annotations

from itertools import pairwise
import math
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from novapolicy.policy_client import PolicyClient
from novapolicy.queued_client import QueuedPolicyClient
from novapolicy.schema import Action, Observation, PolicySchema
from novapolicy.types import ActionChunk

DT = 1000.0 / 30.0
START = [0.0, -500.0, 300.0, math.pi, 0.0, 0.0]


class _Chunks(PolicyClient):
    """Answers with the next prepared chunk."""

    def __init__(self, chunks: list[ActionChunk]) -> None:
        self.chunks = chunks

    async def get_actions(self, states, schema, images=None, io_values=None) -> ActionChunk:
        return self.chunks.pop(0)


def _mg() -> MagicMock:
    mg = MagicMock()
    mg.id = "0@ur10e"
    mg._controller_id = "ur10e"
    mg._cell = "cell"
    return mg


def _setup(chunks: list[list[list[float]]], **kwargs: object):
    mg = _mg()
    schema = PolicySchema(
        observations=[Observation.tcp("tcp_pose", source=mg, tcp="flange")],
        actions=[Action.tcp("eef_delta", target=mg, mode="relative", accumulate=True)],
    )
    inner = _Chunks([ActionChunk(tcp={mg.id: c}, dt_ms=DT) for c in chunks])
    pose = SimpleNamespace(position=START[:3], orientation=START[3:])
    states = {mg.id: SimpleNamespace(joints=[0.0] * 6, pose=pose)}
    return QueuedPolicyClient(inner, smoothing_passes=0, **kwargs), schema, states, mg.id


@pytest.mark.asyncio
async def test_the_first_chunk_is_resolved_and_placed_at_timestep_zero():
    client, schema, states, mg = _setup([[[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]] * 4])
    assert client.requires_first_waypoint_bridge
    chunk = await client.get_actions(states, schema)
    assert chunk.resolved
    assert chunk.action_timestep == 0
    assert [round(s[0], 3) for s in chunk.tcp[mg]] == [1.0, 2.0, 3.0, 4.0]  # accumulated


@pytest.mark.asyncio
async def test_a_replacement_keeps_the_frozen_head_and_blends_the_rest():
    step = [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]] * 8
    jump = [[10.0, 0.0, 0.0, 0.0, 0.0, 0.0]] * 8
    client, schema, states, mg = _setup([step, jump], margin_steps=1)
    first = await client.get_actions(states, schema)
    client.synchronize_action_timestep(3)
    second = await client.get_actions(states, schema)
    assert second.action_timestep == 2  # one published step before the replaceable one
    old = {first.action_timestep + i: s for i, s in enumerate(first.tcp[mg])}
    head = second.tcp[mg][:2]  # the predecessor and the frozen margin step, as published
    assert [s[0] for s in head[:2]] == pytest.approx([old[2][0], old[3][0]])
    blended = second.tcp[mg][3]  # timestep 5: 0.3 old + 0.7 new
    assert blended[0] == pytest.approx(0.3 * old[5][0] + 0.7 * 30.0, abs=1e-3)


@pytest.mark.asyncio
async def test_rotation_vectors_do_not_flip_across_a_seam():
    yaw = [[0.0, 0.0, 0.0, 0.0, 0.0, math.radians(2.0)]] * 8
    client, schema, states, mg = _setup([yaw, yaw])
    await client.get_actions(states, schema)
    client.synchronize_action_timestep(2)
    second = await client.get_actions(states, schema)
    rotvecs = [s[3:] for s in second.tcp[mg]]
    jumps = [max(abs(a - b) for a, b in zip(r0, r1, strict=True)) for r0, r1 in pairwise(rotvecs)]
    assert max(jumps) < 0.2
