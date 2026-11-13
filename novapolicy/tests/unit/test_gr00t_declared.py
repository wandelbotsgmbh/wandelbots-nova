"""Tests for running a GR00T policy as its policy.toml declares it."""

from __future__ import annotations

import dataclasses
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from novapolicy.gr00t import policy_config
from novapolicy.gr00t.client import Gr00tPolicyClient
from novapolicy.gr00t.declared import (
    ChunkLimitError,
    DeclaredGr00tClient,
    GuardError,
    PlanQueue,
    check_guards,
    frame_images,
    gripper_state,
    next_gripper_switch,
    tcp_targets,
)
from novapolicy.schema import Action, Observation, PolicySchema
from novapolicy.types import ActionChunk

EXAMPLE = Path(__file__).parents[2] / "examples" / "gr00t_single_arm_pick_and_place" / "policy.toml"
CFG = policy_config.load(EXAMPLE)
DT = CFG.policy.dt_ms
# Tool pointing straight down above the table, inside the example's workspace.
DOWN = [-100.0, -500.0, 300.0, math.pi, 0.0, 0.0]


def _with_action(**changes: object) -> policy_config.PolicyConfig:
    action = dataclasses.replace(CFG.policy.tcp_action, **changes)
    return dataclasses.replace(CFG, policy=dataclasses.replace(CFG.policy, tcp_action=action))


def test_frame_images_crops_to_the_training_aspect_then_resizes():
    wide = np.zeros((720, 1280, 3), dtype=np.uint8)
    wide[:, :160] = 255  # only in the strip a 4:3 centre crop removes
    out = frame_images({"top": wide}, 320, 240)["top"]
    assert out.shape == (240, 320, 3)
    assert out.max() == 0


def test_tcp_targets_scales_and_accumulates_base_frame_deltas():
    deltas = [[0.01, 0.0, -0.02, 0.0, 0.0, 0.0]] * 3
    targets = tcp_targets(CFG, DOWN, deltas)
    gain = CFG.policy.tcp_action.gain_position
    for i, t in enumerate(targets, start=1):
        assert t[0] == pytest.approx(DOWN[0] + i * 10.0 * gain)
        assert t[2] == pytest.approx(DOWN[2] - i * 20.0 * gain)
        assert t[3:] == pytest.approx(DOWN[3:])


def test_tcp_targets_rotates_about_the_tcp_in_the_base_frame():
    cfg = _with_action(gain_rotation=1.0)
    yaw = math.radians(10.0)
    targets = tcp_targets(cfg, DOWN, [[0.0, 0.0, 0.0, 0.0, 0.0, yaw]])
    assert targets[0][:3] == pytest.approx(DOWN[:3])  # rotating about the TCP keeps it put
    expected = Rotation.from_rotvec([0.0, 0.0, yaw]) * Rotation.from_rotvec(DOWN[3:])
    got = Rotation.from_rotvec(targets[0][3:])
    assert (got * expected.inv()).magnitude() == pytest.approx(0.0, abs=1e-9)


def test_tcp_targets_in_the_tool_frame_follow_the_tool_axes():
    cfg = _with_action(frame="tool", gain_position=1.0)
    targets = tcp_targets(cfg, DOWN, [[0.0, 0.0, 0.01, 0.0, 0.0, 0.0]])  # 10 mm along tool z
    assert targets[0][:3] == pytest.approx([DOWN[0], DOWN[1], DOWN[2] - 10.0])


def test_tcp_targets_stay_continuous_near_a_half_turn():
    cfg = _with_action(gain_rotation=1.0)
    deltas = [[0.0, 0.0, 0.0, 0.0, 0.0, math.radians(2.0)]] * 10
    targets = tcp_targets(cfg, [*DOWN[:3], 0.0, -math.pi, 0.0], deltas)
    rotvecs = np.asarray([t[3:] for t in targets])
    assert np.abs(np.diff(rotvecs, axis=0)).max() < 0.1  # no sign flip between steps


def test_check_guards_accepts_a_small_motion_and_names_each_broken_guard():
    ok = tcp_targets(CFG, DOWN, [[0.005, 0.0, -0.005, 0.0, 0.0, 0.0]] * 8)
    check_guards(CFG, DOWN, ok, DT)
    low = [[*DOWN[:2], 10.0, *DOWN[3:]]]
    with pytest.raises(GuardError, match="below"):
        check_guards(CFG, [*DOWN[:2], 12.0, *DOWN[3:]], low, DT)
    jump = [[DOWN[0] + 100.0, *DOWN[1:]]]
    with pytest.raises(GuardError, match="step 0: step"):
        check_guards(CFG, DOWN, jump, DT)
    outside = [[*DOWN[:1], -100.0, *DOWN[2:]]]
    with pytest.raises(GuardError, match="outside workspace"):
        check_guards(CFG, [*DOWN[:1], -101.0, *DOWN[2:]], outside, DT)


def test_gripper_state_maps_the_jaw_gap_onto_the_declared_values():
    g = CFG.policy.gripper
    assert gripper_state(CFG, CFG.cell.gripper_open_gap_mm, holding=False) == g.state_open
    assert gripper_state(CFG, 0.0, holding=False) == g.state_closed
    half = gripper_state(CFG, CFG.cell.gripper_open_gap_mm / 2, holding=False)
    assert half == pytest.approx((g.state_open + g.state_closed) / 2)
    assert gripper_state(CFG, 30.0, holding=True) == g.state_holding


def test_next_gripper_switch_only_looks_at_executed_steps():
    actions = [1.0, 1.0, 1.0, -1.0, -1.0]
    assert next_gripper_switch(CFG, actions, 5, closed=False) == (3, True)
    assert next_gripper_switch(CFG, actions, 3, closed=False) is None
    assert next_gripper_switch(CFG, actions, 5, closed=None) == (0, False)


def _line(x0: float, n: int, step: float = 1.0) -> list[list[float]]:
    return [[x0 + i * step, *DOWN[1:]] for i in range(n)]


def test_plan_queue_keeps_the_frozen_head_and_blends_the_rest():
    q = PlanQueue(DT, smoothing_passes=0, margin_steps=1, old_weight=0.5)
    first, plan = q.merge(_line(0.0, 8), latency_ms=0.0)
    assert first == 0
    assert [t[0] for t in plan] == pytest.approx(range(8))
    q.replaceable = 2  # the executor reports where replacement may start
    first, plan = q.merge(_line(100.0, 8), latency_ms=DT)  # one step in flight + one margin
    assert first == 1  # one step before the replaceable one anchors the seam
    assert [t[0] for t in plan[:3]] == pytest.approx([1.0, 2.0, 3.0])  # frozen as published
    assert plan[3][0] == pytest.approx(0.5 * 4.0 + 0.5 * 102.0)  # blended old/new


def test_plan_queue_hold_stands_still_after_the_head():
    q = PlanQueue(DT, smoothing_passes=0, hold_steps=3)
    assert q.hold() is None
    q.merge(_line(0.0, 8), latency_ms=0.0)
    held = q.hold()
    assert held is not None
    first, plan = held
    assert first == 0
    assert [t[0] for t in plan[-4:]] == pytest.approx([plan[-4][0]] * 4)


def _mg() -> MagicMock:
    mg = MagicMock()
    mg.id = "0@ur10e"
    mg._controller_id = "ur10e"
    mg._cell = "cell"
    return mg


def _schema(
    mg: MagicMock, *, gripper: bool = True, cameras: tuple[str, ...] = ("wrist", "side", "top")
):
    async def observe(_obs):
        return {"gripper": 0.0}

    observations = [
        Observation.joint_positions("arm_joints", source=mg, action=False),
        Observation.tcp("tcp_pose", source=mg, tcp=CFG.cell.tcp),
        *(Observation.image(key, source=MagicMock()) for key in cameras),
    ]
    if gripper:
        observations.append(Observation.computed(observe))
    return PolicySchema(
        observations=observations, actions=[Action.tcp("eef_delta", target=mg, mode="absolute")]
    )


SERVED: dict[str, object] = {
    "state_keys": ["arm_joints", "gripper"],
    "action_keys": ["eef_delta", "gripper"],
    "video_keys": ["wrist", "side", "top"],
    "language_keys": ["annotation.human.task_description"],
    "action_horizon": 16,
}


def _client(served: dict[str, object] = SERVED, **kwargs: object) -> DeclaredGr00tClient:
    client = DeclaredGr00tClient(CFG, **kwargs)

    async def server_info() -> dict[str, object]:
        return served

    client.get_server_info = server_info  # ty: ignore[invalid-assignment]
    return client


@pytest.mark.asyncio
async def test_validate_schema_accepts_the_declared_policy_with_a_computed_gripper():
    await _client().validate_schema(_schema(_mg()))


@pytest.mark.asyncio
async def test_validate_schema_rejects_another_policy_or_a_schema_that_misses_inputs():
    with pytest.raises(ValueError, match="language_keys"):
        served = {**SERVED, "language_keys": ["annotation.language.language_instruction"]}
        await _client(served).validate_schema(_schema(_mg()))
    with pytest.raises(ValueError, match="gripper"):
        await _client().validate_schema(_schema(_mg(), gripper=False))
    with pytest.raises(ValueError, match="top"):
        await _client().validate_schema(_schema(_mg(), cameras=("wrist", "side")))


def _states(mg: MagicMock, pose: list[float], tcp: str = CFG.cell.tcp) -> dict[str, object]:
    state = SimpleNamespace(tcp=tcp, pose=SimpleNamespace(position=pose[:3], orientation=pose[3:]))
    return {mg.id: state}


def _serve(monkeypatch: pytest.MonkeyPatch, mg: MagicMock, deltas: list[list[float]]) -> list:
    seen: list = []

    async def raw(self, states, schema, images=None, io_values=None):
        seen.append(images)
        return ActionChunk(tcp={mg.id: deltas}, dt_ms=50.0)

    monkeypatch.setattr(Gr00tPolicyClient, "get_actions", raw)
    return seen


@pytest.mark.asyncio
async def test_sequential_get_actions_returns_guarded_absolute_targets(monkeypatch):
    mg = _mg()
    seen = _serve(monkeypatch, mg, [[0.005, 0.0, 0.0, 0.0, 0.0, 0.0]] * 16)
    client = _client(n_action_steps=8, near_z_mm=120.0, near_steps=4, speed_factor=0.5)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    chunk = await client.get_actions(_states(mg, DOWN), _schema(mg), {"top": frame})
    assert seen[0]["top"].shape == (240, 320, 3)  # framed like the training cameras
    assert chunk.dt_ms == pytest.approx(2 * DT)  # the declared spacing, slowed down
    assert len(chunk.tcp[mg.id]) == 8
    assert chunk.tcp[mg.id][0][0] == pytest.approx(
        DOWN[0] + 5.0 * CFG.policy.tcp_action.gain_position
    )
    low = [*DOWN[:2], 100.0, *DOWN[3:]]
    chunk = await client.get_actions(_states(mg, low), _schema(mg))
    assert len(chunk.tcp[mg.id]) == 4  # near the table: re-observe sooner


@pytest.mark.asyncio
async def test_get_actions_refuses_a_chunk_that_breaks_a_guard_or_the_wrong_tcp(monkeypatch):
    mg = _mg()
    _serve(monkeypatch, mg, [[0.0, 0.0, -0.5, 0.0, 0.0, 0.0]] * 16)  # dives into the table
    with pytest.raises(GuardError):
        await _client().get_actions(_states(mg, DOWN), _schema(mg))
    with pytest.raises(RuntimeError, match="declares"):
        await _client().get_actions(_states(mg, DOWN, tcp="flange"), _schema(mg))


@pytest.mark.asyncio
async def test_get_actions_holds_still_while_asked_to(monkeypatch):
    mg = _mg()
    seen = _serve(monkeypatch, mg, [[0.005, 0.0, 0.0, 0.0, 0.0, 0.0]] * 16)
    client = _client(hold_while=lambda: True)
    chunk = await client.get_actions(_states(mg, DOWN), _schema(mg))
    assert not seen  # the policy was not asked
    assert all(t == pytest.approx(DOWN) for t in chunk.tcp[mg.id])


@pytest.mark.asyncio
async def test_continuous_get_actions_places_chunks_on_the_timestep_timeline(monkeypatch):
    mg = _mg()
    _serve(monkeypatch, mg, [[0.002, 0.0, 0.0, 0.0, 0.0, 0.0]] * 16)
    client = _client(continuous=True)
    assert client.requires_first_waypoint_bridge
    assert client.n_action_steps is None
    chunk = await client.get_actions(_states(mg, DOWN), _schema(mg))
    assert chunk.action_timestep == 0
    client.synchronize_action_timestep(5)
    chunk = await client.get_actions(_states(mg, DOWN), _schema(mg))
    assert chunk.action_timestep == 4  # the published step before the replaceable one


@pytest.mark.asyncio
async def test_preview_converts_and_checks_without_raising(monkeypatch):
    mg = _mg()
    _serve(monkeypatch, mg, [[0.0, 0.0, -0.5, 0.0, 0.0, 0.0]] * 16)  # dives into the table
    preview = (await _client(n_action_steps=8).preview(_states(mg, DOWN), _schema(mg)))[mg.id]
    assert len(preview.targets) == 16
    assert preview.steps == 8
    assert any("below" in v for v in preview.violations)


@pytest.mark.asyncio
async def test_the_chunk_limit_raises_its_own_error(monkeypatch):
    mg = _mg()
    _serve(monkeypatch, mg, [[0.002, 0.0, 0.0, 0.0, 0.0, 0.0]] * 16)
    client = _client(max_chunks=1)
    await client.get_actions(_states(mg, DOWN), _schema(mg))
    with pytest.raises(ChunkLimitError):
        await client.get_actions(_states(mg, DOWN), _schema(mg))
