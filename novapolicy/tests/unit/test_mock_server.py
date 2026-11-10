"""Tests for the mock policy service's patterns and request handling.

No broker: the handlers are bytes in, bytes out, so encoding a request and
decoding the reply exercises the whole contract a client depends on.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from novapolicy.nats.mock_server import (
    PATTERNS,
    MockPolicy,
    MockPolicyServer,
    PatternConfig,
    build_policies,
    circle,
    hold,
    triangle,
    updown,
)
from novapolicy.nats.protocol import (
    CloseRequest,
    PolicyProtocolError,
    PredictRequest,
    PredictResponse,
    ResetRequest,
    ResetResponse,
    decode,
    encode,
)

DOF = 7
BLOCKS = ["left", "right"]
CENTER = np.arange(DOF, dtype=np.float32) / 10.0
DT_MS = 20.0
PERIOD_S = 4.0
AMPLITUDE_DEG = 10.0
STEPS_PER_PERIOD = int(PERIOD_S * 1000 / DT_MS)


def _config(joints: list[int] | None = None) -> PatternConfig:
    return PatternConfig(
        amplitude_deg=AMPLITUDE_DEG,
        period_s=PERIOD_S,
        dt_ms=DT_MS,
        joints=joints or [1, 3],
        dof=DOF,
    )


def _policy(pattern: str = "circle", *, horizon: int = 1) -> MockPolicy:
    return MockPolicy(pattern, blocks=BLOCKS, dof=DOF, horizon=horizon, config=_config())


class _Msg:
    """Just the surface a handler reads off a NATS message."""

    def __init__(self, data: bytes) -> None:
        self.data = data
        self.subject = "policy.mock-circle.test"
        self.reply = "inbox"


def _open_session(policy: MockPolicy, session_id: str = "s1") -> None:
    MockPolicyServer._handle_reset(
        policy,
        _Msg(
            encode(
                "reset",
                ResetRequest(session_id=session_id, motion_group_ids=["1@m"]).model_dump(),
            )
        ),
    )


def _predict(policy: MockPolicy, step: int, *, session_id: str = "s1") -> PredictResponse:
    observation = {f"proprio.{block}": np.tile(CENTER, 3) for block in BLOCKS}
    request = PredictRequest(
        session_id=session_id, step=step, t_ms=step * DT_MS, observation=observation
    )
    reply = MockPolicyServer._handle_predict(policy, _Msg(encode("predict", request.to_wire())))
    return PredictResponse.from_wire(decode(reply, expect="predict"))


# -- patterns ---------------------------------------------------------------


@pytest.mark.parametrize("pattern", sorted(PATTERNS))
def test_every_pattern_starts_at_the_opening_pose(pattern: str) -> None:
    """Step zero must command exactly where the robot already is.

    Anything else puts a step discontinuity at the start of every episode, which
    is the one artefact a debugging baseline must not have.
    """
    target = PATTERNS[pattern](CENTER, 0, _config())
    np.testing.assert_allclose(target, CENTER, atol=1e-6)


def test_hold_never_moves() -> None:
    for step in (0, 7, STEPS_PER_PERIOD, 10 * STEPS_PER_PERIOD):
        np.testing.assert_allclose(hold(CENTER, step, _config()), CENTER, atol=1e-6)


def test_updown_is_a_sine_on_one_joint() -> None:
    cfg = _config(joints=[1])
    quarter = updown(CENTER, STEPS_PER_PERIOD // 4, cfg)
    assert quarter[1] == pytest.approx(CENTER[1] + math.radians(AMPLITUDE_DEG), abs=1e-3)
    # Every other joint is untouched, so a plot of the pattern has one moving line.
    untouched = [j for j in range(DOF) if j != 1]
    np.testing.assert_allclose(quarter[untouched], CENTER[untouched], atol=1e-6)
    three_quarter = updown(CENTER, 3 * STEPS_PER_PERIOD // 4, cfg)
    assert three_quarter[1] == pytest.approx(CENTER[1] - math.radians(AMPLITUDE_DEG), abs=1e-3)


def test_updown_returns_to_the_centre_after_one_period() -> None:
    cfg = _config(joints=[1])
    np.testing.assert_allclose(updown(CENTER, STEPS_PER_PERIOD, cfg), CENTER, atol=1e-6)


def test_triangle_moves_at_constant_speed() -> None:
    """The reason this pattern exists: equal steps mean lag reads as a lag."""
    cfg = _config(joints=[2])
    rising = [triangle(CENTER, step, cfg)[2] for step in range(STEPS_PER_PERIOD // 4)]
    deltas = np.diff(rising)
    assert deltas.min() > 0
    # rtol is float32 rounding, not a slack allowance for a curved ramp.
    np.testing.assert_allclose(deltas, deltas[0], rtol=1e-4)


def test_circle_is_quadrature_on_two_joints() -> None:
    cfg = _config(joints=[1, 3])
    amplitude = math.radians(AMPLITUDE_DEG)
    quarter = circle(CENTER, STEPS_PER_PERIOD // 4, cfg)
    assert quarter[1] == pytest.approx(CENTER[1] + amplitude, abs=1e-3)
    assert quarter[3] == pytest.approx(CENTER[3] + amplitude, abs=1e-3)
    half = circle(CENTER, STEPS_PER_PERIOD // 2, cfg)
    assert half[1] == pytest.approx(CENTER[1], abs=1e-3)
    assert half[3] == pytest.approx(CENTER[3] + 2 * amplitude, abs=1e-3)


def test_circle_stays_within_the_declared_amplitude() -> None:
    """Two amplitudes on the ``1 - cos`` axis, and no more, so the excursion a
    caller has to check against joint limits is bounded by what it configured."""
    cfg = _config(joints=[1, 3])
    reach = [circle(CENTER, step, cfg) for step in range(2 * STEPS_PER_PERIOD)]
    offsets = np.abs(np.asarray(reach) - CENTER).max(axis=0)
    assert offsets[1] == pytest.approx(math.radians(AMPLITUDE_DEG), abs=1e-3)
    assert offsets[3] == pytest.approx(2 * math.radians(AMPLITUDE_DEG), abs=1e-3)


def test_pattern_phase_follows_the_step_not_the_clock() -> None:
    """A client running at half rate must play the pattern at half speed, like a
    real policy advancing one reference frame per predict."""
    cfg = _config(joints=[1])
    assert updown(CENTER, 10, cfg)[1] == pytest.approx(updown(CENTER, 10, cfg)[1])
    assert updown(CENTER, 10, cfg)[1] != pytest.approx(updown(CENTER, 20, cfg)[1])


# -- configuration guards ---------------------------------------------------


def test_joint_index_outside_the_dof_is_rejected() -> None:
    with pytest.raises(ValueError, match="outside"):
        PatternConfig(amplitude_deg=1.0, period_s=1.0, dt_ms=DT_MS, joints=[DOF], dof=DOF)


def test_unknown_pattern_is_rejected() -> None:
    with pytest.raises(ValueError, match="Unknown pattern"):
        MockPolicy("spiral", blocks=BLOCKS, dof=DOF, horizon=1, config=_config())


def test_build_policies_serves_every_pattern_under_a_mock_id() -> None:
    policies = build_policies(
        blocks=BLOCKS,
        dof=DOF,
        horizon=1,
        dt_ms=DT_MS,
        amplitude_deg=AMPLITUDE_DEG,
        period_s=PERIOD_S,
        joints=[1, 3],
    )
    ids = {policy.policy_id for policy in policies}
    assert ids == {f"mock-{name}" for name in PATTERNS}


# -- protocol ---------------------------------------------------------------


def test_info_declares_one_proprio_field_per_action_block() -> None:
    """The client refuses to bind a named action without its matching proprio."""
    info = _policy().info
    assert [action.name for action in info.actions] == [
        "joint_target.left",
        "joint_target.right",
    ]
    assert [field.name for field in info.observations] == [
        "proprio.left",
        "proprio.right",
    ]
    for field in info.observations:
        assert field.shape == [3 * DOF]
    assert {action.dt_ms for action in info.actions} == {DT_MS}


def test_reset_returns_the_same_spec_the_client_rebinds_from() -> None:
    policy = _policy()
    reply = MockPolicyServer._handle_reset(
        policy,
        _Msg(encode("reset", ResetRequest(session_id="s1", motion_group_ids=[]).model_dump())),
    )
    response = ResetResponse.model_validate(decode(reply, expect="reset"))
    assert response.session_id == "s1"
    assert response.info.model_dump() == policy.info.model_dump()


def test_predict_returns_horizon_by_dof_per_block() -> None:
    policy = _policy(horizon=25)
    _open_session(policy)
    response = _predict(policy, 0)
    assert sorted(response.actions) == ["joint_target.left", "joint_target.right"]
    for chunk in response.actions.values():
        assert chunk.shape == (25, DOF)
        assert chunk.dtype == np.float32
    assert response.dt_ms == DT_MS


def test_a_horizon_carries_real_future_motion() -> None:
    """The point of ``--horizon``: consecutive steps differ, so the chunk is a
    path rather than one target repeated."""
    policy = _policy("updown", horizon=25)
    _open_session(policy)
    chunk = _predict(policy, 0).actions["joint_target.left"]
    moving = chunk[:, 1]
    assert np.all(np.diff(moving) > 0)


def test_chunks_from_successive_steps_agree_where_they_overlap() -> None:
    """Step ``k+1``'s chunk continues step ``k``'s, so a client that replaces a
    chunk mid-flight does not command a different path for the same moment."""
    policy = _policy("circle", horizon=10)
    _open_session(policy)
    first = _predict(policy, 5).actions["joint_target.left"]
    second = _predict(policy, 6).actions["joint_target.left"]
    np.testing.assert_allclose(first[1:], second[:-1], atol=1e-6)


def test_the_pattern_is_centred_on_the_first_observation() -> None:
    policy = _policy("hold")
    _open_session(policy)
    response = _predict(policy, 0)
    np.testing.assert_allclose(response.actions["joint_target.left"][0], CENTER, atol=1e-6)


def test_the_centre_is_captured_once_and_then_held() -> None:
    """A later observation must not re-centre the pattern, or the commanded path
    would drift with the robot instead of being fixed in advance."""
    policy = _policy("hold")
    _open_session(policy)
    _predict(policy, 0)
    moved = {f"proprio.{block}": np.tile(CENTER + 1.0, 3) for block in BLOCKS}
    request = PredictRequest(session_id="s1", step=1, t_ms=DT_MS, observation=moved)
    reply = MockPolicyServer._handle_predict(policy, _Msg(encode("predict", request.to_wire())))
    response = PredictResponse.from_wire(decode(reply, expect="predict"))
    np.testing.assert_allclose(response.actions["joint_target.left"][0], CENTER, atol=1e-6)


def test_predict_without_a_session_is_an_error_reply() -> None:
    """The client raises on a typed error; a silent success would move a robot
    from an unknown centre."""
    policy = _policy()
    with pytest.raises(PolicyProtocolError, match="No open session"):
        _predict(policy, 0, session_id="never-opened")


def test_a_missing_observation_field_is_an_error_reply() -> None:
    policy = _policy()
    _open_session(policy)
    request = PredictRequest(
        session_id="s1",
        step=0,
        t_ms=0.0,
        observation={"proprio.left": np.tile(CENTER, 3)},
    )
    with pytest.raises(PolicyProtocolError, match=r"proprio\.right"):
        MockPolicyServer._handle_predict(policy, _Msg(encode("predict", request.to_wire())))


def test_a_short_observation_is_an_error_reply() -> None:
    policy = _policy()
    _open_session(policy)
    short = {f"proprio.{block}": np.zeros(DOF - 1, dtype=np.float32) for block in BLOCKS}
    request = PredictRequest(session_id="s1", step=0, t_ms=0.0, observation=short)
    with pytest.raises(PolicyProtocolError, match="wide"):
        MockPolicyServer._handle_predict(policy, _Msg(encode("predict", request.to_wire())))


def test_close_reports_the_served_step_count() -> None:
    policy = _policy()
    _open_session(policy)
    for step in range(3):
        _predict(policy, step)
    reply = MockPolicyServer._handle_close(
        policy, _Msg(encode("close", CloseRequest(session_id="s1").model_dump()))
    )
    body = decode(reply, expect="close")
    assert body["steps"] == 3


def test_closing_an_unknown_session_still_answers() -> None:
    policy = _policy()
    reply = MockPolicyServer._handle_close(
        policy, _Msg(encode("close", CloseRequest(session_id="ghost").model_dump()))
    )
    assert decode(reply, expect="close")["steps"] == 0


def test_sessions_keep_separate_centres() -> None:
    policy = _policy("hold")
    _open_session(policy, "a")
    _open_session(policy, "b")
    _predict(policy, 0, session_id="a")

    other = {f"proprio.{block}": np.tile(CENTER + 0.5, 3) for block in BLOCKS}
    request = PredictRequest(session_id="b", step=0, t_ms=0.0, observation=other)
    reply = MockPolicyServer._handle_predict(policy, _Msg(encode("predict", request.to_wire())))
    response = PredictResponse.from_wire(decode(reply, expect="predict"))
    np.testing.assert_allclose(response.actions["joint_target.left"][0], CENTER + 0.5, atol=1e-6)
