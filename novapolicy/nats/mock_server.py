"""A policy service that emits fixed analytic motion, for debugging the client.

A trained policy is a poor instrument for debugging execution: its output depends
on what the robot just did, so a slow loop, a lagging arm and a bad checkpoint all
look alike, and no two runs are comparable. This serves the same NATS protocol
with motion that is a pure function of the policy step, so the commanded path is
known in advance and every run is identical.

That makes it the reference for questions about the *client*: what rate the loop
actually achieves, how far the arm trails the command, whether a horizon reaches
the controller as continuous motion. Point any client at it with no code change.

The patterns are joint-space, relative to wherever the robot is when the episode
opens, and every one of them starts at that pose. Nothing here knows a robot's
joint limits, so keep the amplitude small.

``python -m novapolicy.nats.mock_server`` serves one policy id per pattern::

    python -m novapolicy.nats.mock_server --servers nats://localhost:4222
    python -m novapolicy.nats.mock_server --horizon 25 --amplitude-deg 5

    # then, from a client
    NatsPolicyClient(policy_id="mock-circle", servers="nats://localhost:4222")

Rate follows the client, exactly as a real policy's does: one step of the pattern
is emitted per ``predict``, so a loop running at half the declared rate plays the
pattern at half speed. The observed interval per session is reported on ``close``,
which makes this a client-pacing diagnostic as well.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import logging
import math
import statistics
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from novapolicy.nats.protocol import (
    PROTOCOL_VERSION,
    ActionSpec,
    CloseRequest,
    CloseResponse,
    FieldSpec,
    PolicyInfo,
    PolicyProtocolError,
    PredictRequest,
    PredictResponse,
    ResetRequest,
    ResetResponse,
    close_subject,
    decode,
    encode,
    encode_error,
    info_subject,
    predict_subject,
    reset_subject,
)

if TYPE_CHECKING:
    from collections.abc import Awaitable, Callable

    from nats.aio.msg import Msg
    from numpy.typing import NDArray

logger = logging.getLogger(__name__)

PROPRIO_BLOCKS = 3
"""Proprio width per block: positions ++ velocities ++ last action."""

_POLICY_ID_PREFIX = "mock-"
"""Mock ids are distinct from any real policy's, so that a shared broker cannot
race a mock and a real service on one subject."""


# ---------------------------------------------------------------------------
# Patterns
# ---------------------------------------------------------------------------


def _phase(step: int, dt_ms: float, period_s: float) -> float:
    """Fraction of one period elapsed at ``step``, unwrapped."""
    return step * (dt_ms / 1000.0) / period_s


def _triangle(phase: float) -> float:
    """Triangle wave in [-1, 1], zero at phase 0, rising.

    Constant speed within each ramp, unlike a sine, so the distance the arm
    trails the command is a lag in milliseconds that can be read straight off a
    plot instead of being confounded with the command's own acceleration.
    """
    return 1.0 - abs(((4.0 * phase + 1.0) % 4.0) - 2.0)


def hold(center: NDArray[np.float32], step: int, cfg: PatternConfig) -> NDArray[np.float32]:
    """Stay at the episode's opening pose. The zero-motion control."""
    del step, cfg
    return center.copy()


def updown(center: NDArray[np.float32], step: int, cfg: PatternConfig) -> NDArray[np.float32]:
    """One joint on a sine, so the arm swings back and forth once per period."""
    target = center.copy()
    angle = 2.0 * math.pi * _phase(step, cfg.dt_ms, cfg.period_s)
    target[cfg.joints[0]] += cfg.amplitude_rad * math.sin(angle)
    return target


def triangle(center: NDArray[np.float32], step: int, cfg: PatternConfig) -> NDArray[np.float32]:
    """One joint at constant speed, reversing at each end of its travel."""
    target = center.copy()
    target[cfg.joints[0]] += cfg.amplitude_rad * _triangle(_phase(step, cfg.dt_ms, cfg.period_s))
    return target


def circle(center: NDArray[np.float32], step: int, cfg: PatternConfig) -> NDArray[np.float32]:
    """Two joints in quadrature: a circle in joint space, an arc in space.

    The second joint uses ``1 - cos`` rather than ``cos`` so that step zero is the
    opening pose on both axes. Plain ``cos`` would command the full amplitude
    immediately, putting a step discontinuity at the start of every episode.
    """
    target = center.copy()
    angle = 2.0 * math.pi * _phase(step, cfg.dt_ms, cfg.period_s)
    first, second = cfg.joints[0], cfg.joints[1 % len(cfg.joints)]
    target[first] += cfg.amplitude_rad * math.sin(angle)
    target[second] += cfg.amplitude_rad * (1.0 - math.cos(angle))
    return target


PATTERNS = {"hold": hold, "updown": updown, "triangle": triangle, "circle": circle}
"""Pattern name to generator. Each is served as policy id ``mock-<name>``."""


class PatternConfig:
    """Shape of the emitted motion, shared by every served pattern."""

    __slots__ = ("amplitude_rad", "dt_ms", "joints", "period_s")

    def __init__(
        self,
        *,
        amplitude_deg: float,
        period_s: float,
        dt_ms: float,
        joints: list[int],
        dof: int,
    ) -> None:
        if period_s <= 0:
            raise ValueError("period_s must be positive")
        if not joints:
            raise ValueError("at least one joint index is required")
        for index in joints:
            if not 0 <= index < dof:
                raise ValueError(f"joint index {index} outside 0..{dof - 1}")
        self.amplitude_rad = math.radians(amplitude_deg)
        self.period_s = period_s
        self.dt_ms = dt_ms
        self.joints = joints


# ---------------------------------------------------------------------------
# Policy
# ---------------------------------------------------------------------------


class _Session:
    """One episode: where it started, and how fast the client is asking."""

    __slots__ = ("centers", "intervals", "last_request_t", "steps")

    def __init__(self) -> None:
        self.centers: dict[str, NDArray[np.float32]] = {}
        self.steps = 0
        self.last_request_t: float | None = None
        self.intervals: list[float] = []

    def observe_arrival(self) -> None:
        now = time.monotonic()
        if self.last_request_t is not None:
            self.intervals.append((now - self.last_request_t) * 1000.0)
        self.last_request_t = now

    def pacing(self) -> str:
        if not self.intervals:
            return "no interval samples"
        median = statistics.median(self.intervals)
        return (
            f"{len(self.intervals) + 1} predicts, interval median {median:.1f}ms "
            f"({1000.0 / median:.1f}Hz), min {min(self.intervals):.1f} "
            f"max {max(self.intervals):.1f}"
        )


class MockPolicy:
    """Serves one pattern, per session, over the declared blocks."""

    def __init__(
        self,
        name: str,
        *,
        blocks: list[str],
        dof: int,
        horizon: int,
        config: PatternConfig,
    ) -> None:
        if name not in PATTERNS:
            raise ValueError(f"Unknown pattern {name!r}; known: {sorted(PATTERNS)}")
        self.name = name
        self.policy_id = f"{_POLICY_ID_PREFIX}{name}"
        self._pattern = PATTERNS[name]
        self._blocks = blocks
        self._dof = dof
        self._horizon = horizon
        self._config = config
        self._sessions: dict[str, _Session] = {}

    @property
    def info(self) -> PolicyInfo:
        """The advertised spaces. Named blocks need one proprio field each."""
        return PolicyInfo(
            protocol_version=PROTOCOL_VERSION,
            policy_id=self.policy_id,
            name=f"Mock '{self.name}' pattern",
            version="1.0.0",
            embodiment=f"any-{self._dof}dof",
            observations=[
                FieldSpec(name=f"proprio.{block}", shape=[PROPRIO_BLOCKS * self._dof])
                for block in self._blocks
            ],
            actions=[
                ActionSpec(
                    name=f"joint_target.{block}",
                    dof=self._dof,
                    horizon=self._horizon,
                    dt_ms=self._config.dt_ms,
                )
                for block in self._blocks
            ],
            description=(
                f"{self.name}: amplitude {math.degrees(self._config.amplitude_rad):.1f}deg, "
                f"period {self._config.period_s:.1f}s, joints {self._config.joints}, "
                f"relative to the pose at episode start"
            ),
        )

    def reset(self, session_id: str) -> None:
        self._sessions[session_id] = _Session()

    def close(self, session_id: str) -> tuple[int, str]:
        session = self._sessions.pop(session_id, None)
        return (0, "unknown session") if session is None else (session.steps, session.pacing())

    def predict(self, request: PredictRequest) -> PredictResponse:
        """One chunk of ``horizon`` steps, starting at the requested step.

        The episode's opening pose is taken from the first request's own
        observation, so the pattern is centred wherever the robot happens to be
        and needs no configuration per robot.
        """
        session = self._sessions.get(request.session_id)
        if session is None:
            raise PolicyProtocolError(
                f"No open session {request.session_id!r}; send reset first.",
                code="unknown_session",
            )
        session.observe_arrival()
        session.steps += 1

        actions: dict[str, NDArray[np.float32]] = {}
        for block in self._blocks:
            center = self._center(session, block, request)
            steps = [
                self._pattern(center, request.step + offset, self._config)
                for offset in range(self._horizon)
            ]
            actions[f"joint_target.{block}"] = np.asarray(steps, dtype=np.float32)

        return PredictResponse(
            session_id=request.session_id, actions=actions, dt_ms=self._config.dt_ms
        )

    def _center(
        self, session: _Session, block: str, request: PredictRequest
    ) -> NDArray[np.float32]:
        """The pose this block's pattern is centred on, captured once."""
        cached = session.centers.get(block)
        if cached is not None:
            return cached
        field = f"proprio.{block}"
        observed = request.observation.get(field)
        if observed is None:
            raise PolicyProtocolError(
                f"Observation {field!r} missing; got {sorted(request.observation)}.",
                code="obs_mismatch",
            )
        values = np.asarray(observed, dtype=np.float32).reshape(-1)
        if values.size < self._dof:
            raise PolicyProtocolError(
                f"Observation {field!r} is {values.size} wide, need at least "
                f"{self._dof} joint positions.",
                code="obs_mismatch",
            )
        center = values[: self._dof].copy()
        session.centers[block] = center
        return center


# ---------------------------------------------------------------------------
# Server
# ---------------------------------------------------------------------------


class MockPolicyServer:
    """Serves every pattern as its own policy id on one NATS connection."""

    def __init__(
        self,
        policies: list[MockPolicy],
        *,
        servers: list[str],
        subject_prefix: str = "policy",
    ) -> None:
        self._policies = {policy.policy_id: policy for policy in policies}
        self._servers = servers
        self._prefix = subject_prefix
        self._nc: Any = None

    async def start(self) -> None:
        import nats  # ruff: ignore[import-outside-top-level]

        self._nc = await nats.connect(
            servers=self._servers, max_reconnect_attempts=-1, name="mock-policy"
        )
        for policy_id, policy in self._policies.items():
            for subject, handler in (
                (info_subject(policy_id, self._prefix), self._handle_info),
                (reset_subject(policy_id, self._prefix), self._handle_reset),
                (predict_subject(policy_id, self._prefix), self._handle_predict),
                (close_subject(policy_id, self._prefix), self._handle_close),
            ):
                await self._nc.subscribe(subject, cb=self._guard(policy, handler))
            action = policy.info.actions[0]
            logger.info(
                "Serving '%s' blocks=%d dof=%d horizon=%d dt=%.1fms: %s",
                policy_id,
                len(policy.info.actions),
                action.dof,
                action.horizon,
                action.dt_ms,
                policy.info.description,
            )

    async def stop(self) -> None:
        if self._nc is not None:
            nc, self._nc = self._nc, None
            with contextlib.suppress(Exception):
                await nc.drain()

    def _guard(  # ruff: ignore[no-self-use]
        self, policy: MockPolicy, handler: Callable[[MockPolicy, Msg], bytes]
    ) -> Callable[[Msg], Awaitable[None]]:
        """Always answer: a silent failure here is a client-side timeout instead."""

        async def callback(msg: Msg) -> None:
            try:
                reply = handler(policy, msg)
            except PolicyProtocolError as exc:
                logger.warning("%s: %s", msg.subject, exc)
                reply = encode_error(str(exc), code=exc.code)
            except Exception as exc:
                logger.exception("%s: unhandled handler error", msg.subject)
                reply = encode_error(f"{type(exc).__name__}: {exc}", code="internal_error")
            if msg.reply:
                with contextlib.suppress(Exception):
                    await msg.respond(reply)

        return callback

    @staticmethod
    def _handle_info(policy: MockPolicy, msg: Msg) -> bytes:
        del msg
        return encode("info", policy.info.model_dump())

    @staticmethod
    def _handle_reset(policy: MockPolicy, msg: Msg) -> bytes:
        request = ResetRequest.model_validate(decode(msg.data, expect="reset"))
        policy.reset(request.session_id)
        logger.info(
            "[%s] reset session=%s motion_groups=%s",
            policy.policy_id,
            request.session_id,
            request.motion_group_ids,
        )
        return encode(
            "reset",
            ResetResponse(session_id=request.session_id, info=policy.info).model_dump(),
        )

    @staticmethod
    def _handle_predict(policy: MockPolicy, msg: Msg) -> bytes:
        request = PredictRequest.from_wire(decode(msg.data, expect="predict"))
        return encode("predict", policy.predict(request).to_wire())

    @staticmethod
    def _handle_close(policy: MockPolicy, msg: Msg) -> bytes:
        request = CloseRequest.model_validate(decode(msg.data, expect="close"))
        steps, pacing = policy.close(request.session_id)
        logger.info("[%s] close session=%s %s", policy.policy_id, request.session_id, pacing)
        return encode(
            "close",
            CloseResponse(session_id=request.session_id, steps=steps).model_dump(),
        )


def build_policies(
    *,
    blocks: list[str],
    dof: int,
    horizon: int,
    dt_ms: float,
    amplitude_deg: float,
    period_s: float,
    joints: list[int],
    patterns: list[str] | None = None,
) -> list[MockPolicy]:
    """One :class:`MockPolicy` per pattern, all sharing one motion shape."""
    config = PatternConfig(
        amplitude_deg=amplitude_deg,
        period_s=period_s,
        dt_ms=dt_ms,
        joints=joints,
        dof=dof,
    )
    return [
        MockPolicy(name, blocks=blocks, dof=dof, horizon=horizon, config=config)
        for name in (patterns or sorted(PATTERNS))
    ]


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--servers", default="nats://localhost:4222", help="Comma-separated")
    parser.add_argument("--subject-prefix", default="policy")
    parser.add_argument(
        "--blocks",
        default="left,right",
        help="Named blocks, in the order clients bind motion groups to them",
    )
    parser.add_argument("--dof", type=int, default=7, help="Joints per block")
    parser.add_argument(
        "--horizon",
        type=int,
        default=1,
        help="Steps per chunk. Above 1 the chunk carries real future motion",
    )
    parser.add_argument("--dt-ms", type=float, default=20.0, help="Declared step spacing")
    parser.add_argument("--amplitude-deg", type=float, default=10.0)
    parser.add_argument("--period-s", type=float, default=4.0)
    parser.add_argument(
        "--joints",
        default="1,3",
        help="Joint indices the patterns drive; the first two are used by 'circle'",
    )
    parser.add_argument("--log-level", default="INFO")
    return parser.parse_args(argv)


async def serve(args: argparse.Namespace) -> None:
    """Run until cancelled."""
    policies = build_policies(
        blocks=[b.strip() for b in args.blocks.split(",") if b.strip()],
        dof=args.dof,
        horizon=args.horizon,
        dt_ms=args.dt_ms,
        amplitude_deg=args.amplitude_deg,
        period_s=args.period_s,
        joints=[int(j) for j in args.joints.split(",") if j.strip()],
    )
    server = MockPolicyServer(
        policies,
        servers=[s.strip() for s in args.servers.split(",") if s.strip()],
        subject_prefix=args.subject_prefix,
    )
    await server.start()
    try:
        await asyncio.Event().wait()
    finally:
        await server.stop()


def main(argv: list[str] | None = None) -> None:
    args = _parse_args(argv)
    logging.basicConfig(
        level=args.log_level.upper(),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    with contextlib.suppress(KeyboardInterrupt):
        asyncio.run(serve(args))


if __name__ == "__main__":
    main()
