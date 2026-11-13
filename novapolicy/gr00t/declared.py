"""Run a GR00T policy as its ``policy.toml`` declares it.

Everything here follows from a :class:`~novapolicy.gr00t.policy_config.PolicyConfig`, so a
deployment script only builds the schema and picks an execution mode:

- :class:`DeclaredGr00tClient`: sends the prompt under the declared language key, keeps the
  declared state keys, frames camera images like the training cameras, forces the declared
  step spacing, turns the declared end-effector action into absolute TCP targets, checks
  the guards, and checks the server against the config.
- :func:`tcp_targets`, :func:`check_guards`, :func:`frame_images`, :func:`gripper_state`,
  :func:`next_gripper_switch`: the same steps on their own.
- :class:`PlanQueue`: continuous execution without jumps at chunk seams.
- :meth:`DeclaredGr00tClient.preview`: one inference converted and checked, for dry runs.
- :func:`lift_if_low`, :func:`move_home`: lift straight up when low, then the joint move
  to the declared home.
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
import math
import time
from typing import TYPE_CHECKING, Any, cast

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from nova.actions import joint_ptp, linear
from nova.types import MotionSettings, Pose
from novapolicy.chunking import smooth_action_chunk, unwrap_tcp_rotations
from novapolicy.gr00t.client import Gr00tPolicyClient
from novapolicy.types import ActionChunk

if TYPE_CHECKING:
    from collections.abc import Callable

    from nova.cell.motion_group import MotionGroup
    from nova.types import RobotState
    from novapolicy.gr00t.policy_config import PolicyConfig
    from novapolicy.schema import PolicySchema

Targets = list[list[float]]  # [x, y, z, rx, ry, rz] per step, mm and rotation vector


class GuardError(RuntimeError):
    """A chunk would break a guard declared in ``policy.toml``."""


class ChunkLimitError(RuntimeError):
    """Sequential run reached ``max_chunks``; the arm stands still at the last chunk's end."""


@dataclass(frozen=True)
class Preview:
    """One inference for one motion group, converted and checked but not executed."""

    deltas: list[list[float]]  # the raw end-effector action, all horizon steps
    targets: Targets  # absolute TCP targets for all horizon steps
    steps: int  # how many of them would run before the next inference
    violations: list[str]  # guards the running steps break; empty when none
    gripper_actions: list[float]  # the raw gripper action
    inference_ms: float


# --- Conversions -------------------------------------------------------------------------------


def frame_images(frames: dict[str, np.ndarray], width: int, height: int) -> dict[str, np.ndarray]:
    """Frame camera images like the training cameras: centre-crop to the training aspect,
    then resize to the training resolution."""
    out: dict[str, np.ndarray] = {}
    for key, frame in frames.items():
        h, w = frame.shape[:2]
        crop_w = min(w, round(h * width / height))
        crop_h = min(h, round(w * height / width))
        x0, y0 = (w - crop_w) // 2, (h - crop_h) // 2
        image = Image.fromarray(frame[y0 : y0 + crop_h, x0 : x0 + crop_w])
        out[key] = np.asarray(image.resize((width, height), Image.Resampling.LANCZOS))
    return out


def tcp_targets(cfg: PolicyConfig, start: list[float], deltas: list[list[float]]) -> Targets:
    """Absolute TCP targets for the declared end-effector action, starting at ``start``.

    Each step's delta (in the declared frame and unit, scaled by the declared gains) is
    applied to the pose the previous step reached; rotations pivot about the TCP. The result
    is unwrapped onto ``start``'s rotation vector, ready for component-wise interpolation.
    """
    action = cfg.policy.tcp_action
    p = np.asarray(start[:3], dtype=float)
    r: Rotation = Rotation.from_rotvec(start[3:6])
    to_nova = Rotation.from_euler("z", action.base_rotation_z_deg, degrees=True)
    out: Targets = []
    for step in deltas:
        dp = np.asarray(step[:3], dtype=float) * action.position_scale_mm * action.gain_position
        rv = np.asarray(step[3:6], dtype=float) * action.gain_rotation
        frame = r if action.frame == "tool" else to_nova
        dp, rv = frame.apply(dp), frame.apply(rv)
        p += dp
        r = cast("Rotation", Rotation.from_rotvec(rv) * r)
        out.append([*p.tolist(), *r.as_rotvec().tolist()])
    return unwrap_tcp_rotations(out, start[3:6])


def guard_violations(
    cfg: PolicyConfig, start: list[float], targets: Targets, dt_ms: float
) -> list[str]:
    """Every guard a chunk breaks, as readable text; empty when it breaks none."""
    g = cfg.guards
    max_step_mm = g.max_tcp_speed_mm_s * dt_ms / 1000.0
    broken: list[str] = []
    prev = start
    for i, t in enumerate(targets):
        step_mm = math.dist(prev[:3], t[:3])
        relative = cast(
            "Rotation", Rotation.from_rotvec(t[3:6]) * Rotation.from_rotvec(prev[3:6]).inv()
        )
        step_deg = math.degrees(relative.magnitude())
        x, y, z = t[:3]
        checks = [
            (step_mm > max_step_mm, f"step {step_mm:.1f} > {max_step_mm:.1f} mm"),
            (step_deg > g.max_rotation_step_deg, f"rotation step {step_deg:.1f} deg"),
            (math.dist(start[:3], t[:3]) > g.max_chunk_travel_mm, "chunk travel"),
            (not g.workspace_x_mm[0] <= x <= g.workspace_x_mm[1], f"x {x:.0f} outside workspace"),
            (not g.workspace_y_mm[0] <= y <= g.workspace_y_mm[1], f"y {y:.0f} outside workspace"),
            (z < g.min_tcp_z_mm, f"z {z:.0f} below {g.min_tcp_z_mm} mm"),
        ]
        broken += [f"step {i}: {text}" for bad, text in checks if bad]
        prev = t
    return broken


def check_guards(
    cfg: PolicyConfig,
    start: list[float],
    targets: Targets,
    dt_ms: float,
) -> None:
    """Raise :class:`GuardError` if a chunk breaks a declared guard."""
    if broken := guard_violations(cfg, start, targets, dt_ms):
        raise GuardError("; ".join(broken))


def gripper_state(cfg: PolicyConfig, jaw_gap_mm: float, *, holding: bool) -> float:
    """The policy's gripper state for a real jaw gap: the declared holding value while
    holding, else the gap mapped linearly between the declared closed and open values."""
    g = cfg.policy.gripper
    if holding:
        return g.state_holding
    opening = min(max(jaw_gap_mm / cfg.cell.gripper_open_gap_mm, 0.0), 1.0)
    return g.state_closed + opening * (g.state_open - g.state_closed)


def next_gripper_switch(
    cfg: PolicyConfig, actions: list[float], steps: int, *, closed: bool | None
) -> tuple[int, bool] | None:
    """First of the next ``steps`` gripper actions that asks for a state other than
    ``closed``: ``(step, close)``, or None."""
    for index, value in enumerate(actions[:steps]):
        close = cfg.policy.gripper.closes(value)
        if close != closed:
            return index, close
    return None


async def lift_if_low(
    mg: MotionGroup, cfg: PolicyConfig, *, safe_z_mm: float = 200.0, speed_mm_s: float = 100.0
) -> None:
    """Below ``safe_z_mm``, move the declared TCP straight up to it, so a following joint
    move cannot sweep the tool over the table."""
    tcp = cfg.cell.tcp
    pose = (await mg.get_state(tcp)).pose
    if pose.position.z >= safe_z_mm:
        return
    up = Pose((pose.position.x, pose.position.y, safe_z_mm, *pose.orientation))
    lift = [linear(up, settings=MotionSettings(tcp_velocity_limit=speed_mm_s))]
    await mg.execute(await mg.plan(lift, tcp), tcp, actions=lift)


async def move_home(
    mg: MotionGroup, cfg: PolicyConfig, *, safe_z_mm: float = 200.0, speed_mm_s: float = 100.0
) -> None:
    """:func:`lift_if_low`, then the joint move to the declared home."""
    await lift_if_low(mg, cfg, safe_z_mm=safe_z_mm, speed_mm_s=speed_mm_s)
    settings = MotionSettings(tcp_velocity_limit=speed_mm_s)
    home = [joint_ptp(tuple(math.radians(d) for d in cfg.cell.home_deg), settings=settings)]
    await mg.execute(await mg.plan(home, cfg.cell.tcp), cfg.cell.tcp, actions=home)


# --- Continuous execution ----------------------------------------------------------------------


class PlanQueue:
    """Absolute TCP plan keyed by policy timestep on the executor's timeline.

    For continuous execution: the executor places a chunk with ``action_timestep >= 0`` by
    its absolute timestep and reports the first replaceable one before every request. Each
    new prediction keeps the head that executed during inference (plus a margin) exactly as
    published, blends the overlap with the previous plan (``old_weight``) and smooths it with
    that head retained, as the LeRobot async queue does, so replacements meet without jumps.
    """

    def __init__(
        self,
        dt_ms: float,
        *,
        margin_steps: int = 2,
        old_weight: float = 0.3,
        smoothing_passes: int = 2,
        hold_steps: int = 5,
    ) -> None:
        self.dt_ms = dt_ms
        self.margin_steps = margin_steps
        self.old_weight = old_weight
        self.smoothing_passes = smoothing_passes
        self.hold_steps = hold_steps
        self.replaceable = 0
        self.published: dict[int, list[float]] = {}
        self.frozen_steps = margin_steps

    def reset(self) -> None:
        self.replaceable = 0
        self.published = {}
        self.frozen_steps = self.margin_steps

    def _head(self) -> dict[int, list[float]]:
        start, stop = self.replaceable - 1, self.replaceable + self.frozen_steps
        return {ts: v for ts, v in self.published.items() if start <= ts < stop}

    def _publish(self, plan: dict[int, list[float]], retained: int) -> tuple[int, Targets]:
        keys = sorted(plan)
        if keys != list(range(keys[0], keys[0] + len(keys))):
            msg = f"TCP plan has a gap in its timesteps: {keys}"
            raise RuntimeError(msg)
        chunk = ActionChunk(tcp={"_": [plan[ts] for ts in keys]}, dt_ms=self.dt_ms)
        smoothed = smooth_action_chunk(
            chunk, passes=self.smoothing_passes, retained_prefix_steps=retained
        ).tcp["_"]
        self.published = dict(zip(keys, smoothed, strict=True))
        return keys[0], smoothed

    def merge(self, targets: Targets, latency_ms: float) -> tuple[int, Targets]:
        """Merge targets for timesteps ``replaceable, replaceable + 1, ...``; returns the
        first timestep and the published plan from there."""
        self.frozen_steps = math.ceil(latency_ms / self.dt_ms) + self.margin_steps
        plan = self._head()
        retained = len(plan)
        if plan:  # a fresh reading near a half-turn may flip the rotation vector's sign
            targets = unwrap_tcp_rotations(targets, plan[min(plan)][3:6])
        w = self.old_weight
        for index, value in enumerate(targets):
            ts = self.replaceable + index
            if ts in plan:
                continue  # frozen: already executing or about to
            old = self.published.get(ts)
            blended = (
                [w * o + (1.0 - w) * n for o, n in zip(old, value, strict=True)] if old else value
            )
            plan[ts] = blended
        return self._publish(plan, retained)

    def hold(self) -> tuple[int, Targets] | None:
        """Keep the frozen head, then stand still at its last target."""
        plan = self._head()
        if not plan:
            return None
        last = max(plan)
        for offset in range(1, self.hold_steps + 1):
            plan[last + offset] = plan[last]
        return self._publish(plan, len(plan))


# --- Client ------------------------------------------------------------------------------------


class DeclaredGr00tClient(Gr00tPolicyClient):
    """GR00T client that runs the policy as ``policy.toml`` declares it.

    Args:
        cfg: The loaded config.
        continuous: Continuous execution (chunks merged on a :class:`PlanQueue`);
            sequential otherwise.
        speed_factor: Step spacing is the declared rate's divided by this.
        n_action_steps: Sequential: steps of each chunk that run (see ``steps_to_run``).
        near_z_mm, near_steps: Sequential: below this TCP height run only ``near_steps``
            per chunk, so the policy re-observes (and corrects) more often close to the
            table. Off by default.
        hold_while: Called before each request; while it returns True (e.g. a gripper
            still closing) the arm holds still and the policy is not asked.
        max_chunks: Sequential: stop after this many chunks (None: no limit).

    After each request, ``gripper_actions`` holds the declared gripper action's values and
    ``executed_steps`` how many steps of the chunk run before the next one.
    """

    def __init__(
        self,
        cfg: PolicyConfig,
        *,
        continuous: bool = False,
        speed_factor: float = 1.0,
        n_action_steps: int = 8,
        near_z_mm: float | None = None,
        near_steps: int | None = None,
        hold_while: Callable[[], bool] | None = None,
        max_chunks: int | None = None,
    ) -> None:
        self.cfg = cfg
        self.dt_ms = cfg.policy.dt_ms / speed_factor
        super().__init__(host=cfg.server.host, port=cfg.server.port, dt_ms=self.dt_ms)
        self.continuous = continuous
        self._n_action_steps = n_action_steps
        self.near_z_mm = near_z_mm
        self.near_steps = near_steps
        self.hold_while = hold_while
        self.max_chunks = max_chunks
        self.queue = PlanQueue(self.dt_ms)
        self.chunks = 0
        self.gripper_actions: list[float] = []
        self.executed_steps = n_action_steps

    def steps_to_run(self, start: list[float]) -> int:
        """Sequential: steps of a chunk starting at ``start`` that run."""
        if self.near_z_mm is not None and self.near_steps and start[2] < self.near_z_mm:
            return self.near_steps
        return self._n_action_steps

    @property
    def n_action_steps(self) -> int | None:
        """Sequential: the executor runs at most this many steps of each chunk."""
        return None if self.continuous else self._n_action_steps

    async def connect(self, motion_group_ids: list[str]) -> None:
        self.queue.reset()
        self.chunks = 0
        await super().connect(motion_group_ids)

    @property
    def requires_first_waypoint_bridge(self) -> bool:
        return self.continuous  # chunks carry absolute timesteps; only the first is bridged

    def synchronize_action_timestep(self, timestep: int) -> None:
        self.queue.replaceable = timestep

    async def validate_schema(self, schema: PolicySchema) -> None:
        """The server must serve the declared policy, and the schema must feed it.

        Unlike the stock check, computed observations count as state, and the prompt comes
        from the config rather than an ``Observation.constant``.
        """
        info = await self.get_server_info()
        p = self.cfg.policy
        served = {
            "state_keys": sorted(cast("list[str]", info["state_keys"])),
            "action_keys": sorted(cast("list[str]", info["action_keys"])),
            "video_keys": sorted(cast("list[str]", info["video_keys"])),
            "language_keys": sorted(cast("list[str]", info["language_keys"])),
            "action_horizon": info["action_horizon"],
        }
        declared: dict[str, object] = {
            "state_keys": sorted(p.state_keys),
            "action_keys": sorted(p.action_keys),
            "video_keys": sorted(p.video_keys),
            "language_keys": [p.language_key],
            "action_horizon": p.action_horizon,
        }
        if mismatch := {k: v for k, v in served.items() if v != declared[k]}:
            details = "; ".join(
                f"{k}: declared {declared[k]}, served {v}" for k, v in mismatch.items()
            )
            msg = f"The GR00T server does not serve the declared policy: {details}"
            raise ValueError(msg)
        fed = {m.key for m in schema.joint_mappings} | {m.key for m in schema.tcp_mappings}
        fed |= {p.gripper.state_key} if schema.computed_observations else set()
        missing = (set(p.state_keys) - fed) | (set(p.video_keys) - set(schema.image_sources))
        if missing:
            msg = f"The schema does not feed the declared inputs {sorted(missing)}"
            raise ValueError(msg)

    async def _build_groot_obs(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, Any] | None,
        io_values: dict[str, object] | None,
    ) -> dict[str, Any]:
        obs = await super()._build_groot_obs(states, schema, images, io_values)
        p = self.cfg.policy
        obs["state"] = {k: v for k, v in obs["state"].items() if k in p.state_keys}
        obs["language"] = {p.language_key: [[p.prompt]]}
        return obs

    def _decode_action(
        self, schema: PolicySchema, action: dict[str, object], info: dict[str, object]
    ) -> ActionChunk:
        """Also keep the gripper action for an ``Action.computed`` hook (``gripper_actions``)."""
        chunk = super()._decode_action(schema, action, info)
        key = self.cfg.policy.gripper.action_key
        if key in action:
            self.gripper_actions = np.asarray(action[key], dtype=float).reshape(-1).tolist()
        return chunk

    async def get_actions(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, Any] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        if self.hold_while is not None and self.hold_while():
            return await self._hold(states, schema)
        if not self.continuous and self.max_chunks is not None and self.chunks >= self.max_chunks:
            msg = f"chunk limit reached ({self.max_chunks})"
            raise ChunkLimitError(msg)
        self.chunks += 1
        p = self.cfg.policy
        frames = frame_images(images, p.image_width, p.image_height) if images else images
        t0 = time.monotonic()
        chunk = await super().get_actions(states, schema, frames, io_values)
        latency_ms = (time.monotonic() - t0) * 1000.0
        tcp: dict[str, Targets] = {}
        for mg_id, deltas in chunk.tcp.items():
            start = self._start(states[mg_id])
            targets = tcp_targets(self.cfg, start, deltas)
            if self.continuous:
                first, targets = self.queue.merge(targets, latency_ms)
                check_guards(self.cfg, start, targets, self.dt_ms)
                self.executed_steps = math.ceil(latency_ms / self.dt_ms) + 1
                return chunk.model_copy(
                    update={"tcp": {mg_id: targets}, "action_timestep": first, "dt_ms": self.dt_ms}
                )
            targets = targets[: self.steps_to_run(start)]
            check_guards(self.cfg, start, targets, self.dt_ms)
            tcp[mg_id] = targets
            self.executed_steps = len(targets)
        # The declared step spacing, whatever the server says: guards, queue and gripper
        # timing all assume it.
        return chunk.model_copy(update={"tcp": tcp, "dt_ms": self.dt_ms})

    async def preview(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, Any] | None = None,
    ) -> dict[str, Preview]:
        """One inference, converted and checked like :meth:`get_actions` but neither queued
        nor raising on a guard: a :class:`Preview` per motion group id. For dry runs."""
        p = self.cfg.policy
        frames = frame_images(images, p.image_width, p.image_height) if images else images
        t0 = time.monotonic()
        chunk = await super().get_actions(states, schema, frames)
        inference_ms = (time.monotonic() - t0) * 1000.0
        out: dict[str, Preview] = {}
        for mg_id, deltas in chunk.tcp.items():
            start = self._start(states[mg_id])
            targets = tcp_targets(self.cfg, start, deltas)
            steps = len(targets) if self.continuous else self.steps_to_run(start)
            violations = guard_violations(self.cfg, start, targets[:steps], self.dt_ms)
            out[mg_id] = Preview(
                deltas, targets, steps, violations, list(self.gripper_actions), inference_ms
            )
        return out

    def _start(self, state: RobotState) -> list[float]:
        if state.tcp != self.cfg.cell.tcp:
            msg = f"state pose is for TCP {state.tcp!r}, policy.toml declares {self.cfg.cell.tcp!r}"
            raise RuntimeError(msg)
        return [*state.pose.position, *state.pose.orientation]

    async def _hold(self, states: dict[str, RobotState], schema: PolicySchema) -> ActionChunk:
        await asyncio.sleep(self.dt_ms / 1000.0)  # no inference; don't spin the executor loop
        self.gripper_actions = []  # nothing new for a gripper hook to act on
        held = self.queue.hold() if self.continuous else None
        if held is not None:
            first, targets = held
            tcp = {mg.id: targets for _key, mg in schema.tcp_action_keys}
            return ActionChunk(tcp=tcp, dt_ms=self.dt_ms, action_timestep=first)
        tcp = {}
        for _key, mg in schema.tcp_action_keys:
            pose = states[mg.id].pose
            tcp[mg.id] = [[*pose.position, *pose.orientation]] * self.queue.hold_steps
        return ActionChunk(tcp=tcp, dt_ms=self.dt_ms)
