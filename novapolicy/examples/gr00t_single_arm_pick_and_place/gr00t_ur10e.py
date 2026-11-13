"""Example: Run a GR00T policy on the real single-arm UR10e ``ur10e-calibration``.

The policy server on this cell expects (queried from ``get_modality_config``):

    state:    arm_joints (6), gripper (1)
    video:    wrist, side, top
    language: annotation.human.task_description
    action:   eef_delta (16 x 6), gripper (16 x 1)

Things the stock ``Gr00tPolicyClient`` does not do for this model are handled
by :class:`SingleArmGr00tClient` below:

* it sends the language under ``annotation.human.task_description`` (the stock
  client hardcodes ``annotation.language.language_instruction``, which this
  server rejects),
* it turns ``eef_delta`` into absolute TCP targets with proper rotation
  composition. NOVA's built-in ``mode="relative"`` adds rotation vectors
  component-wise, which is wrong for a tool-down pose (rotation ~ pi), and
* it hands the ``gripper`` action to :class:`ZimmerGripper`, not to a NOVA IO.

The gripper is a Zimmer GEH6060IL on a Turck IO-Link master, driven by the
``zimmer-gripper-controller`` package over Modbus TCP. It enters the policy
through ``Observation.computed`` (state) and ``Action.computed`` (command).

Shared implementation of ``run_sequential.py`` and ``run_continuous.py``; see README.md
next to this file for how to start them.

Sequential (the default): one chunk at a time, run to standstill. Continuous: the
policy is re-queried as fast as the server answers, and :class:`TcpPlanQueue` keeps
each chunk on the executor's timestep timeline (frozen head, old/new blending,
smoothing), like the LeRobot async queue. The tool orientation is held fixed;
position follows the policy.

Without ``--execute`` a script does a DRY RUN: one real observation, one inference,
the targets printed. Nothing is commanded, and the gripper is not connected
(connecting starts its homing, which moves the jaws). ``--execute`` moves the robot
and the gripper; it asks for confirmation first.
"""

from __future__ import annotations

import argparse
import asyncio
import importlib
import logging
import math
from pathlib import Path
import sys
import time
from typing import Any, cast

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from nova import ProgramContext, api, program, run_program, viewers
from nova.actions import joint_ptp, linear
from nova.cell.motion_group import MotionGroup
from nova.types import MotionSettings, Pose, RobotState
from novapolicy import (
    Action,
    ActionChunk,
    ContinuousExecution,
    Gr00tPolicyClient,
    Observation,
    PolicyExecutor,
    PolicySchema,
    SequentialExecution,
    WebRTCCameras,
)
from novapolicy.chunking import smooth_action_chunk, unwrap_tcp_rotations
from novapolicy.gr00t import policy_config

logger = logging.getLogger(__name__)

# --- Declared config (policy.toml): policy facts, the cell, the guards -----------------
CONFIG_PATH = Path(__file__).with_name("policy.toml")
CFG = policy_config.load(CONFIG_PATH)
P, CELL, GUARDS = CFG.policy, CFG.cell, CFG.guards

CONTROLLER = CELL.controller
TCP = CELL.tcp
HOME_DEG = tuple(CELL.home_deg)
HOME = tuple(math.radians(d) for d in HOME_DEG)
GROOT_HOST, GROOT_PORT = CFG.server.host, CFG.server.port
LANGUAGE_KEY = P.language_key
TASK = P.prompt
TRAINING_DT_MS = P.dt_ms
CAMERA_SERVER = CELL.camera_server
IMAGE_SIZE = (P.image_width, P.image_height)
GRIPPER_HOST = CELL.gripper_host

# --- Execution (not part of the policy) -------------------------------------------------
HOME_SETTINGS = MotionSettings(tcp_velocity_limit=100.0)
# Below this TCP height, first move straight up to it before the joint move home, so the
# joint interpolation cannot sweep the gripper over the cube or the table.
SAFE_Z_MM = 200.0
# Playback speed relative to the training rate: 1.0 plays one policy step per
# 1 / rate_hz; below 1.0 the same path runs slower.
SPEED_FACTOR = 1.0
DT_MS = TRAINING_DT_MS / SPEED_FACTOR
N_ACTION_STEPS = 8
# Sequential: near the table, run half chunks so the policy re-observes (and corrects its
# placement over the cube) twice as often during the descent.
NEAR_TABLE_Z_MM = 120.0
NEAR_TABLE_STEPS = N_ACTION_STEPS // 2
TIMEOUT_S = 300.0
# Execution mode, chosen by the entry script. Sequential (default, run_sequential.py): run
# N_ACTION_STEPS per chunk to standstill, observe, infer again. Continuous
# (run_continuous.py): re-infer as fast as the server answers; TcpPlanQueue merges each new
# 16-step chunk on the timestep timeline and smooths it with the [1, 2, 1] / 4 filter
# (two passes) the LeRobot client uses.
SMOOTHING_PASSES = 2
HOLD_STEPS = 5  # waypoints of a hold-in-place chunk while the gripper is busy
# Continuous seams, as the LeRobot async queue does it: the steps that executed while
# this inference was in flight (its measured latency) plus a margin stay exactly as
# published; later overlapping steps are blended old/new 0.3/0.7.
QUEUE_MARGIN_STEPS = 2
QUEUE_OLD_WEIGHT = 0.3
MAX_CHUNKS = 120  # sequential: a cap on chunks per run

# --- Zimmer gripper hardware ------------------------------------------------------------
# Homing opens the jaws to an 80.00 mm gap. In force mode the grip force also sets the
# closing speed (open -> 50 mm gap: 20% 3.7 s, 50% 1.8 s, 100% 1.5 s). 50% for the PLA cube.
GRIPPER_FORCE_PERCENT = 50
GRIPPER_VELOCITY_PERCENT = 100  # positioning moves (open_and_wait) only
GRIPPER_OPEN_GAP_M = 0.080

# --- Guards (declared in policy.toml) ----------------------------------------------------
MAX_TCP_SPEED_MM_S = GUARDS.max_tcp_speed_mm_s
MAX_STEP_MM = MAX_TCP_SPEED_MM_S * DT_MS / 1000.0  # the speed limit as a distance per step
MAX_CHUNK_TRAVEL_MM = GUARDS.max_chunk_travel_mm
MAX_STEP_DEG = GUARDS.max_rotation_step_deg
MAX_TILT_DEG = GUARDS.max_tilt_deg
MAX_YAW_DEG = GUARDS.max_yaw_deg
MIN_TCP_Z_MM = GUARDS.min_tcp_z_mm
WORKSPACE_X_MM = tuple(GUARDS.workspace_x_mm)
WORKSPACE_Y_MM = tuple(GUARDS.workspace_y_mm)


def open_gripper_session(host: str = GRIPPER_HOST) -> Any:
    """Connect to the Zimmer gripper, home it and open it (moves the jaws). Blocking."""
    # Imported here so dry runs work without the (optional) gripper package installed.
    zimmer = importlib.import_module("zimmer_gripper_controller")
    session = zimmer.GripperSession(zimmer.SessionConfig(modbus=zimmer.ModbusConfig(host=host)))
    session.connect()
    session.wait_until_ready()
    session.set_force_percent(GRIPPER_FORCE_PERCENT)
    session.set_velocity_percent(GRIPPER_VELOCITY_PERCENT)
    return session


async def lift_if_low(mg: MotionGroup, settings: MotionSettings = HOME_SETTINGS) -> None:
    """Below SAFE_Z_MM, move straight up to it, so a following joint move cannot sweep
    the gripper over the cube or the table."""
    pose = (await mg.get_state(TCP)).pose
    if pose.position.z >= SAFE_Z_MM:
        return
    print(f"Lifting straight up from z {pose.position.z:.0f} to {SAFE_Z_MM:.0f} mm...")
    up = Pose((pose.position.x, pose.position.y, SAFE_Z_MM, *pose.orientation))
    actions = [linear(up, settings=settings)]
    await mg.execute(await mg.plan(actions, TCP), TCP, actions=actions)


class ZimmerGripper:
    """Feeds the Zimmer jaw gap to the policy and executes its ``gripper`` action.

    ``GripperSession`` is blocking and thread based, so commands run in a worker
    thread and never stall the executor loop. In binary mode a command is only
    sent when the open/closed decision changes; ``grip()`` blocks until the jaws
    hold or reach the end, and a newer decision waits for it.
    """

    def __init__(self, host: str) -> None:
        self._host = host
        self._session: Any = None  # created in connect(); dry runs never connect
        self._task: asyncio.Task[None] | None = None
        self._task_started = False  # the scheduled grip()/open() has left its delay
        self._closed: bool | None = None
        self._holding = False
        self.last_action: list[float] = []  # raw 16-step `gripper` action of the last chunk
        self.executed_steps = N_ACTION_STEPS  # steps of the last chunk that actually run

    async def connect(self) -> None:
        """Start the controller, home, and open (moves the jaws)."""
        self._session = await asyncio.to_thread(open_gripper_session, self._host)
        self._closed = False

    async def close(self) -> None:
        """Let a running grip()/open() finish; drop one still waiting for its step."""
        if self._task is not None and not self._task.done():
            if not self._task_started:
                self._task.cancel()  # the run ended before the command's step came
            await asyncio.gather(self._task, return_exceptions=True)
        if self._session is not None:
            self._session.close()

    @property
    def busy(self) -> bool:
        """A grip()/open_and_wait() is still running."""
        return self._task is not None and not self._task.done()

    async def wait_idle(self) -> None:
        """Wait for a running grip()/open_and_wait() to finish."""
        if self._task is not None and not self._task.done():
            await asyncio.gather(self._task, return_exceptions=True)

    async def observe(self, _obs: dict[str, Any]) -> dict[str, float]:
        """``Observation.computed`` hook: the jaw gap as the policy's ``gripper`` state."""
        state = self._session.state()
        if state.has_error:
            msg = f"Gripper error: diag=0x{state.diagnosis:04X} {state.last_error_text}"
            raise RuntimeError(msg)
        g = P.gripper
        if self._closed and self._holding:
            return {g.state_key: g.state_holding}
        # The jaw gap mapped linearly between the declared open and closed state values.
        opening = min(max(state.jaw_gap_m / GRIPPER_OPEN_GAP_M, 0.0), 1.0)
        return {g.state_key: g.state_closed + opening * (g.state_open - g.state_closed)}

    def next_switch(self, *, closed: bool | None = None) -> tuple[int, bool] | None:
        """First executed step whose open/closed decision differs from ``closed``.

        ``closed`` defaults to the last commanded state.
        """
        current = self._closed if closed is None else closed
        for index, value in enumerate(self.last_action[: self.executed_steps]):
            close = P.gripper.closes(value)
            if close != current:
                return index, close
        return None

    async def act(self, _chunk: ActionChunk) -> None:
        """``Action.computed`` hook, fired as the arm reaches the chunk's first step.

        Binary mode: switch at the first executed step that asks for it, delayed by
        that step's offset, so a close at step 5 is not held back to the next chunk.
        """
        if not self.last_action:
            return
        switch = self.next_switch()
        if switch is None:
            return
        if self._task is not None and not self._task.done():
            return  # previous grip/open still running; re-decide on the next chunk
        index, close = switch
        self._closed = close
        self._task_started = False
        self._task = asyncio.create_task(self._command(close=close, delay_s=index * DT_MS / 1000))

    async def _command(self, *, close: bool, delay_s: float = 0.0) -> None:
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


def to_training_view(frames: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    """Centre-crop each frame to the sim cameras' 4:3 and resize to ``IMAGE_SIZE``."""
    out: dict[str, np.ndarray] = {}
    width, height = IMAGE_SIZE
    for key, frame in frames.items():
        h, w = frame.shape[:2]
        crop_w = min(w, round(h * width / height))
        crop_h = min(h, round(w * height / width))
        x0, y0 = (w - crop_w) // 2, (h - crop_h) // 2
        image = Image.fromarray(frame[y0 : y0 + crop_h, x0 : x0 + crop_w])
        out[key] = np.asarray(image.resize((width, height), Image.Resampling.LANCZOS))
    return out


def deltas_to_absolute(
    start: list[float],
    deltas: list[list[float]],
    pos_gain: float = P.tcp_action.gain_position,
    rot_gain: float = P.tcp_action.gain_rotation,
) -> list[list[float]]:
    """Compose per-step TCP deltas onto ``start`` ([x, y, z, rx, ry, rz], mm / rotvec), as
    declared in ``policy.tcp_action``: each step applied to the pose the previous one
    reached, rotations pre-multiplied about the TCP."""
    action = P.tcp_action
    p = np.asarray(start[:3], dtype=float)
    r: Rotation = Rotation.from_rotvec(start[3:6])
    out: list[list[float]] = []
    to_nova = Rotation.from_euler("z", action.base_rotation_z_deg, degrees=True)
    for step in deltas:
        dp = np.asarray(step[:3], dtype=float) * action.position_scale_mm * pos_gain
        rv = np.asarray(step[3:6], dtype=float) * rot_gain
        # Position and rotation delta in NOVA's base frame.
        if action.frame == "tool":
            dp, rv = r.apply(dp), r.apply(rv)
        else:
            dp, rv = to_nova.apply(dp), to_nova.apply(rv)
        p += dp
        # Rotate about the TCP (pre-multiplied, base frame); a tool-frame delta expressed in
        # base axes gives the same result as post-multiplying in tool axes.
        r = cast("Rotation", Rotation.from_rotvec(rv) * r)
        out.append([*p.tolist(), *r.as_rotvec().tolist()])
    # Near pi (tool down) scipy flips the rotation vector's sign; keep it continuous
    # with the start pose, as the executor's bridges and ramps interpolate component-wise.
    return unwrap_tcp_rotations(out, start[3:6])


def executed_steps(start: list[float]) -> int:
    """Sequential: steps to run from a chunk that starts at ``start``, half near the table."""
    return NEAR_TABLE_STEPS if start[2] < NEAR_TABLE_Z_MM else N_ACTION_STEPS


def tilt_deg(pose: list[float]) -> float:
    """Angle between the tool z-axis and straight down (base -z), in degrees."""
    tool_z = Rotation.from_rotvec(pose[3:6]).apply([0.0, 0.0, 1.0])
    return math.degrees(math.acos(max(-1.0, min(1.0, -tool_z[2]))))


def yaw_deg(pose: list[float]) -> float:
    """Heading of the tool x-axis in the base xy-plane, in degrees."""
    tool_x = Rotation.from_rotvec(pose[3:6]).apply([1.0, 0.0, 0.0])
    return math.degrees(math.atan2(tool_x[1], tool_x[0]))


def yaw_offset_deg(pose: list[float], reference_yaw: float) -> float:
    """Signed yaw of ``pose`` relative to ``reference_yaw``, in (-180, 180]."""
    return (yaw_deg(pose) - reference_yaw + 180.0) % 360.0 - 180.0


def guard_violations(
    start: list[float], prev: list[float], target: list[float], reference_yaw: float
) -> list[str]:
    """The guards ``target`` breaks, coming from ``prev`` in a chunk that starts at ``start``."""
    step_mm = math.dist(prev[:3], target[:3])
    step_deg = math.degrees(
        cast(
            "Rotation", Rotation.from_rotvec(target[3:6]) * Rotation.from_rotvec(prev[3:6]).inv()
        ).magnitude()
    )
    travel_mm = math.dist(start[:3], target[:3])
    tilt, yaw = tilt_deg(target), yaw_offset_deg(target, reference_yaw)
    x, y, z = target[:3]
    checks = [
        (step_mm > MAX_STEP_MM, f"step {step_mm:.1f} > {MAX_STEP_MM:.1f} mm"),
        (step_deg > MAX_STEP_DEG, f"rotation step {step_deg:.1f} > {MAX_STEP_DEG} deg"),
        (
            travel_mm > MAX_CHUNK_TRAVEL_MM,
            f"chunk travel {travel_mm:.0f} > {MAX_CHUNK_TRAVEL_MM} mm",
        ),
        (tilt > MAX_TILT_DEG, f"tilt {tilt:.1f} > {MAX_TILT_DEG} deg"),
        (
            abs(yaw) > MAX_YAW_DEG,
            f"yaw {yaw:+.1f} beyond +-{MAX_YAW_DEG} deg",
        ),
        (not WORKSPACE_X_MM[0] <= x <= WORKSPACE_X_MM[1], f"x {x:.0f} outside {WORKSPACE_X_MM}"),
        (not WORKSPACE_Y_MM[0] <= y <= WORKSPACE_Y_MM[1], f"y {y:.0f} outside {WORKSPACE_Y_MM}"),
        (z < MIN_TCP_Z_MM, f"z {z:.0f} below {MIN_TCP_Z_MM} mm"),
    ]
    return [text for broken, text in checks if broken]


def check_targets(
    start: list[float], targets: list[list[float]], reference_yaw: float | None = None
) -> None:
    """Raise if a chunk breaks a guard.

    ``reference_yaw`` is the run's start heading for ``MAX_YAW_DEG`` (default: ``start``).
    """
    ref = yaw_deg(start) if reference_yaw is None else reference_yaw
    prev = start
    for i, target in enumerate(targets):
        if broken := guard_violations(start, prev, target, ref):
            msg = f"eef_delta step {i} exceeds guards: {'; '.join(broken)}"
            raise RuntimeError(msg)
        prev = target


class TcpPlanQueue:
    """Absolute TCP plan keyed by policy timestep on the executor's NOVA timeline.

    The executor places a chunk with ``action_timestep >= 0`` by its absolute timestep
    and reports the first replaceable one before every request
    (``synchronize_action_timestep``). Each new prediction keeps the frozen head
    (what runs while the next inference is in flight) exactly as published, blends
    the overlap with the previous plan, and is smoothed with that head retained, so
    replacements meet without the jumps of a hard replace.
    """

    def __init__(self) -> None:
        self.replaceable = 0
        self.published: dict[int, list[float]] = {}
        self.frozen_steps = QUEUE_MARGIN_STEPS

    def reset(self) -> None:
        self.replaceable = 0
        self.published = {}
        self.frozen_steps = QUEUE_MARGIN_STEPS

    def _head(self) -> dict[int, list[float]]:
        start, stop = self.replaceable - 1, self.replaceable + self.frozen_steps
        return {ts: v for ts, v in self.published.items() if start <= ts < stop}

    def _publish(
        self, plan: dict[int, list[float]], retained: int
    ) -> tuple[int, list[list[float]]]:
        keys = sorted(plan)
        if keys != list(range(keys[0], keys[0] + len(keys))):
            msg = f"TCP plan has a gap in its timesteps: {keys}"
            raise RuntimeError(msg)
        steps = [plan[ts] for ts in keys]
        chunk = ActionChunk(tcp={"_": steps}, dt_ms=DT_MS)
        smoothed = smooth_action_chunk(
            chunk, passes=SMOOTHING_PASSES, retained_prefix_steps=retained
        ).tcp["_"]
        self.published = dict(zip(keys, smoothed, strict=True))
        return keys[0], smoothed

    def merge(self, targets: list[list[float]], latency_ms: float) -> tuple[int, list[list[float]]]:
        """Merge targets for timesteps ``replaceable, replaceable + 1, ...``.

        ``latency_ms`` is how long this inference took: the published steps that ran
        meanwhile (plus a margin) are kept exactly.
        """
        self.frozen_steps = math.ceil(latency_ms / DT_MS) + QUEUE_MARGIN_STEPS
        plan = self._head()
        retained = len(plan)
        # A fresh pose reading near a half-turn can come back with the rotation vector's
        # sign flipped; blending +v with -v component-wise is a different orientation.
        # Express the new targets on the published plan's side before blending.
        if plan:
            targets = unwrap_tcp_rotations(targets, plan[min(plan)][3:6])
        for index, value in enumerate(targets):
            ts = self.replaceable + index
            if ts in plan:
                continue  # frozen: already executing or about to
            old = self.published.get(ts)
            plan[ts] = (
                value
                if old is None
                else [
                    QUEUE_OLD_WEIGHT * o + (1.0 - QUEUE_OLD_WEIGHT) * n
                    for o, n in zip(old, value, strict=True)
                ]
            )
        return self._publish(plan, retained)

    def hold(self) -> tuple[int, list[list[float]]] | None:
        """Keep the frozen head, then stand still at its last target."""
        plan = self._head()
        if not plan:
            return None
        last = max(plan)
        for offset in range(1, HOLD_STEPS + 1):
            plan[last + offset] = plan[last]
        return self._publish(plan, len(plan))


class SingleArmGr00tClient(Gr00tPolicyClient):
    """GR00T client for the ``arm_joints``/``eef_delta`` model on this cell."""

    def __init__(
        self, *args: Any, gripper: ZimmerGripper, continuous: bool = False, **kwargs: Any
    ) -> None:
        super().__init__(*args, dt_ms=DT_MS, **kwargs)
        self._gripper = gripper
        self._continuous = continuous
        self.chunks = 0
        self._queue = TcpPlanQueue()
        self._reference_yaw: float | None = None  # the run's start heading, for MAX_YAW_DEG

    async def connect(self, motion_group_ids: list[str]) -> None:
        self._queue.reset()
        self._reference_yaw = None
        await super().connect(motion_group_ids)

    @property
    def requires_first_waypoint_bridge(self) -> bool:
        # Continuous: chunks carry absolute timesteps; only the first one is bridged.
        return self._continuous

    def synchronize_action_timestep(self, timestep: int) -> None:
        self._queue.replaceable = timestep

    async def validate_schema(self, schema: PolicySchema) -> None:
        """The server must serve the policy declared in policy.toml, and the schema feed it."""
        info = await self.get_server_info()
        served = {
            "state_keys": sorted(cast("list[str]", info["state_keys"])),
            "action_keys": sorted(cast("list[str]", info["action_keys"])),
            "video_keys": sorted(cast("list[str]", info["video_keys"])),
            "language_keys": sorted(cast("list[str]", info["language_keys"])),
            "action_horizon": info["action_horizon"],
        }
        declared = {
            "state_keys": sorted(P.state_keys),
            "action_keys": sorted(P.action_keys),
            "video_keys": sorted(P.video_keys),
            "language_keys": [P.language_key],
            "action_horizon": P.action_horizon,
        }
        if mismatch := {k: (declared[k], v) for k, v in served.items() if v != declared[k]}:
            details = "; ".join(f"{k}: declared {d}, served {v}" for k, (d, v) in mismatch.items())
            msg = f"The GR00T server does not serve the policy in {CONFIG_PATH.name}: {details}"
            raise ValueError(msg)
        provided = {m.key for m in schema.joint_mappings} | {P.gripper.state_key}
        missing = set(P.state_keys) - provided
        missing |= set(P.video_keys) - set(schema.image_sources)
        if missing:
            msg = f"Schema does not provide GR00T inputs: {sorted(missing)}"
            raise ValueError(msg)

    async def _build_groot_obs(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, Any] | None,
        io_values: dict[str, object] | None,
    ) -> dict[str, Any]:
        obs = await super()._build_groot_obs(states, schema, images, io_values)
        # The TCP observation only exists to pin the TCP; the model does not take it.
        obs["state"] = {k: v for k, v in obs["state"].items() if k in P.state_keys}
        obs["language"] = {LANGUAGE_KEY: [[TASK]]}
        return obs

    def _decode_action(
        self, schema: PolicySchema, action: dict[str, object], info: dict[str, object]
    ) -> ActionChunk:
        # The step spacing is DT_MS (training rate x SPEED_FACTOR), whatever the server's
        # info says: the guards, the plan queue and the gripper timing all assume it.
        chunk = super()._decode_action(schema, action, info).model_copy(update={"dt_ms": DT_MS})
        # Not a NOVA IO: the Action.computed hook reads it from the gripper bridge.
        values = np.asarray(action[P.gripper.action_key], dtype=float).reshape(-1)
        self._gripper.last_action = values.tolist()
        return chunk

    async def get_actions(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, Any] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        # While the jaws close or open, hold the arm where it is instead of letting it play
        # out a lookahead that may already lift. The policy is asked again once they settle.
        if self._gripper.busy:
            if self._continuous:
                held = await self._hold_queue(schema)
                if held is not None:
                    return held
            return await self._hold_chunk(states, schema)
        # Sequential execution: the previous chunk has finished, so the arm stands still here.
        if not self._continuous and self.chunks >= MAX_CHUNKS:
            msg = f"Chunk limit reached ({MAX_CHUNKS}); stopping at standstill"
            raise RuntimeError(msg)
        self.chunks += 1
        frames = to_training_view(images) if images else images
        t0 = time.monotonic()
        chunk = await super().get_actions(states, schema, frames, io_values)
        latency_ms = (time.monotonic() - t0) * 1000.0
        tcp: dict[str, list[list[float]]] = {}
        for mg_id, deltas in chunk.tcp.items():
            if states[mg_id].tcp != TCP:
                msg = f"State pose is for TCP {states[mg_id].tcp!r}, expected {TCP!r}"
                raise RuntimeError(msg)
            pose = states[mg_id].pose
            start = [*pose.position, *pose.orientation]
            if self._reference_yaw is None:
                self._reference_yaw = yaw_deg(start)
            if self._continuous:
                first, targets = self._queue.merge(deltas_to_absolute(start, deltas), latency_ms)
                check_targets(start, targets, self._reference_yaw)
                # Steps that run before the next chunk replaces this one.
                self._gripper.executed_steps = math.ceil(latency_ms / DT_MS) + 1
                return chunk.model_copy(update={"tcp": {mg_id: targets}, "action_timestep": first})
            targets = deltas_to_absolute(start, deltas)[: executed_steps(start)]
            check_targets(start, targets, self._reference_yaw)
            tcp[mg_id] = targets
            self._gripper.executed_steps = len(targets)
        return chunk.model_copy(update={"tcp": tcp})

    async def _hold_queue(self, schema: PolicySchema) -> ActionChunk | None:
        await asyncio.sleep(DT_MS / 1000.0)  # no inference here; don't spin the executor loop
        held = self._queue.hold()
        if held is None:
            return None
        self._gripper.last_action = []  # nothing new for the gripper hook to act on
        first, targets = held
        tcp = {mg.id: targets for _key, mg in schema.tcp_action_keys}
        return ActionChunk(tcp=tcp, dt_ms=DT_MS, action_timestep=first)

    async def _hold_chunk(self, states: dict[str, RobotState], schema: PolicySchema) -> ActionChunk:
        await asyncio.sleep(DT_MS / 1000.0)  # no inference here; don't spin the executor loop
        self._gripper.last_action = []  # nothing new for the gripper hook to act on
        tcp = {}
        for _key, mg in schema.tcp_action_keys:
            pose = states[mg.id].pose
            tcp[mg.id] = [[*pose.position, *pose.orientation]] * HOLD_STEPS
        return ActionChunk(tcp=tcp, dt_ms=DT_MS)


def make_program(*, continuous: bool, execute: bool) -> Any:
    """The NOVA program for one execution mode; dry run unless ``execute``."""

    @program(id="gr00t_ur10e_calibration", name="GR00T UR10e Calibration", viewer=viewers.Rerun())
    async def gr00t_ur10e_calibration(ctx: ProgramContext) -> None:
        mg = (await ctx.nova.cell().controller(CONTROLLER))[0]

        # Full frames; to_training_view() crops and resizes them before inference.
        cameras = WebRTCCameras(api_url=CAMERA_SERVER, frame_history=1)
        images = {key: cameras.device(serial) for key, serial in CELL.cameras.items()}
        gripper = ZimmerGripper(GRIPPER_HOST)

        async def dry_run_gripper_state(_obs: dict[str, Any]) -> dict[str, float]:
            return {P.gripper.state_key: P.gripper.state_open}

        schema = PolicySchema(
            observations=[
                Observation.joint_positions(P.joints.key, source=mg, action=False),
                # Not sent to the model; pins `umi_corrected` for state + Cartesian jogging.
                Observation.tcp("tcp_pose", source=mg, tcp=TCP),
                Observation.computed(gripper.observe if execute else dry_run_gripper_state),
                *(Observation.image(key, source=src) for key, src in images.items()),
                Observation.constant("language", value=TASK),
            ],
            actions=[
                Action.tcp(P.tcp_action.key, target=mg, mode="absolute"),
                Action.computed(gripper.act),
            ],
        )

        client = SingleArmGr00tClient(
            host=GROOT_HOST, port=GROOT_PORT, gripper=gripper, continuous=continuous
        )
        await client.connect([mg.id])
        try:
            await client.validate_schema(schema)  # the executor checks again before running
            print(f"GR00T server serves the policy declared in {CONFIG_PATH.name}")
            if not execute:
                await dry_run(mg, client, schema, images, gripper)
                return
            answer = await asyncio.to_thread(
                input,
                f"This homes and opens the gripper, moves {CONTROLLER} to home {HOME_DEG}, "
                "and starts the policy. Type 'yes': ",
            )
            if answer.strip().lower() != "yes":
                print("Aborted, nothing moved.")
                return
            await run_policy(mg, client, schema, gripper, continuous=continuous)
        finally:
            await client.close()  # the executor closes it too; closing twice is safe

    return gr00t_ur10e_calibration


async def run_policy(
    mg: MotionGroup,
    client: SingleArmGr00tClient,
    schema: PolicySchema,
    gripper: ZimmerGripper,
    *,
    continuous: bool,
) -> None:
    """Gripper home + open, lift if low, joint move home, then run the policy."""
    try:
        print("Connecting gripper (homing + open)...")
        await gripper.connect()
        await lift_if_low(mg)
        print("Moving to home...")
        actions = [joint_ptp(HOME, settings=HOME_SETTINGS)]
        await mg.execute(await mg.plan(actions, TCP), TCP, actions=actions)

        executor = PolicyExecutor(
            schema,
            client,
            timeout_s=TIMEOUT_S,
            execution=ContinuousExecution() if continuous else SequentialExecution(),
            # Continuous: keep the whole 16-step horizon as lookahead; each new chunk
            # replaces its tail. Sequential: the client already trimmed each chunk.
            n_action_steps=0 if continuous else N_ACTION_STEPS,
        )
        mode = "continuous" if continuous else "sequential"
        print(f"Running GR00T policy ({mode}) for up to {TIMEOUT_S}s...")
        t0 = time.monotonic()
        try:
            result = await executor.run()
        except Exception:
            logger.exception("Policy run failed after %.1fs", time.monotonic() - t0)
            raise
        dt = time.monotonic() - t0
        print(f"\nDone: reason={result.reason} steps={result.steps} duration={dt:.1f}s")
    finally:
        await gripper.close()


async def dry_run(mg, client, schema, images, gripper) -> None:
    """One real observation + inference. Prints targets, commands nothing."""
    for cam in images.values():
        await cam.connect()
    try:
        frames = to_training_view({key: cam.read() for key, cam in images.items()})
        state = await mg.get_state(TCP)
        print(f"TCP: {state.tcp}  joints(deg): {[round(math.degrees(j), 1) for j in state.joints]}")
        print(f"Gripper state sent (not connected in dry run): {P.gripper.state_open}")
        print(f"Frames: { {k: v.shape for k, v in frames.items()} }")

        t0 = time.monotonic()
        # Raw chunk (skip this subclass's conversion) so the deltas are visible.
        raw = await Gr00tPolicyClient.get_actions(client, {mg.id: state}, schema, frames)
        print(f"Inference: {(time.monotonic() - t0) * 1000:.0f} ms, dt_ms={raw.dt_ms}")
        start = [*state.pose.position, *state.pose.orientation]
        a = P.tcp_action
        print(f"Gains: position {a.gain_position}, rotation {a.gain_rotation}")
        targets = deltas_to_absolute(start, raw.tcp[mg.id])
        print(f"start pose (mm, rotvec): {np.round(start, 3).tolist()}")
        print(" step  raw eef_delta                              -> target pos (mm)  dpos_mm")
        for i, (d, t) in enumerate(zip(raw.tcp[mg.id], targets, strict=True)):
            mark = "*" if i < executed_steps(start) else " "
            dpos = np.subtract(t[:3], start[:3])
            print(
                f"{mark}{i:2d}  {np.round(d, 4).tolist()}  -> "
                f"{np.round(t[:3], 1).tolist()}  {np.round(dpos, 1).tolist()}"
            )
        switch = gripper.next_switch(closed=False)  # dry run: jaws assumed open
        command = (
            "none (stays open)"
            if switch is None
            else f"{'grip()' if switch[1] else 'open()'} at step {switch[0]}"
        )
        print(f"Gripper raw (16 steps): {np.round(gripper.last_action, 3).tolist()}")
        print(f"Gripper command that would be sent: {command}")
        try:
            check_targets(start, targets[: executed_steps(start)])  # only these execute
            print("Guards: OK")
        except RuntimeError as e:
            print(f"Guards: REJECTED — {e}")
        print("(* = executed with n_action_steps; dry run, nothing commanded)")
    finally:
        for cam in images.values():
            await cam.disconnect()


def cli(*, continuous: bool = False) -> None:
    """Entry point of ``run_sequential.py`` / ``run_continuous.py``; exits 1 if the run fails."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="move robot + gripper (asks first)")
    execute = parser.parse_args().execute
    runner = run_program(make_program(continuous=continuous, execute=execute))
    if runner.program_run.state == api.models.ProgramRunState.FAILED:
        sys.exit(1)
