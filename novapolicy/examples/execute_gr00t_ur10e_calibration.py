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

The default is a DRY RUN: one real observation, one inference, print the
targets. Nothing is commanded, and the gripper is not connected (connecting
starts its homing, which moves the jaws). Pass ``--execute`` to move the robot
and the gripper; it asks for confirmation first.

Run (dry run, no motion):
    NOVA_API=http://wandelbox-hhmnwy PYTHONPATH=. uv run --extra novapolicy-gr00t \
        --extra nova-rerun-bridge --with python-statemachine \
        --with ~/Git/zimmer-gripper-controller \
        python novapolicy/examples/execute_gr00t_ur10e_calibration.py

Run (moves the robot and the gripper):
    ... python novapolicy/examples/execute_gr00t_ur10e_calibration.py --execute
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import math
import time
from typing import Any, cast

import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation
from zimmer_gripper_controller import GripperSession, ModbusConfig, SessionConfig

from nova import ProgramContext, program, run_program, viewers
from nova.actions import joint_ptp
from nova.types import MotionSettings, RobotState
from novapolicy import (
    Action,
    ActionChunk,
    Gr00tPolicyClient,
    Observation,
    PolicyExecutor,
    PolicySchema,
    SequentialExecution,
    WebRTCCameras,
)

logger = logging.getLogger(__name__)

# --- Cell ---------------------------------------------------------------------
CONTROLLER = "ur10e-calibration"
# Midpoint between the finger pads, matching the sim IK point (GRASP_OFFSET).
TCP = "umi_corrected"
HOME_DEG = (90.0, -90.0, 90.0, -90.0, -90.0, 0.0)
HOME = tuple(math.radians(d) for d in HOME_DEG)
HOME_SETTINGS = MotionSettings(tcp_velocity_limit=100.0)

# --- GR00T server -------------------------------------------------------------
GROOT_HOST = "172.31.11.129"
GROOT_PORT = 30555
LANGUAGE_KEY = "annotation.human.task_description"
TASK = "pick up the cube and place it on the yellow square"
# Training env controls at 30 Hz (sim dt 1/120, decimation 4); one eef_delta per step.
TRAINING_DT_MS = 1000.0 / 30.0
# First supervised run: same path, played back 3x slower than training.
DT_MS = 100.0
N_ACTION_STEPS = 8
# Near the table, execute fewer steps per chunk so the policy re-observes (and corrects
# its xy placement over the cube) twice as often during the final descent.
NEAR_TABLE_Z_MM = 120.0
NEAR_TABLE_STEPS = 4
TIMEOUT_S = 300.0
MAX_CHUNKS = 120  # sim ~34 chunks of 8; real approach ~2x, plus 4-step chunks near the table

# --- Cameras (RealSense manager, same WebRTC signaling API as novapolicy) -----
CAMERA_SERVER = "http://172.31.11.253/cell/realsense"
CAM_WRIST = "427622271414"  # D405
CAM_SIDE = "319522063360"  # D415, angled view across the table
CAM_TOP = "314522065367"  # D415, bird's-eye view straight down
# Sim cameras rendered 320x240 (4:3). Real frames are 16:9: centre-crop to 4:3, then resize.
IMAGE_SIZE = (320, 240)

# --- Zimmer gripper -------------------------------------------------------------
# Turck TBEN-S2-4IOL. Homing opens the jaws to an 80.00 mm gap; the open() that
# wait_until_ready() sends after it aims for 78.44 mm (1 mm device limit) and reports
# diagnosis 0x0301 without moving. The library classes 0x0301 as recoverable, not an error.
GRIPPER_HOST = "172.31.13.49"
# In Zimmer force mode the grip force also sets the closing speed (open -> 50 mm gap:
# 20% 3.7 s, 50% 1.8 s, 100% 1.5 s; drive velocity has no effect). 50% for the PLA cube.
GRIPPER_FORCE_PERCENT = 50
GRIPPER_VELOCITY_PERCENT = 100  # positioning moves (open_and_wait) only
GRIPPER_OPEN_GAP_M = 0.080
# Training env (UR10ePlaceCubeEnvCfg) + checkpoint-14000 statistics:
# - state `gripper` is the UMI PrismaticJoint, limits [-0.023, 0.0]; 0.0 = open,
#   -0.023 = closed. The trained data only spans [-0.0151, 0.0001] (fingers stop on the
#   cube), so the value sent is clipped to that. The jaw gap maps linearly onto the joint.
# - action `gripper` is BinaryJointPositionAction, -1 / +1: < 0 closes.
GRIPPER_STATE_OPEN = 0.0
GRIPPER_STATE_CLOSED = -0.023
GRIPPER_STATE_TRAINED_RANGE = (-0.0151, 0.0001)
# Training episode 75 reads -0.0136 while the fingers hold the cube. Sent after a grip
# that reports holding, whatever the real cube's width makes the jaw gap.
GRIPPER_STATE_HOLDING = -0.0136
GRIPPER_ACTION_MODE = "binary"  # "binary": grip()/open(); "gap": move_to_gap_m(value)
GRIPPER_CLOSE_BELOW = 0.0  # binary mode: policy value below this grips
GRIPPER_DRY_RUN_STATE = GRIPPER_STATE_OPEN  # sent in the dry run (gripper not connected)

# --- eef_delta convention ------------------------------------------------------
# Server passes eef_delta through raw (ABSOLUTE/NON_EEF/DEFAULT). It is the training env's
# arm action: DifferentialIK, command_type="pose", use_relative_mode=True, scale 1.0 —
# [dx, dy, dz] in metres + rotation vector in rad, in the robot root frame, applied each
# 30 Hz step to the *measured* pose of the fingertip point (target = current + delta,
# rotation pre-multiplied). That is the "base" mode below.
# Frame evidence: the checkpoint's mean delta (-0.0025, +0.0032) matches the net
# home -> yellow square move in NOVA base (-215, +258) mm in sign and ratio; the scene
# frame (NOVA rotated -90 deg about z, from TARGET_XY vs. GOAL_XY) gives (-258, -215).
# So the root frame is NOVA's base frame: no rotation.
EEF_DELTA_POSITION_SCALE = 1000.0  # policy unit -> mm (1000 = metres)
EEF_DELTA_FRAME = "base"  # "base" or "tool"
EEF_DELTA_CUMULATIVE = True  # True: step-to-step deltas; False: each vs. chunk start
# Each delta is an IK *command* the sim PD tracked for only 4 substeps, so the robot moved
# less than the delta per step. GAIN = realized / commanded motion, fitted on training
# episode 75 (FK with NOVA's DH + umi_corrected on arm_joints vs. eef_delta, 271 frames):
# position 0.24 (R^2 0.84; the -90/+90/180 deg frames all fit R^2 < 0.2), rotation 0.30
# (R^2 0.59). Same episode: 7.9 mm realized per step on average, 20 mm at most.
EEF_DELTA_POS_GAIN = 0.24
# Fitted rotation gain is 0.30, but on real images the policy keeps commanding about
# -0.022 rad/step of yaw and never corrects it (run 4: 78 deg yaw + 34 deg tilt after
# ~184 steps; the sim episode rotated 6.7 deg in total). Hold the orientation instead.
EEF_DELTA_ROT_GAIN = 0.0

# --- Safety guards on decoded chunks ------------------------------------------
MAX_STEP_MM = 25.0  # largest allowed move between two consecutive targets (sim max 20)
MAX_CHUNK_TRAVEL_MM = 160.0  # largest allowed distance from the start pose
MAX_STEP_DEG = 5.0  # largest allowed rotation between two consecutive targets
MAX_TILT_DEG = 15.0  # largest allowed angle between the tool axis and straight down
# The controller's safety plane (the table) is z = 0 in the base frame. The TCP is the
# finger-pad midpoint, so it sits ~half a cube height above the table when gripping.
# Episode 75 grasped at z 41.7 and released at z 29.8 (min 29.5).
MIN_TCP_Z_MM = 25.0
# Absolute workspace box in NOVA base (mm): the training cube range x [0.36, 0.70],
# y [-0.40, -0.06] m and target (0.42, 0.0) m, mapped from the scene frame
# (x_nova = y_scene, y_nova = -x_scene), plus the home TCP. Episode 75's TCP path spans
# x [-376, 181], y [-758, -271], z >= 29.5 (grasp at z 41.7, release at z 29.8).
WORKSPACE_X_MM = (-460.0, 275.0)
WORKSPACE_Y_MM = (-800.0, -240.0)


class ZimmerGripper:
    """Feeds the Zimmer jaw gap to the policy and executes its ``gripper`` action.

    ``GripperSession`` is blocking and thread based, so commands run in a worker
    thread and never stall the executor loop. In binary mode a command is only
    sent when the open/closed decision changes; ``grip()`` blocks until the jaws
    hold or reach the end, and a newer decision waits for it.
    """

    def __init__(self, host: str) -> None:
        self._session = GripperSession(SessionConfig(modbus=ModbusConfig(host=host)))
        self._task: asyncio.Task[None] | None = None
        self._closed: bool | None = None
        self._holding = False
        self.last_action: list[float] = []  # raw 16-step `gripper` action of the last chunk
        self.executed_steps = N_ACTION_STEPS  # steps of the last chunk that actually run

    async def connect(self) -> None:
        """Start the controller, home, and open (moves the jaws)."""
        self._session.connect()
        await asyncio.to_thread(self._session.wait_until_ready)
        self._session.set_force_percent(GRIPPER_FORCE_PERCENT)
        self._session.set_velocity_percent(GRIPPER_VELOCITY_PERCENT)
        self._closed = False

    async def close(self) -> None:
        if self._task is not None:
            await asyncio.gather(self._task, return_exceptions=True)
        self._session.close()

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
        opening = min(max(state.jaw_gap_m / GRIPPER_OPEN_GAP_M, 0.0), 1.0)
        joint = GRIPPER_STATE_CLOSED + opening * (GRIPPER_STATE_OPEN - GRIPPER_STATE_CLOSED)
        if self._closed and self._holding:
            return {"gripper": GRIPPER_STATE_HOLDING}
        low, high = GRIPPER_STATE_TRAINED_RANGE
        return {"gripper": min(max(joint, low), high)}

    def next_switch(self, *, closed: bool | None = None) -> tuple[int, bool] | None:
        """First executed step whose open/closed decision differs from ``closed``.

        ``closed`` defaults to the last commanded state.
        """
        current = self._closed if closed is None else closed
        for index, value in enumerate(self.last_action[: self.executed_steps]):
            close = value < GRIPPER_CLOSE_BELOW
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
        if GRIPPER_ACTION_MODE == "gap":
            gap = action_to_gap_m(self.last_action[0])
            await asyncio.to_thread(self._session.move_to_gap_m, gap)
            return
        switch = self.next_switch()
        if switch is None:
            return
        if self._task is not None and not self._task.done():
            return  # previous grip/open still running; re-decide on the next chunk
        index, close = switch
        self._closed = close
        self._task = asyncio.create_task(self._command(close=close, delay_s=index * DT_MS / 1000))

    async def _command(self, *, close: bool, delay_s: float = 0.0) -> None:
        await asyncio.sleep(delay_s)
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


def action_to_gap_m(value: float) -> float:
    """Gap mode: map the -1 (closed) .. +1 (open) policy action to a jaw gap."""
    return (min(max(value, -1.0), 1.0) + 1.0) / 2.0 * GRIPPER_OPEN_GAP_M


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
    pos_gain: float = EEF_DELTA_POS_GAIN,
    rot_gain: float = EEF_DELTA_ROT_GAIN,
) -> list[list[float]]:
    """Compose ``eef_delta`` steps onto ``start`` ([x, y, z, rx, ry, rz], mm / rotvec)."""
    p_start = np.asarray(start[:3], dtype=float)
    r_start = Rotation.from_rotvec(start[3:6])
    p = p_start.copy()
    r: Rotation = r_start
    out: list[list[float]] = []
    for step in deltas:
        dp = np.asarray(step[:3], dtype=float) * EEF_DELTA_POSITION_SCALE * pos_gain
        dr = Rotation.from_rotvec(np.asarray(step[3:6], dtype=float) * rot_gain)
        if not EEF_DELTA_CUMULATIVE:
            p, r = p_start.copy(), r_start
        if EEF_DELTA_FRAME == "tool":
            p += r.apply(dp)
            r *= dr
        else:
            p += dp
            r = cast("Rotation", dr * r)  # base frame: pre-multiply
        prev_rv = out[-1][3:6] if out else start[3:6]
        out.append([*p.tolist(), *continuous_rotvec(r.as_rotvec(), prev_rv)])
    return out


def continuous_rotvec(rv: np.ndarray, prev: list[float]) -> list[float]:
    """Pick the rotvec equivalent of ``rv`` closest to ``prev``.

    Near pi (tool-down) scipy flips the sign of the axis. NOVA's chunk ramps
    interpolate rotvecs component-wise, so a flip would swing the wrist.
    """
    angle = float(np.linalg.norm(rv))
    if angle < 1e-9:
        return rv.tolist()
    alt = rv * (angle - 2.0 * math.pi) / angle  # same rotation, other side of pi
    prev_arr = np.asarray(prev, dtype=float)
    best = alt if np.linalg.norm(alt - prev_arr) < np.linalg.norm(rv - prev_arr) else rv
    return best.tolist()


def executed_steps(start: list[float]) -> int:
    """Steps to run from a chunk that starts at ``start``: fewer near the table."""
    return NEAR_TABLE_STEPS if start[2] < NEAR_TABLE_Z_MM else N_ACTION_STEPS


def tilt_deg(pose: list[float]) -> float:
    """Angle between the tool z-axis and straight down (base -z), in degrees."""
    tool_z = Rotation.from_rotvec(pose[3:6]).apply([0.0, 0.0, 1.0])
    return math.degrees(math.acos(max(-1.0, min(1.0, -tool_z[2]))))


def check_targets(start: list[float], targets: list[list[float]]) -> None:
    """Raise if a chunk jumps further than the guards allow."""
    prev = start
    for i, t in enumerate(targets):
        step_mm = math.dist(prev[:3], t[:3])
        step_deg = math.degrees(
            (Rotation.from_rotvec(t[3:6]) * Rotation.from_rotvec(prev[3:6]).inv()).magnitude()
        )
        travel_mm = math.dist(start[:3], t[:3])
        in_box = (
            WORKSPACE_X_MM[0] <= t[0] <= WORKSPACE_X_MM[1]
            and WORKSPACE_Y_MM[0] <= t[1] <= WORKSPACE_Y_MM[1]
            and t[2] >= MIN_TCP_Z_MM
        )
        if (
            step_mm > MAX_STEP_MM
            or step_deg > MAX_STEP_DEG
            or tilt_deg(t) > MAX_TILT_DEG
            or travel_mm > MAX_CHUNK_TRAVEL_MM
            or not in_box
        ):
            msg = (
                f"eef_delta step {i} exceeds guards: step={step_mm:.1f} mm, "
                f"rot={step_deg:.1f} deg, tilt={tilt_deg(t):.1f} deg, travel={travel_mm:.1f} mm, "
                f"pos={[round(v, 1) for v in t[:3]]}"
            )
            raise RuntimeError(msg)
        prev = t


class SingleArmGr00tClient(Gr00tPolicyClient):
    """GR00T client for the ``arm_joints``/``eef_delta`` model on this cell."""

    def __init__(self, *args: Any, gripper: ZimmerGripper, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._gripper = gripper
        self.chunks = 0

    async def validate_schema(self, schema: PolicySchema) -> None:
        """Like the stock check, but ``gripper`` comes from an ``Observation.computed``."""
        info = await self.get_server_info()
        provided = {m.key for m in schema.joint_mappings} | {"gripper"}
        missing = set(cast("list[str]", info["state_keys"])) - provided
        missing |= set(cast("list[str]", info["video_keys"])) - set(schema.image_sources)
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
        obs["state"] = {k: v for k, v in obs["state"].items() if k in {"arm_joints", "gripper"}}
        obs["language"] = {LANGUAGE_KEY: [[TASK]]}
        return obs

    def _decode_action(
        self, schema: PolicySchema, action: dict[str, object], info: dict[str, object]
    ) -> ActionChunk:
        chunk = super()._decode_action(schema, action, info)
        # Not a NOVA IO: the Action.computed hook reads it from the gripper bridge.
        self._gripper.last_action = np.asarray(action["gripper"], dtype=float).reshape(-1).tolist()
        return chunk

    async def get_actions(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, Any] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        # A grip/open started by the last chunk finishes before the next observation,
        # so the policy never lifts on jaws that are still closing.
        await self._gripper.wait_idle()
        # Sequential execution: the previous chunk has finished, so the arm stands still here.
        if self.chunks >= MAX_CHUNKS:
            msg = f"Chunk limit reached ({MAX_CHUNKS}); stopping at standstill"
            raise RuntimeError(msg)
        self.chunks += 1
        frames = to_training_view(images) if images else images
        chunk = await super().get_actions(states, schema, frames, io_values)
        tcp: dict[str, list[list[float]]] = {}
        for mg_id, deltas in chunk.tcp.items():
            if states[mg_id].tcp != TCP:
                msg = f"State pose is for TCP {states[mg_id].tcp!r}, expected {TCP!r}"
                raise RuntimeError(msg)
            pose = states[mg_id].pose
            start = [*pose.position, *pose.orientation]
            steps = executed_steps(start)
            targets = deltas_to_absolute(start, deltas)[:steps]
            check_targets(start, targets)
            tcp[mg_id] = targets
            self._gripper.executed_steps = steps
        return chunk.model_copy(update={"tcp": tcp})


EXECUTE = False


@program(id="gr00t_ur10e_calibration", name="GR00T UR10e Calibration", viewer=viewers.Rerun())
async def gr00t_ur10e_calibration(ctx: ProgramContext):
    cell = ctx.nova.cell()
    controller = await cell.controller(CONTROLLER)
    mg = controller[0]

    # Full frames; to_training_view() crops and resizes them before inference.
    cameras = WebRTCCameras(api_url=CAMERA_SERVER, frame_history=1)
    images = {
        "wrist": cameras.device(CAM_WRIST),
        "side": cameras.device(CAM_SIDE),
        "top": cameras.device(CAM_TOP),
    }
    gripper = ZimmerGripper(GRIPPER_HOST)

    async def dry_run_gripper_state(_obs: dict[str, Any]) -> dict[str, float]:
        return {"gripper": GRIPPER_DRY_RUN_STATE}

    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm_joints", source=mg, action=False),
            # Not sent to the model; pins `umi_corrected` for state + Cartesian jogging.
            Observation.tcp("tcp_pose", source=mg, tcp=TCP),
            Observation.computed(gripper.observe if EXECUTE else dry_run_gripper_state),
            *(Observation.image(key, source=src) for key, src in images.items()),
            Observation.constant("language", value=TASK),
        ],
        actions=[
            Action.tcp("eef_delta", target=mg, mode="absolute"),
            Action.computed(gripper.act),
        ],
    )

    client = SingleArmGr00tClient(host=GROOT_HOST, port=GROOT_PORT, dt_ms=DT_MS, gripper=gripper)
    await client.connect([mg.id])
    print(f"GR00T server: {await client.get_server_info()}")

    if not EXECUTE:
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

    try:
        print("Connecting gripper (homing + open)...")
        await gripper.connect()

        print("Moving to home...")
        actions = [joint_ptp(HOME, settings=HOME_SETTINGS)]
        trajectory = await mg.plan(actions, TCP)
        await mg.execute(trajectory, TCP, actions=actions)

        executor = PolicyExecutor(
            schema,
            client,
            timeout_s=TIMEOUT_S,
            execution=SequentialExecution(),
            n_action_steps=N_ACTION_STEPS,
        )
        print(f"Running GR00T policy for up to {TIMEOUT_S}s...")
        t0 = time.monotonic()
        try:
            result = await executor.run()
            dt = time.monotonic() - t0
            print(f"\nDone: reason={result.reason} steps={result.steps} duration={dt:.1f}s")
        except Exception as e:
            dt = time.monotonic() - t0
            print(f"\nError after {dt:.1f}s: {type(e).__name__}: {e}")
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
        print(f"Gripper state sent (not connected in dry run): {GRIPPER_DRY_RUN_STATE}")
        print(f"Frames: { {k: v.shape for k, v in frames.items()} }")

        t0 = time.monotonic()
        # Raw chunk (skip this subclass's conversion) so the deltas are visible.
        raw = await Gr00tPolicyClient.get_actions(client, {mg.id: state}, schema, frames)
        print(f"Inference: {(time.monotonic() - t0) * 1000:.0f} ms, dt_ms={raw.dt_ms}")
        start = [*state.pose.position, *state.pose.orientation]
        print(f"Gains: position {EEF_DELTA_POS_GAIN}, rotation {EEF_DELTA_ROT_GAIN}")
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
        if GRIPPER_ACTION_MODE == "gap":
            command = f"move_to_gap_m({action_to_gap_m(gripper.last_action[0]):.4f})"
        else:
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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="move robot + gripper (asks first)")
    EXECUTE = parser.parse_args().execute
    run_program(gr00t_ur10e_calibration)
