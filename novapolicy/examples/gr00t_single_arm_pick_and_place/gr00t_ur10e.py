"""Example: Run a GR00T policy on the real single-arm UR10e ``ur10e-calibration``.

Shared implementation of ``run_sequential.py`` and ``run_continuous.py``; see README.md
next to this file for how to start them. The policy is an IsaacLab-trained cube
pick-and-place (``UR10ePlaceCubeEnvCfg``) behind a GR00T server; everything about it is a
plain ``PolicySchema``:

- state ``arm_joints`` (joint radians) and ``gripper`` (from the Zimmer gripper),
- three cameras, centre-cropped and resized to the sim cameras' 320 x 240,
- action ``eef_delta``: per-step TCP deltas in metres / rotation vector in the robot base
  frame (the sim's DifferentialIK relative pose command), as ``Action.tcp(mode="relative",
  accumulate=True)``,
- action ``gripper``: a binary command for the Zimmer gripper, read by ``Action.computed``.

The gripper is a Zimmer GEH6060IL on a Turck IO-Link master, driven by the optional
``zimmer-gripper-controller`` package over Modbus TCP.

Without ``--execute`` a script does a DRY RUN: one real observation, one inference, the
targets and limit check printed. Nothing is commanded, and the gripper is not connected
(connecting starts its homing, which moves the jaws). ``--execute`` moves the robot and
the gripper; it asks for confirmation first.
"""

from __future__ import annotations

import argparse
import asyncio
import importlib
import logging
import math
import sys
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from nova import ProgramContext, api, program, run_program, viewers
from nova.actions import joint_ptp, linear
from nova.types import MotionSettings, Pose
from novapolicy import (
    Action,
    ActionChunk,
    ContinuousExecution,
    Gr00tPolicyClient,
    Observation,
    PolicyClient,
    PolicyExecutor,
    PolicySchema,
    QueuedPolicyClient,
    Scale,
    SequentialExecution,
    TcpLimits,
    WebRTCCameras,
    resolve_actions,
)
from novapolicy.chunking import tcp_limit_violations

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup

logger = logging.getLogger(__name__)

# --- The policy (from the training setup) -------------------------------------------------
GROOT_HOST, GROOT_PORT = "172.31.11.129", 30555
PROMPT = "pick up the cube and place it on the yellow square"
RATE_HZ = 30.0  # sim dt 1/120 s, decimation 4
IMAGE_SIZE = (320, 240)  # the sim cameras' resolution (4:3)
# The sim's DifferentialIK tracked each relative pose command for 4 substeps and realized
# only part of it; measured from a training episode (settings_from_episode.py): realized /
# commanded.
GAIN_POSITION, GAIN_ROTATION = 0.24, 0.30
# Gripper: BinaryJointPositionAction, a negative command closes. State: the UMI finger
# joint, 0.0 open, -0.023 closed, -0.0136 when the fingers hold the cube.
GRIPPER_OPEN, GRIPPER_CLOSED, GRIPPER_HOLDING = 0.0, -0.023, -0.0136

# --- The cell ----------------------------------------------------------------------------
CONTROLLER = "ur10e-calibration"
TCP = "umi_corrected"  # finger-pad midpoint, the sim's IK point
HOME_DEG = (90.0, -90.0, 90.0, -90.0, -90.0, 0.0)  # the training episodes' start
CAMERA_SERVER = "http://172.31.11.253/cell/realsense"
CAMERAS = {"wrist": "427622271414", "side": "319522063360", "top": "314522065367"}
GRIPPER_HOST = "172.31.13.49"  # Turck TBEN-S2-4IOL for the Zimmer GEH6060IL
GRIPPER_OPEN_GAP_MM = 80.0  # jaw gap after homing
# Software limits on every chunk (the table is z = 0), not a safety system.
LIMITS = TcpLimits(
    max_speed_mm_s=650.0,
    max_rotation_speed_deg_s=150.0,
    max_chunk_travel_mm=250.0,
    x_mm=(-460.0, 275.0),
    y_mm=(-800.0, -240.0),
    z_mm=(25.0, 1000.0),
)

# --- Execution ---------------------------------------------------------------------------
SPEED_FACTOR = 1.0  # below 1 plays the steps slower than the training rate
N_ACTION_STEPS = 8  # sequential: steps of each chunk that run before the next inference
TIMEOUT_S = 300.0
SAFE_Z_MM = 200.0  # lift straight up to this before the joint move home
HOLD_STEPS = 5  # steps of a stand-still chunk while the gripper moves
HOME_SETTINGS = MotionSettings(tcp_velocity_limit=100.0)
# In force mode the grip force also sets the closing speed (open -> 50 mm gap: 20% 3.7 s,
# 50% 1.8 s, 100% 1.5 s). 50% for the PLA cube.
GRIPPER_FORCE_PERCENT = 50
GRIPPER_VELOCITY_PERCENT = 100  # positioning moves (open_and_wait) only


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


async def lift_if_low(mg: MotionGroup) -> None:
    """Below SAFE_Z_MM, move straight up to it, so a following joint move cannot sweep the
    gripper over the cube or the table."""
    pose = (await mg.get_state(TCP)).pose
    if pose.position.z >= SAFE_Z_MM:
        return
    up = Pose((pose.position.x, pose.position.y, SAFE_Z_MM, *pose.orientation))
    actions = [linear(up, settings=HOME_SETTINGS)]
    await mg.execute(await mg.plan(actions, TCP), TCP, actions=actions)


async def move_home(mg: MotionGroup) -> None:
    await lift_if_low(mg)
    actions = [joint_ptp(tuple(math.radians(d) for d in HOME_DEG), settings=HOME_SETTINGS)]
    await mg.execute(await mg.plan(actions, TCP), TCP, actions=actions)


class ZimmerGripper:
    """Feeds the jaw gap to the policy and executes its ``gripper`` action.

    ``GripperSession`` is blocking and thread based, so commands run in a worker thread.
    A command is only sent when the open/closed decision changes, at the step of the
    chunk that asks for it. While a command runs, :class:`HoldWhileGripping` keeps the
    arm still and the policy is not asked, so its next observation is taken after the
    jaws have settled.
    """

    def __init__(self, client: Gr00tPolicyClient, *, lookahead_steps: int, dt_ms: float) -> None:
        self._client = client
        self._lookahead_steps = lookahead_steps
        self._dt_ms = dt_ms
        self._session: Any = None  # created in connect(); dry runs never connect
        self._task: asyncio.Task[None] | None = None
        self._task_started = False  # the scheduled grip()/open() has left its delay
        self._closed: bool | None = None
        self._holding = False

    async def connect(self) -> None:
        """Start the controller, home, and open (moves the jaws)."""
        self._session = await asyncio.to_thread(open_gripper_session)
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
        """``Observation.computed``: the jaw gap mapped onto the sim's finger joint."""
        if self._session is None:  # dry run: jaws assumed open
            return {"gripper": GRIPPER_OPEN}
        state = self._session.state()
        if state.has_error:
            msg = f"Gripper error: diag=0x{state.diagnosis:04X} {state.last_error_text}"
            raise RuntimeError(msg)
        if self._closed and self._holding:
            return {"gripper": GRIPPER_HOLDING}
        opening = min(max(state.jaw_gap_m * 1000.0 / GRIPPER_OPEN_GAP_MM, 0.0), 1.0)
        return {"gripper": GRIPPER_CLOSED + opening * (GRIPPER_OPEN - GRIPPER_CLOSED)}

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

    A zero relative TCP delta holds the pose the arm is at. The policy's next observation
    is then taken after the jaws have settled, with images and joints that show it.
    """

    def __init__(self, policy: Gr00tPolicyClient, gripper: ZimmerGripper, mg_id: str) -> None:
        self._policy = policy
        self._gripper = gripper
        self._mg_id = mg_id

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
        dt_ms = 1000.0 / RATE_HZ / SPEED_FACTOR
        await asyncio.sleep(dt_ms / 1000.0)  # no inference; don't spin the executor loop
        self._policy.last_raw_action = {}  # nothing new for the gripper hook to act on
        hold = [[0.0] * 6] * HOLD_STEPS
        return ActionChunk(tcp={self._mg_id: hold}, dt_ms=dt_ms)


def build(
    mg: MotionGroup, *, continuous: bool
) -> tuple[Gr00tPolicyClient, PolicySchema, ZimmerGripper, dict[str, Any]]:
    """The GR00T client, the schema, the gripper and the cameras."""
    dt_ms = 1000.0 / RATE_HZ / SPEED_FACTOR
    client = Gr00tPolicyClient(host=GROOT_HOST, port=GROOT_PORT, dt_ms=dt_ms)
    # Sequential: a switch anywhere in the steps that run. Continuous: the policy is asked
    # again every few steps, so act on the current step's decision only.
    gripper = ZimmerGripper(
        client, lookahead_steps=1 if continuous else N_ACTION_STEPS, dt_ms=dt_ms
    )
    webrtc = WebRTCCameras(api_url=CAMERA_SERVER, resize=IMAGE_SIZE, resize_mode="crop")
    cameras = {key: webrtc.device(serial) for key, serial in CAMERAS.items()}
    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm_joints", source=mg, action=False),
            # Not a model input: pins the TCP the relative actions compose with.
            Observation.tcp("tcp_pose", source=mg, tcp=TCP),
            Observation.computed(gripper.observe, keys=["gripper"]),
            *(Observation.image(key, source=cam) for key, cam in cameras.items()),
            Observation.constant("language", value=PROMPT),
        ],
        actions=[
            Action.tcp(
                "eef_delta",
                target=mg,
                mode="relative",
                accumulate=True,  # each step applies to the pose the previous one reached
                frame="base",
                position_ops=[Scale(0.001), Scale(1.0 / GAIN_POSITION)],  # metres, realized
                orientation_ops=[Scale(1.0 / GAIN_ROTATION)],
            ),
            Action.computed(gripper.act),
        ],
    )
    return client, schema, gripper, cameras


def make_program(*, continuous: bool, execute: bool) -> Any:
    """The NOVA program for one execution mode; dry run unless ``execute``."""

    @program(id="gr00t_ur10e_calibration", name="GR00T UR10e Calibration", viewer=viewers.Rerun())
    async def gr00t_ur10e_calibration(ctx: ProgramContext) -> None:
        mg = (await ctx.nova.cell().controller(CONTROLLER))[0]
        client, schema, gripper, cameras = build(mg, continuous=continuous)
        await client.connect([mg.id])
        try:
            await client.validate_schema(schema)  # the executor checks again before running
            print(f"GR00T server {GROOT_HOST}:{GROOT_PORT} serves the policy this schema feeds")
            if not execute:
                await dry_run(mg, client, schema, cameras)
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
    client: Gr00tPolicyClient,
    schema: PolicySchema,
    gripper: ZimmerGripper,
    *,
    continuous: bool,
) -> None:
    """Gripper home + open, lift if low, joint move home, then run the policy."""
    try:
        print("Connecting gripper (homing + open)...")
        await gripper.connect()
        print("Moving to home...")
        await move_home(mg)
        held = HoldWhileGripping(client, gripper, mg.id)
        executor = PolicyExecutor(
            schema,
            QueuedPolicyClient(held) if continuous else held,
            timeout_s=TIMEOUT_S,
            execution=ContinuousExecution() if continuous else SequentialExecution(),
            n_action_steps=0 if continuous else N_ACTION_STEPS,
            tcp_limits=LIMITS,
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


async def dry_run(
    mg: MotionGroup, client: Gr00tPolicyClient, schema: PolicySchema, cameras: dict[str, Any]
) -> None:
    """One real observation + inference, resolved as the executor would. Commands nothing."""
    for cam in cameras.values():
        await cam.connect()
    try:
        state = await mg.get_state(TCP)
        frames = {key: cam.read() for key, cam in cameras.items()}
        print(f"TCP: {state.tcp}  joints(deg): {[round(math.degrees(j), 1) for j in state.joints]}")
        print(f"Frames: { {k: v.shape for k, v in frames.items()} }")
        t0 = time.monotonic()
        raw = await client.get_actions({mg.id: state}, schema, frames)
        print(f"Inference: {(time.monotonic() - t0) * 1000:.0f} ms, dt_ms={raw.dt_ms:.1f}")
        targets = resolve_actions(schema, raw, {mg.id: state}).tcp[mg.id]
        start = [*state.pose.position, *state.pose.orientation]
        print(f"start pose (mm, rotvec): {np.round(start, 3).tolist()}")
        for i, t in enumerate(targets):
            mark = "*" if i < N_ACTION_STEPS else " "
            dpos = np.subtract(t[:3], start[:3])
            print(f"{mark}{i:2d}  {np.round(t[:3], 1).tolist()}  dpos {np.round(dpos, 1).tolist()}")
        command = np.asarray(client.last_raw_action.get("gripper", []), dtype=float).reshape(-1)
        print(f"Gripper action: {[round(v, 2) for v in command.tolist()]}")
        broken = tcp_limit_violations(start, targets[:N_ACTION_STEPS], raw.dt_ms, LIMITS)
        print(f"Limits: {'; '.join(broken) or 'OK'}")
        print("(* = executed per sequential chunk; dry run, nothing commanded)")
    finally:
        for cam in cameras.values():
            await cam.disconnect()


def cli(*, continuous: bool = False) -> None:
    """Entry point of ``run_sequential.py`` / ``run_continuous.py``; exits 1 if the run fails."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="move robot + gripper (asks first)")
    execute = parser.parse_args().execute
    runner = run_program(make_program(continuous=continuous, execute=execute))
    if runner.program_run.state == api.models.ProgramRunState.FAILED:
        sys.exit(1)
