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

The gripper (``zimmer_gripper.py``) is a Zimmer GEH6060IL on a Turck IO-Link master, driven
by the optional ``zimmer-gripper-controller`` package over Modbus TCP.

Without ``--execute`` a script does a DRY RUN: one real observation, one inference, the
targets and limit check printed. Nothing is commanded, and the gripper is not connected
(connecting starts its homing, which moves the jaws). ``--execute`` moves the robot and
the gripper; it asks for confirmation first.
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import math
from pathlib import Path
import sys
import time
from typing import TYPE_CHECKING, Any

import numpy as np
from settings_from_episode import EpisodeSettings
from zimmer_gripper import GripperStates, HoldWhileGripping, ZimmerGripper

from nova import ProgramContext, api, program, run_program, viewers
from nova.actions import joint_ptp, linear
from nova.types import MotionSettings, Pose
from novapolicy import (
    Action,
    ContinuousExecution,
    Gr00tPolicyClient,
    Observation,
    PolicyExecutor,
    PolicySchema,
    QueuedPolicyClient,
    SequentialExecution,
    TcpLimits,
    WebRTCCameras,
    resolve_actions,
)
from novapolicy.chunking import tcp_limit_violations

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup

logger = logging.getLogger(__name__)

# --- Measured from a training episode (settings_from_episode.py --write) -------------------
# Rate, home, the deltas' coordinate system, the training controller's gains and the gripper
# values; README.md shows how to regenerate the file for another policy or episode.
SETTINGS = EpisodeSettings.load(Path(__file__).with_name("episode_settings.json"))
ARM = SETTINGS.robots["arm"]
if ARM.gripper_open is None or ARM.gripper_closed is None or ARM.gripper_holding is None:
    sys.exit("episode_settings.json has no gripper for 'arm': regenerate it with gripper=...")
CONTROLLER = ARM.motion_group.split("@", 1)[1]
TCP = ARM.tcp  # the fit was made for this TCP

# --- The policy server and what the episode cannot tell -------------------------------------
GROOT_HOST, GROOT_PORT = "172.31.11.129", 30555
PROMPT = "pick up the cube and place it on the yellow square"
IMAGE_SIZE = (320, 240)  # the sim cameras' resolution (dataset meta/info.json)

# --- The cell ----------------------------------------------------------------------------
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
    home = tuple(math.radians(d) for d in ARM.home_deg)
    actions = [joint_ptp(home, settings=HOME_SETTINGS)]
    await mg.execute(await mg.plan(actions, TCP), TCP, actions=actions)


def build(
    mg: MotionGroup, *, continuous: bool
) -> tuple[Gr00tPolicyClient, PolicySchema, ZimmerGripper, dict[str, Any]]:
    """The GR00T client, the schema, the gripper and the cameras."""
    dt_ms = 1000.0 / SETTINGS.rate_hz / SPEED_FACTOR
    client = Gr00tPolicyClient(host=GROOT_HOST, port=GROOT_PORT, dt_ms=dt_ms)
    # Sequential: a switch anywhere in the steps that run. Continuous: the policy is asked
    # again every few steps, so act on the current step's decision only.
    gripper = ZimmerGripper(
        client,
        GRIPPER_HOST,
        states=GripperStates(
            open=ARM.gripper_open,
            closed=ARM.gripper_closed,
            holding=ARM.gripper_holding,
        ),
        close_when_negative=bool(ARM.gripper_close_when_negative),
        state_key=ARM.gripper_state_key,
        action_key=ARM.gripper_action_key,
        open_gap_mm=GRIPPER_OPEN_GAP_MM,
        lookahead_steps=1 if continuous else N_ACTION_STEPS,
        dt_ms=dt_ms,
        force_percent=GRIPPER_FORCE_PERCENT,
    )
    webrtc = WebRTCCameras(api_url=CAMERA_SERVER, resize=IMAGE_SIZE, resize_mode="crop")
    cameras = {key: webrtc.device(serial) for key, serial in CAMERAS.items()}
    schema = PolicySchema(
        observations=[
            Observation.joint_positions(ARM.joints_key, source=mg, action=False),
            # Not a model input: pins the TCP the relative actions compose with.
            Observation.tcp("tcp_pose", source=mg, tcp=TCP),
            Observation.computed(gripper.observe, keys=[ARM.gripper_state_key]),
            *(Observation.image(key, source=cam) for key, cam in cameras.items()),
            Observation.constant("language", value=PROMPT),
        ],
        actions=[
            ARM.action(mg),  # the measured reading: TCP deltas, units and gains
            Action.computed(gripper.act),
        ],
    )
    return client, schema, gripper, cameras


def make_program(*, continuous: bool, execute: bool) -> Any:
    """The NOVA program for one execution mode; dry run unless ``execute``."""

    @program(id="gr00t_ur10e_calibration", name="GR00T UR10e Calibration", viewer=viewers.Rerun())
    async def gr00t_ur10e_calibration(ctx: ProgramContext) -> None:
        mg = (await ctx.nova.cell().controller(CONTROLLER)).motion_group(ARM.motion_group)
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
                f"This homes and opens the gripper, moves {CONTROLLER} to home {ARM.home_deg}, "
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
        held = HoldWhileGripping(
            client,
            gripper,
            mg.id,
            dt_ms=1000.0 / SETTINGS.rate_hz / SPEED_FACTOR,
            hold_steps=HOLD_STEPS,
        )
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
        targets = resolve_actions(schema, raw, {mg.id: state}).tcp[mg.id]  # a TCP policy
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
