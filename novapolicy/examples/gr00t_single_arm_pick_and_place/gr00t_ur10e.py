"""Example: Run a GR00T policy on the real single-arm UR10e ``ur10e-calibration``.

Shared implementation of ``run_sequential.py`` and ``run_continuous.py``; see README.md
next to this file for how to start them. Every policy fact, the cell and the guards come
from ``policy.toml``; :class:`~novapolicy.gr00t.declared.DeclaredGr00tClient` runs the
policy as declared there (prompt key, image framing, step spacing, delta -> absolute TCP
targets, guards, plan queue). What is left here is this cell's hardware and the run.

The gripper is a Zimmer GEH6060IL on a Turck IO-Link master, driven by the
``zimmer-gripper-controller`` package over Modbus TCP. It enters the policy through
``Observation.computed`` (state) and ``Action.computed`` (command).

Without ``--execute`` a script does a DRY RUN: one real observation, one inference, the
targets printed. Nothing is commanded, and the gripper is not connected (connecting
starts its homing, which moves the jaws). ``--execute`` moves the robot and the gripper;
it asks for confirmation first.
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
from typing import TYPE_CHECKING, Any

import numpy as np

from nova import ProgramContext, api, program, run_program, viewers
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
from novapolicy.gr00t import declared, policy_config
from novapolicy.gr00t.declared import (
    DeclaredGr00tClient,
    GuardError,
    check_guards,
    frame_images,
    gripper_state,
    move_home,
    next_gripper_switch,
    tcp_targets,
)

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup

logger = logging.getLogger(__name__)

CONFIG_PATH = Path(__file__).with_name("policy.toml")
CFG = policy_config.load(CONFIG_PATH)
CONTROLLER = CFG.cell.controller

# --- Execution (not part of the policy) -------------------------------------------------
# Playback speed relative to the training rate: 1.0 plays one policy step per 1 / rate_hz.
SPEED_FACTOR = 1.0
N_ACTION_STEPS = 8
# Sequential: near the table, run half chunks so the policy re-observes (and corrects its
# placement over the cube) twice as often during the descent.
NEAR_TABLE_Z_MM = 120.0
NEAR_TABLE_STEPS = N_ACTION_STEPS // 2
MAX_CHUNKS = 120  # sequential: a cap on chunks per run
TIMEOUT_S = 300.0
SAFE_Z_MM = 200.0  # lift straight up to this before the joint move home

# --- Zimmer gripper hardware ------------------------------------------------------------
# In force mode the grip force also sets the closing speed (open -> 50 mm gap: 20% 3.7 s,
# 50% 1.8 s, 100% 1.5 s). 50% for the PLA cube.
GRIPPER_FORCE_PERCENT = 50
GRIPPER_VELOCITY_PERCENT = 100  # positioning moves (open_and_wait) only


def open_gripper_session(host: str = CFG.cell.gripper_host) -> Any:
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
    """Below SAFE_Z_MM, move straight up to it (before a joint move home)."""
    await declared.lift_if_low(mg, CFG, safe_z_mm=SAFE_Z_MM)


class ZimmerGripper:
    """Feeds the Zimmer jaw gap to the policy and executes its gripper action.

    ``GripperSession`` is blocking and thread based, so commands run in a worker thread
    and never stall the executor loop. A command is only sent when the open/closed
    decision changes; ``grip()`` blocks until the jaws hold or reach the end, and the
    client holds the arm still meanwhile (``busy``).
    """

    def __init__(self, cfg: policy_config.PolicyConfig, client: DeclaredGr00tClient) -> None:
        self._cfg = cfg
        self._client = client
        self._session: Any = None  # created in connect(); dry runs never connect
        self._task: asyncio.Task[None] | None = None
        self._task_started = False  # the scheduled grip()/open() has left its delay
        self._closed: bool | None = None
        self._holding = False

    async def connect(self) -> None:
        """Start the controller, home, and open (moves the jaws)."""
        self._session = await asyncio.to_thread(open_gripper_session, self._cfg.cell.gripper_host)
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

    async def observe(self, _obs: dict[str, Any]) -> dict[str, float]:
        """``Observation.computed`` hook: the jaw gap as the policy's gripper state."""
        key = self._cfg.policy.gripper.state_key
        if self._session is None:  # dry run: jaws assumed open
            return {key: self._cfg.policy.gripper.state_open}
        state = self._session.state()
        if state.has_error:
            msg = f"Gripper error: diag=0x{state.diagnosis:04X} {state.last_error_text}"
            raise RuntimeError(msg)
        holding = bool(self._closed and self._holding)
        return {key: gripper_state(self._cfg, state.jaw_gap_m * 1000.0, holding=holding)}

    async def act(self, _chunk: ActionChunk) -> None:
        """``Action.computed`` hook, fired as the arm reaches the chunk's first step.

        Switch at the first executed step that asks for it, delayed by that step's offset,
        so a close at step 5 is not held back to the next chunk.
        """
        if self._session is None or self.busy:
            return  # dry run, or the previous grip/open still runs; re-decide next chunk
        c = self._client
        switch = next_gripper_switch(
            self._cfg, c.gripper_actions, c.executed_steps, closed=self._closed
        )
        if switch is None:
            return
        index, close = switch
        self._closed = close
        self._task_started = False
        delay_s = index * c.dt_ms / 1000.0
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


def make_program(*, continuous: bool, execute: bool) -> Any:
    """The NOVA program for one execution mode; dry run unless ``execute``."""
    cell, p = CFG.cell, CFG.policy

    @program(id="gr00t_ur10e_calibration", name="GR00T UR10e Calibration", viewer=viewers.Rerun())
    async def gr00t_ur10e_calibration(ctx: ProgramContext) -> None:
        mg = (await ctx.nova.cell().controller(cell.controller))[0]

        # Full frames; the client frames them like the training cameras before inference.
        cameras = WebRTCCameras(api_url=cell.camera_server, frame_history=1)
        images = {key: cameras.device(serial) for key, serial in cell.cameras.items()}
        client = DeclaredGr00tClient(
            CFG,
            continuous=continuous,
            speed_factor=SPEED_FACTOR,
            n_action_steps=N_ACTION_STEPS,
            near_z_mm=NEAR_TABLE_Z_MM,
            near_steps=NEAR_TABLE_STEPS,
            max_chunks=MAX_CHUNKS,
        )
        gripper = ZimmerGripper(CFG, client)
        client.hold_while = lambda: gripper.busy  # hold the arm while the jaws move

        schema = PolicySchema(
            observations=[
                Observation.joint_positions(p.joints.key, source=mg, action=False),
                # Not sent to the model; pins the declared TCP for the state poses.
                Observation.tcp("tcp_pose", source=mg, tcp=cell.tcp),
                Observation.computed(gripper.observe),
                *(Observation.image(key, source=src) for key, src in images.items()),
            ],
            actions=[
                Action.tcp(p.tcp_action.key, target=mg, mode="absolute"),
                Action.computed(gripper.act),
            ],
        )

        await client.connect([mg.id])
        try:
            await client.validate_schema(schema)  # the executor checks again before running
            print(f"GR00T server serves the policy declared in {CONFIG_PATH.name}")
            if not execute:
                await dry_run(mg, client, schema, images)
                return
            answer = await asyncio.to_thread(
                input,
                f"This homes and opens the gripper, moves {cell.controller} to home "
                f"{cell.home_deg}, and starts the policy. Type 'yes': ",
            )
            if answer.strip().lower() != "yes":
                print("Aborted, nothing moved.")
                return
            await run_policy(mg, client, schema, gripper)
        finally:
            await client.close()  # the executor closes it too; closing twice is safe

    return gr00t_ur10e_calibration


async def run_policy(
    mg: MotionGroup, client: DeclaredGr00tClient, schema: PolicySchema, gripper: ZimmerGripper
) -> None:
    """Gripper home + open, lift if low, joint move home, then run the policy."""
    try:
        print("Connecting gripper (homing + open)...")
        await gripper.connect()
        print("Moving to home...")
        await move_home(mg, CFG, safe_z_mm=SAFE_Z_MM)

        executor = PolicyExecutor(
            schema,
            client,
            timeout_s=TIMEOUT_S,
            execution=ContinuousExecution() if client.continuous else SequentialExecution(),
            # Continuous: keep the whole horizon as lookahead; each new chunk replaces its
            # tail. Sequential: the client trims each chunk (n_action_steps).
            n_action_steps=0 if client.continuous else None,
        )
        mode = "continuous" if client.continuous else "sequential"
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
    mg: MotionGroup, client: DeclaredGr00tClient, schema: PolicySchema, images: dict[str, Any]
) -> None:
    """One real observation + inference. Prints targets, commands nothing."""
    p, tcp = CFG.policy, CFG.cell.tcp
    for cam in images.values():
        await cam.connect()
    try:
        frames = frame_images(
            {key: cam.read() for key, cam in images.items()}, p.image_width, p.image_height
        )
        state = await mg.get_state(tcp)
        print(f"TCP: {state.tcp}  joints(deg): {[round(math.degrees(j), 1) for j in state.joints]}")
        print(f"Frames: { {k: v.shape for k, v in frames.items()} }")

        t0 = time.monotonic()
        # Raw chunk (skip the declared conversion) so the deltas are visible.
        raw = await Gr00tPolicyClient.get_actions(client, {mg.id: state}, schema, frames)
        print(f"Inference: {(time.monotonic() - t0) * 1000:.0f} ms, dt_ms={raw.dt_ms:.1f}")
        start = [*state.pose.position, *state.pose.orientation]
        targets = tcp_targets(CFG, start, raw.tcp[mg.id])
        steps = client.steps_to_run(start)
        print(f"start pose (mm, rotvec): {np.round(start, 3).tolist()}")
        print(" step  raw eef_delta                              -> target pos (mm)  dpos_mm")
        for i, (d, t) in enumerate(zip(raw.tcp[mg.id], targets, strict=True)):
            dpos = np.subtract(t[:3], start[:3])
            print(
                f"{'*' if i < steps else ' '}{i:2d}  {np.round(d, 4).tolist()}  -> "
                f"{np.round(t[:3], 1).tolist()}  {np.round(dpos, 1).tolist()}"
            )
        switch = next_gripper_switch(CFG, client.gripper_actions, steps, closed=False)
        command = (
            "none (stays open)"
            if switch is None
            else (f"{'grip()' if switch[1] else 'open()'} at step {switch[0]}")
        )
        print(f"Gripper raw: {np.round(client.gripper_actions, 3).tolist()}")
        print(f"Gripper command that would be sent: {command}")
        try:
            check_guards(CFG, start, targets[:steps], client.dt_ms)  # only these execute
            print("Guards: OK")
        except GuardError as e:
            print(f"Guards: REJECTED - {e}")
        print("(* = executed per chunk; dry run, nothing commanded)")
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
