"""Reach a detected part with a trained two-point reach policy.

The policy is a LEAPP export (``exported/reach/``) that ``LeappPolicyClient`` runs locally at the
30 Hz it was trained at, one action per step. It observes the joint positions and velocities and a
target pose, and outputs the next joint target. The target comes from a part detection: the
flange goes tool down to APPROACH_MM above wherever the part is detected. The policy was trained on
two part positions, A and B; a part detected anywhere else stops the run before the robot moves
toward it.

The detection is an observation signal like any other, a NOVA pose (mm, rotation vector) that the
schema turns into the metres and quaternion the policy was trained on. Two detectors are built in:

- scripted (default): the part is reported at each position of ``--parts`` for 4 s;
- ``--isaac-sim URL --prim PATH``: the part's prim in an Isaac Sim scene, through the Wandelbots
  NOVA extension (Isaac Sim may run on another machine).

A camera or pose estimator plugs in the same way: anything with ``start``/``stop``/``read``.

Dry run by default: one observation and one inference, the target printed, nothing commanded.
Either ``--virtual`` (a throwaway virtual UR10e, deleted afterwards) or ``--controller NAME`` is
required; with ``--controller``, ``--execute`` asks before it moves the robot.
"""

from __future__ import annotations

import argparse
import asyncio
import math
import os
from pathlib import Path
import sys
import time
from typing import TYPE_CHECKING

import numpy as np
from scipy.spatial.transform import Rotation

import nova
from nova import api, run_program
from nova.actions import jnt
from nova.cell import virtual_controller
from nova.types import Pose
from novapolicy import (
    Action,
    ContinuousExecution,
    EmergencyStopError,
    LeappPolicyClient,
    MotionError,
    Observation,
    OmniverseScene,
    OrientationType,
    PolicyExecutor,
    PolicySchema,
    Scale,
    StopContext,
)

if TYPE_CHECKING:
    from nova.cell.motion_group import MotionGroup
    from novapolicy import PoseSource

EXPORT = Path(__file__).with_name("exported") / "reach" / "reach.yaml"
START_JOINTS = [0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]  # as trained
TOOL_DOWN = (math.pi / math.sqrt(2), math.pi / math.sqrt(2), 0.0)  # rotation vector, as at START
APPROACH_MM = 100.0  # the flange stops this far above the part
PART_POSITIONS = {  # where the part can lie, mm in the robot's base frame (trained: flange above)
    "A": (-750.0, -350.0, 400.0),
    "B": (-750.0, 50.0, 400.0),
}
TRAINED_TOLERANCE_MM = 30.0  # a part detected farther than this from A and B stops the run
SWITCH_S = 4.0  # how long the scripted detector reports each position
TCP = "Flange"
VIRTUAL_CONTROLLER = "leapp-reach-ur10e"


class ScriptedDetection:
    """Stands in for a part detector: reports the part at each position of ``sequence`` for
    SWITCH_S, from the first read on, then keeps the last one."""

    def __init__(self, sequence: list[str]) -> None:
        unknown = set(sequence) - set(PART_POSITIONS)
        if unknown:
            msg = f"Unknown part positions {sorted(unknown)}; use {sorted(PART_POSITIONS)}"
            raise ValueError(msg)
        self.sequence = sequence
        self._started: float | None = None

    async def start(self) -> None:
        self._started = None

    async def stop(self) -> None:
        pass

    def read(self) -> Pose:
        if self._started is None:
            self._started = time.monotonic()
        visit = int((time.monotonic() - self._started) // SWITCH_S)
        name = self.sequence[min(visit, len(self.sequence) - 1)]
        return Pose((*PART_POSITIONS[name], *TOOL_DOWN))


class PartTarget:
    """The flange target for a detected part: tool down, APPROACH_MM above it. A pose source,
    so the executor starts and stops the detector with the run."""

    def __init__(self, detector: PoseSource) -> None:
        self.detector = detector

    async def start(self) -> None:
        await self.detector.start()

    async def stop(self) -> None:
        await self.detector.stop()

    def read(self) -> Pose:
        part = self.detector.read()
        x, y, z = part.position
        return Pose((x, y, z + APPROACH_MM, *TOOL_DOWN))

    def trained_position(self) -> tuple[str | None, float]:
        """The trained position nearest to the detected part, and its distance (mm)."""
        part = list(self.detector.read().position)
        name, distance = min(
            ((n, math.dist(part, p)) for n, p in PART_POSITIONS.items()), key=lambda item: item[1]
        )
        return (name if distance <= TRAINED_TOLERANCE_MM else None), distance


def build_schema(mg: MotionGroup, target: PartTarget) -> PolicySchema:
    return PolicySchema(
        observations=[
            Observation.joint_positions("joint_pos", source=mg, action=False),
            Observation.joint_velocities("joint_vel", source=mg),
            Observation.pose(
                "ee_pose",  # the export's command:ee_pose input: the detected part's target
                source=target,
                orientation_type=OrientationType.QUATERNION,  # (x, y, z, w), as trained
                position_ops=[Scale(0.001)],  # mm -> m
            ),
            Observation.tcp("flange", source=mg, tcp=TCP),  # for the report
        ],
        actions=[Action.joint_positions("arm", target=mg)],  # the export outputs absolute targets
    )


def part_outside_trained_positions(target: PartTarget):
    """Stop before the robot moves toward a part the policy was not trained to reach."""

    def outside(_ctx: StopContext) -> bool:
        name, distance = target.trained_position()
        if name is None:
            outside.__name__ = f"part_outside_trained_positions ({distance:.0f} mm from A or B)"
        return name is None

    outside.__name__ = "part_outside_trained_positions"
    return outside


class Recorder:
    """A stop condition that never stops: it records the flange and the detected position."""

    __name__ = "recorder"

    def __init__(self, target: PartTarget) -> None:
        self.target = target
        self.samples: list[tuple[str, Pose, Pose]] = []

    def __call__(self, ctx: StopContext) -> bool:
        name, _ = self.target.trained_position()
        self.samples.append((name or "?", ctx.state.pose, self.target.read()))
        return False

    def report(self) -> list[tuple[str, float, float]]:
        """Per detected position: its name, the flange distance (mm) and tool-axis angle (deg)
        to its target, at the end of the time the part was there."""
        if not self.samples:
            return []
        ends = [
            i for i in range(len(self.samples) - 1) if self.samples[i][0] != self.samples[i + 1][0]
        ]
        ends.append(len(self.samples) - 1)
        rows = []
        for i in ends:
            name, pose, goal = self.samples[i]
            distance = math.dist(list(pose.position), list(goal.position))
            axis = Rotation.from_rotvec(list(pose.orientation)).as_matrix()[:, 2]
            goal_axis = Rotation.from_rotvec(list(goal.orientation)).as_matrix()[:, 2]
            angle = math.degrees(math.acos(float(np.clip(axis @ goal_axis, -1.0, 1.0))))
            rows.append((name, distance, angle))
        return rows


def detector_from(args: argparse.Namespace) -> PoseSource:
    if args.isaac_sim:
        return OmniverseScene(args.isaac_sim).prim(args.prim, relative_to=args.relative_to)
    return ScriptedDetection(args.parts.split(","))


def virtual_robot() -> nova.ProgramPreconditions:
    """A virtual UR10e that NOVA creates before the program and deletes after it."""
    return nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name=VIRTUAL_CONTROLLER,
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
            )
        ],
        cleanup_controllers=True,
    )


def make_program(args: argparse.Namespace):
    """The NOVA program for these arguments: on a virtual UR10e with ``--virtual``."""

    @nova.program(
        id="leapp_two_point_reach",
        viewer=nova.viewers.Rerun(),  # 3D view of the robot (needs the nova-rerun-bridge extra)
        name="LEAPP two-point reach",
        preconditions=virtual_robot() if args.virtual else None,
    )
    async def leapp_two_point_reach(ctx: nova.ProgramContext) -> None:
        controller = await ctx.cell.controller(
            VIRTUAL_CONTROLLER if args.virtual else args.controller
        )
        async with controller[0] as mg:
            await run(mg, args)

    return leapp_two_point_reach


async def run(mg: MotionGroup, args: argparse.Namespace) -> None:
    client = LeappPolicyClient(args.export)
    target = PartTarget(detector_from(args))
    schema = build_schema(mg, target)

    if not args.execute:
        await schema.start_sources()
        try:
            await client.connect([mg.id])
            await client.validate_schema(schema)
            state = await mg.get_state(TCP)
            chunk = await client.get_actions({mg.id: state}, schema)
            position, distance = target.trained_position()
            print(f"part detected at {target.detector.read().position} ({position or 'untrained'})")
            print(f"joints now (deg):   {[round(math.degrees(q), 1) for q in state.joints]}")
            print(
                f"joint target (deg): {[round(math.degrees(q), 1) for q in chunk.joints[mg.id][0]]}"
            )
            print(f"{client.control_freq:g} Hz; dry run, nothing commanded")
        finally:
            await schema.stop_sources()
        return

    if not args.virtual:
        answer = await asyncio.to_thread(
            input,
            f"This moves the real robot {args.controller} to the start pose and then to the "
            "detected part. Type 'yes': ",
        )
        if answer.strip().lower() != "yes":
            print("Aborted, nothing moved.")
            return
    await mg.plan_and_execute([jnt(START_JOINTS)], tcp=TCP)

    recorder = Recorder(target)
    duration_s = args.duration or len(args.parts.split(",")) * SWITCH_S
    executor = PolicyExecutor(
        schema,
        client,
        execution=ContinuousExecution(rate_hz=client.control_freq),
        n_action_steps=0,  # one step per inference, replaced by the next one
        stop_conditions=[part_outside_trained_positions(target), recorder],
        timeout_s=duration_s,
    )
    try:
        result = await executor.run()
    except (MotionError, EmergencyStopError) as exc:  # NOVA stopped the robot
        print(f"Stopped by the controller: {exc}")
        raise
    rate = result.steps / result.duration_s if result.duration_s else 0.0
    print(f"{result.reason} after {result.steps} policy steps ({rate:.1f} Hz)")
    for position, distance, angle in recorder.report():
        print(
            f"  part at {position}: flange {distance:6.1f} mm from its target, "
            f"tool axis {angle:4.1f} deg off"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--export", type=Path, default=EXPORT, help="the LEAPP export's .yaml")
    robot = parser.add_mutually_exclusive_group(required=True)
    robot.add_argument("--controller", help="an existing NOVA controller (asks before moving it)")
    robot.add_argument("--virtual", action="store_true", help="a throwaway virtual UR10e")
    parser.add_argument("--execute", action="store_true", help="move the robot")
    parser.add_argument(
        "--parts", default="A,B,A", help="scripted detections, 4 s each: positions A and B"
    )
    parser.add_argument(
        "--isaac-sim",
        default=os.environ.get("ISAAC_SIM_API"),
        help="detect the part in Isaac Sim: http://isaac-host:8011/omniservice/api/v2",
    )
    parser.add_argument("--prim", default="/World/Part", help="the part's prim (with --isaac-sim)")
    parser.add_argument(
        "--relative-to", default=None, help="the robot base prim (with --isaac-sim)"
    )
    parser.add_argument("--duration", type=float, default=None, help="run time in s")
    runner = run_program(make_program(parser.parse_args()))
    if runner.program_run.state == api.models.ProgramRunState.FAILED:
        sys.exit(1)


if __name__ == "__main__":
    main()
