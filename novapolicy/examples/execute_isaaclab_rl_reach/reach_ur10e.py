"""Run an Isaac Lab reinforcement-learning reach policy (``IsaacContrib-Deploy-Reach-UR10e``) on a NOVA UR10e.

The policy is a LEAPP export (``isaaclab leapp export``): ONNX + YAML, run locally by
``LeappPolicyClient`` at the 60 Hz it was trained at, one action per step. It observes the joint
positions and velocities and a target pose; it outputs the next joint target.

Dry run by default: one observation and one inference, the target printed, nothing commanded.
``--execute`` runs the policy until the flange reaches the target or 12 s pass. Either
``--virtual`` (a throwaway virtual UR10e, deleted afterwards) or ``--controller NAME`` is required;
with ``--controller``, ``--execute`` asks before it moves the robot.

The target is a NOVA pose in mm (rotation vector), fixed with ``--target`` or read from an Isaac
Sim prim with ``--isaac-sim`` and ``--prim``. See README.md.
"""

from __future__ import annotations

import argparse
import asyncio
import math
import os
from pathlib import Path
import sys
from typing import TYPE_CHECKING

from scipy.spatial.transform import Rotation

from nova import ProgramContext, ProgramPreconditions, api, program, run_program, viewers
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
    from novapolicy.mapping.pose_sources import PoseSource

EXPORT = Path(__file__).with_name("exported") / "reach_ur10e" / "reach_ur10e.yaml"
# The start pose of the training environment (Isaac Lab's UR10e_CFG): tool down over the table.
START_JOINTS = [math.pi, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]
# A target inside the trained workspace (x 640..1140, y -350..-100, z 100..300 mm, tool down).
DEFAULT_TARGET = (850.0, -225.0, 200.0, math.pi, 0.0, 0.0)
TCP = "Flange"
VIRTUAL_CONTROLLER = "isaaclab-reach-ur10e"  # the policy tracks wrist_3_link, the flange
TIMEOUT_S = 12.0  # the training episode length
REACHED_MM, REACHED_DEG = 5.0, 2.0


def build_schema(mg: MotionGroup, target: Pose | PoseSource) -> PolicySchema:
    """What the reach policy observes and commands, in NOVA's units and conventions."""
    return PolicySchema(
        observations=[
            Observation.joint_positions("joint_pos", source=mg, action=False),
            Observation.joint_velocities("joint_vel", source=mg),
            Observation.pose(
                "ee_pose",  # the export's command:ee_pose input
                source=target,
                orientation_type=OrientationType.QUATERNION,  # (x, y, z, w), as trained
                position_ops=[Scale(0.001)],  # mm -> m
            ),
            Observation.tcp("flange", source=mg, tcp=TCP),  # for the stop condition
        ],
        # The export applies the action scale itself: its output is the absolute joint target.
        actions=[Action.joint_positions("arm", target=mg)],
    )


def target_reached(target: Pose | PoseSource):
    """Stop once the flange is within REACHED_MM and REACHED_DEG of the target."""

    def reached(ctx: StopContext) -> bool:
        goal = target if isinstance(target, Pose) else target.read()
        pose = ctx.state.pose
        distance = math.dist(list(pose.position), list(goal.position))
        turn = Rotation.from_rotvec(list(pose.orientation)).inv() * Rotation.from_rotvec(
            list(goal.orientation)
        )
        return distance < REACHED_MM and math.degrees(turn.magnitude()) < REACHED_DEG

    reached.__name__ = "target_reached"
    return reached


async def dry_run(mg: MotionGroup, client: LeappPolicyClient, schema: PolicySchema) -> None:
    """One observation and one inference; nothing is commanded."""
    await schema.start_sources()
    try:
        await client.connect([mg.id])
        await client.validate_schema(schema)
        state = await mg.get_state(TCP)
        chunk = await client.get_actions({mg.id: state}, schema)
        print(f"joints now (deg):    {[round(math.degrees(q), 1) for q in state.joints]}")
        print(f"joint target (deg):  {[round(math.degrees(q), 1) for q in chunk.joints[mg.id][0]]}")
        print(
            f"step: {chunk.dt_ms:.1f} ms ({client.control_freq:g} Hz); dry run, nothing commanded"
        )
    finally:
        await schema.stop_sources()


def virtual_robot() -> ProgramPreconditions:
    """A virtual UR10e that NOVA creates before the program and deletes after it."""
    return ProgramPreconditions(
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

    @program(
        id="isaaclab_reach_ur10e",
        viewer=viewers.Rerun(),  # 3D view of the robot (needs the nova-rerun-bridge extra)
        name="Isaac Lab reach on a UR10e",
        preconditions=virtual_robot() if args.virtual else None,
    )
    async def isaaclab_reach_ur10e(ctx: ProgramContext) -> None:
        controller = await ctx.cell.controller(
            VIRTUAL_CONTROLLER if args.virtual else args.controller
        )
        async with controller[0] as mg:
            await run(mg, args)

    return isaaclab_reach_ur10e


async def run(mg: MotionGroup, args: argparse.Namespace) -> None:
    client = LeappPolicyClient(args.export)

    target: Pose | PoseSource = Pose(tuple(args.target))
    if args.isaac_sim:
        target = OmniverseScene(args.isaac_sim).prim(args.prim, relative_to=args.relative_to)
    schema = build_schema(mg, target)

    if args.execute and not args.virtual:
        answer = await asyncio.to_thread(
            input,
            f"This moves the real robot {args.controller} to the start pose and runs the "
            "policy. Type 'yes': ",
        )
        if answer.strip().lower() != "yes":
            print("Aborted, nothing moved.")
            return

    if not args.execute:
        if args.virtual:  # a fresh virtual robot starts elsewhere: show the trained start
            await mg.plan_and_execute([jnt(START_JOINTS)], tcp=TCP)
        await dry_run(mg, client, schema)
        return

    executor = PolicyExecutor(
        schema,
        client,
        execution=ContinuousExecution(rate_hz=client.control_freq),
        n_action_steps=0,  # one step per inference, replaced by the next one
        start_joint_position={mg: START_JOINTS},
        stop_conditions=[target_reached(target)],
        timeout_s=TIMEOUT_S,
    )
    try:
        result = await executor.run()
    except (MotionError, EmergencyStopError) as exc:  # NOVA stopped the robot
        print(f"Stopped by the controller: {exc}")
        raise
    state = await mg.get_state(TCP)
    print(f"{result.reason}; flange at {[round(v, 1) for v in state.pose.to_tuple()]}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--export", type=Path, default=EXPORT, help="the LEAPP export's .yaml")
    robot = parser.add_mutually_exclusive_group(required=True)
    robot.add_argument("--controller", help="an existing NOVA controller (asks before moving it)")
    robot.add_argument("--virtual", action="store_true", help="a throwaway virtual UR10e")
    parser.add_argument("--execute", action="store_true", help="move the robot")
    parser.add_argument(
        "--target", type=float, nargs=6, default=DEFAULT_TARGET, metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
        help="target flange pose: mm and rotation vector (rad)",
    )  # fmt: skip
    parser.add_argument(
        "--isaac-sim",
        default=os.environ.get("ISAAC_SIM_API"),
        help="Isaac Sim extension API, e.g. http://isaac-host:8011/omniservice/api/v2",
    )
    parser.add_argument("--prim", default="/World/Target", help="prim whose pose is the target")
    parser.add_argument(
        "--relative-to", default=None, help="express the prim relative to this prim (robot base)"
    )
    runner = run_program(make_program(parser.parse_args()))
    if runner.program_run.state == api.models.ProgramRunState.FAILED:
        sys.exit(1)


if __name__ == "__main__":
    main()
