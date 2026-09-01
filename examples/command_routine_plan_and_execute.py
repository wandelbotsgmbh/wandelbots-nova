"""
Example: Load a teaching dataset's CommandRoutine and plan + execute it as a trajectory.

CommandRoutine is a data-based alternative to `Action` for describing a motion and signal
sequence (see nova/command_routines/ and tests/command_routines/). `MotionGroup.plan`,
`execute` and `plan_and_execute` all accept a CommandRoutine directly wherever they accept
actions -- it is converted under the hood into the same Action list a hand-written program
would use (see nova/command_routines/planning.py for which parts of a routine convert today).

command_routine_pick_and_place.json holds a whole teaching dataset (the same shape the
teaching API returns: `api.models.GetDatasetResponse`), not just a bare routine:
  - a "pick-place-station" coordinate system, offset from world
  - five poses taught within that coordinate system (ready, approach-pick, pick, via-sweep,
    place)
  - a "pick-and-place-demo" CommandRoutine whose motion commands reference those poses by id
    (`LocalPoseReference`), mixing joint, Cartesian PTP, linear and circular motions with
    varying velocities and blending settings:
      1. joint PTP to a safe retract position (raw joints, no dataset pose)
      2. Cartesian PTP to the ready pose
      3. Cartesian PTP approach, fast with auto-blending
      4. a straight line descent to the pick pose, slow with position-zone blending
      5. close the gripper, wait
      6. a straight line retract back to the approach pose
      7. a circular sweep (via an intermediate pose) to the place pose, medium speed
      8. open the gripper, wait
      9. Cartesian PTP back to the ready pose

A dataset can hold several command routines; `get_command_routine` picks out the one with
the given id (`COMMAND_ROUTINE_ID` below) rather than assuming there is only one.

A routine's `LocalPoseReference` targets can't be resolved on their own -- they're only
meaningful together with the dataset's coordinate systems and poses. `resolve_dataset_poses`
walks the coordinate-system chains once and returns a `pose_id -> Pose` mapping, which is
passed to `plan`/`execute` as `pose_resolver`.

The routine itself does not set a `tcp`, so a TCP still has to be supplied when actually
loading a trajectory onto a controller for execution -- the program below fetches one from
the controller and passes it in explicitly. The Cartesian poses are illustrative; adjust them
if they fall outside your cell's reachable workspace.

Prerequisites:
- Create an NOVA instance
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
"""

from pathlib import Path

import nova
from nova import api, run_program, viewers
from nova.cell import virtual_controller
from nova.command_routines import get_command_routine, resolve_dataset_poses

_HERE = Path(__file__).parent
COMMAND_ROUTINE_ID = "pick-and-place-demo"


def load_pick_and_place_dataset() -> api.models.GetDatasetResponse:
    path = _HERE / "command_routine_pick_and_place.json"
    return api.models.GetDatasetResponse.model_validate_json(path.read_text())


@nova.program(
    name="Command Routine Plan and Execute",
    viewer=viewers.Rerun(),
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10e",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
            )
        ],
        # dataset=ds.local_dataset("examples/command_routine_pick_and_place.json"),
        cleanup_controllers=False,
    ),
)
async def command_routine_plan_and_execute(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    controller = await cell.controller("ur10e")
    motion_group = controller[0]
    tcp_names = await motion_group.tcp_names()
    tcp = tcp_names[0]

    dataset = load_pick_and_place_dataset()
    poses = resolve_dataset_poses(dataset)
    routine = get_command_routine(dataset, COMMAND_ROUTINE_ID)

    # `plan`, `execute` and `plan_and_execute` all accept the CommandRoutine directly.
    # `pose_resolver` resolves the routine's `LocalPoseReference` targets against the
    # dataset's poses. The routine doesn't set `start_joint_position`, so plan() uses the
    # robot's live joints as the trajectory's start.
    joint_trajectory = await motion_group.plan(routine, tcp=tcp, pose_resolver=poses)
    print(f"Planned {len(joint_trajectory.joint_positions)} joint positions")

    await motion_group.execute(joint_trajectory, tcp, actions=routine, pose_resolver=poses)
    print("Execution finished")


if __name__ == "__main__":
    run_program(command_routine_plan_and_execute)
