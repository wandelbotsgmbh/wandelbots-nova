import pytest

from nova.actions.motions import CartesianPTP, JointPTP
from nova.cell.simulation import SimulatedRobot, naive_joints_to_pose
from wandelscript.datatypes import Pose


def test_simulated_robot():
    robot = SimulatedRobot(
        configuration=SimulatedRobot.Configuration(
            type="simulated_robot", id="simulated_robot", initial_pose=Pose((100, 0, 0, 0, 0, 0))
        )
    )
    assert robot.configuration.type == "simulated_robot"


@pytest.mark.asyncio
async def test_simulated_robot_execution():
    # 1. Create a SimulatedRobot instance with default config
    robot = SimulatedRobot()

    # 2. Define some actions for the robot to plan:
    #    a) Move joints directly to [0, 1, 2, 0, 0, 0] with JointPTP
    #    b) Then move to a cartesian Pose using naive IK with CartesianPTP
    joint_target = (0, 1, 2, 0, 0, 0)
    cartesian_target = Pose(
        (100, 200, 300, 10, 20, 30)
    )  # x=100mm, y=200mm, z=300mm, rx=10°, ry=20°, rz=30°

    actions = [JointPTP(target=joint_target), CartesianPTP(target=cartesian_target)]

    # 3. Plan the trajectory
    trajectory = await robot._plan(actions, tcp="Flange")

    assert len(trajectory.joint_positions) > 0, "Planned trajectory should not be empty."
    assert len(trajectory.times) == len(trajectory.joint_positions)
    assert len(trajectory.locations) == len(trajectory.joint_positions)

    # 4. Execute the planned trajectory
    #    We'll collect each MotionState from on_movement to verify final position.
    recorded_motion_states = []

    motion_iter = robot.stream_execute(
        joint_trajectory=trajectory, tcp="Flange", actions=actions, movement_controller=None
    )
    async for motion_state in motion_iter:
        recorded_motion_states.append(motion_state)

    # 5. Check that we actually moved through all steps
    assert len(recorded_motion_states) == len(trajectory.joint_positions), (
        f"Expected {len(trajectory.joint_positions)} motion states, got {len(recorded_motion_states)}."
    )

    # 6. Check the final state matches the last step of the trajectory
    final_state = await robot.get_state("Flange")
    final_joints = final_state.joints

    # The last action was CartesianPTP to cartesian_target. Because of naive "IK",
    # let's see if the final_joints produce a Pose close to cartesian_target.
    # We'll just do a simple check that naive_joints_to_pose is near the cartesian target.
    final_pose = naive_joints_to_pose(final_joints)

    # Some tolerance for floating‐point, e.g. ±1 mm, ±1 degree
    assert abs(final_pose.position.x - cartesian_target.position.x) < 1.0
    assert abs(final_pose.position.y - cartesian_target.position.y) < 1.0
    assert abs(final_pose.position.z - cartesian_target.position.z) < 1.0
    assert abs(final_pose.orientation.x - cartesian_target.orientation.x) < 1.0
    assert abs(final_pose.orientation.y - cartesian_target.orientation.y) < 1.0
    assert abs(final_pose.orientation.z - cartesian_target.orientation.z) < 1.0

    # Print some diagnostics
    print("Trajectory length:", len(trajectory.joint_positions))
    print("Final Joints:", final_joints)
    print("Final Pose (from naive FK):", final_pose)
