"""
This example demonstrates a handover between four different robots brands.
Each robot picks up a cube and passes it to the next robot in the sequence.

All robots are operating in the world coordinate system.
"""

import asyncio
from dataclasses import dataclass
from math import pi

import numpy as np
from wandelbots_api_client.models import (
    CoordinateSystem,
    RotationAngles,
    RotationAngleTypes,
    Vector3d,
)

from nova import Controller, Nova
from nova.actions import Action, jnt
from nova.actions.motions import ptp
from nova.api import models
from nova.core.exceptions import PlanTrajectoryFailed
from nova_rerun_bridge import NovaRerunBridge
from nova_rerun_bridge.trajectory import TimingMode


@dataclass
class RobotPosition:
    """Represents a robot's position and associated poses"""

    mounting: Vector3d
    rotation: RotationAngles  # Added rotation field
    cube_position: tuple[float, float, float, float, float, float]
    handover_position: tuple[float, float, float, float, float, float]
    home_position: tuple[float, ...]
    motion_group_id: int


ROBOT_POSITIONS = {
    "FANUC": RobotPosition(
        mounting=Vector3d(x=600, y=0, z=350),
        rotation=RotationAngles(
            angles=[0, 0, 0],  # Facing +X
            type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ,
        ),
        cube_position=(1000, 0, 100, pi, 0, 0),  # TCP down
        # Halfway between FANUC and KUKA
        handover_position=(300, 300, 400, pi, 0, 0),  # TCP down for handover
        home_position=(0.0, 0.0, 0.0, 0.0, -pi / 2, 0.0),
        motion_group_id=1,
    ),
    "KUKA": RobotPosition(
        mounting=Vector3d(x=0, y=600, z=0),
        rotation=RotationAngles(
            angles=[0, 0, pi / 2],  # Facing +Y
            type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ,
        ),
        cube_position=(0, 1000, 100, pi, 0, 0),  # TCP down
        # Halfway between KUKA and YASKAWA
        handover_position=(-300, 300, 400, pi, 0, 0),  # TCP down for handover
        home_position=(0.0, -pi / 2, pi / 2, 0.0, pi / 2, 0.0),
        motion_group_id=0,
    ),
    "YASKAWA": RobotPosition(
        mounting=Vector3d(x=-600, y=0, z=350),
        rotation=RotationAngles(
            angles=[0, 0, pi],  # Facing -X
            type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ,
        ),
        cube_position=(-1000, 0, 100, pi, 0, 0),  # TCP down
        # Halfway between YASKAWA and ABB
        handover_position=(-300, -300, 400, pi, 0, 0),  # TCP down for handover
        home_position=(0.0, 0.0, 0.0, 0.0, -pi / 2, 0.0),
        motion_group_id=0,
    ),
    "ABB": RobotPosition(
        mounting=Vector3d(x=0, y=-600, z=0),
        rotation=RotationAngles(
            angles=[0, 0, -pi / 2],  # Facing -Y
            type=RotationAngleTypes.EULER_ANGLES_EXTRINSIC_XYZ,
        ),
        cube_position=(0, -1000, 100, pi, 0, 0),  # TCP down
        # Halfway between ABB and FANUC
        handover_position=(300, -300, 400, pi, 0, 0),  # TCP down for handover
        home_position=(0.0, 0.0, 0.0, 0.0, pi / 2, 0.0),
        motion_group_id=0,
    ),
}


async def pick_and_pass_cube(
    controller: Controller,
    bridge: NovaRerunBridge,
    pos_config: RobotPosition,
    action: str,
    motion_group_id: int = 0,
    sync: bool = False,
):
    """Handle cube picking and passing for a single robot"""
    async with controller[motion_group_id] as motion_group:
        await bridge.log_saftey_zones(motion_group)
        tcp = (await motion_group.tcp_names())[0]

        # Calculate handover position with orientation
        handover_pos = list(pos_config.handover_position)
        if action in ["pickup_and_handover", "go_to_handover"]:
            handover_pos[3:] = calculate_handover_orientation(
                pos_config.mounting, tuple(handover_pos), is_receiver=(action == "go_to_handover")
            )
        handover_target: tuple[float, float, float, float, float, float] = (
            handover_pos[0],
            handover_pos[1],
            handover_pos[2],
            handover_pos[3],
            handover_pos[4],
            handover_pos[5],
        )

        # Define actions based on robot state
        actions_map: dict[str, list[Action]] = {
            "pickup_and_handover": [ptp(pos_config.cube_position), ptp(handover_target)],
            "go_to_handover": [ptp(handover_target)],
            "go_home_with_cube": [jnt(pos_config.home_position)],
            "place_cube": [ptp(pos_config.cube_position), jnt(pos_config.home_position)],
        }

        try:
            joint_trajectory = await motion_group.plan(actions_map[action], tcp)
            await bridge.log_actions(actions_map[action])
            await bridge.log_trajectory(
                joint_trajectory,
                tcp,
                motion_group,
                timing_mode=TimingMode.SYNC if sync else TimingMode.CONTINUE,
            )
            await motion_group.execute(joint_trajectory, tcp, actions_map[action])
        except PlanTrajectoryFailed as e:
            await bridge.log_actions(actions_map[action], show_connection=True)
            if e.error.joint_trajectory:
                await bridge.log_trajectory(e.error.joint_trajectory, tcp, motion_group)
            await bridge.log_error_feedback(e.error.error_feedback)


async def move_to_initial_positions(
    robots: dict[str, Controller], bridge: NovaRerunBridge, positions: dict[str, RobotPosition]
) -> None:
    """Move all robots to their initial pickup positions"""
    tasks = []
    for robot_name, pos_config in positions.items():

        async def move_robot(robot_name=robot_name, pos_config=pos_config):
            async with robots[robot_name][pos_config.motion_group_id] as motion_group:
                tcp_names = await motion_group.tcp_names()
                tcp = tcp_names[0]

                # Move to home, then to cube position
                actions: list[Action] = [jnt(pos_config.home_position)]

                try:
                    joint_trajectory = await motion_group.plan(actions, tcp)
                    await bridge.log_actions(actions)
                    await bridge.log_trajectory(
                        joint_trajectory, tcp, motion_group, timing_mode=TimingMode.SYNC
                    )
                    await motion_group.execute(joint_trajectory, tcp, actions=actions)
                except PlanTrajectoryFailed as e:
                    await bridge.log_actions(actions, show_connection=True)
                    if e.error.joint_trajectory:
                        await bridge.log_trajectory(e.error.joint_trajectory, tcp, motion_group)
                    await bridge.log_error_feedback(e.error.error_feedback)

        tasks.append(move_robot())

    await asyncio.gather(*tasks)


def calculate_handover_orientation(
    base_pos: Vector3d, handover_pos: tuple[float, ...], is_receiver: bool = False
) -> tuple[float, float, float]:
    """Calculate TCP orientation for handover using axis-angle representation"""
    direction = np.array(
        [
            handover_pos[0] - base_pos.x,
            handover_pos[1] - base_pos.y,
            0,  # Ignore Z for horizontal orientation
        ]
    )
    direction = direction / np.linalg.norm(direction)
    if not is_receiver:
        direction = -direction

    # Cross product with downward vector to get rotation axis
    rotation_axis = np.cross([0, 0, -1], direction)
    angle = np.arccos(-direction[1])  # Angle from downward

    return (
        float(rotation_axis[0] * angle),
        float(rotation_axis[1] * angle),
        float(rotation_axis[2] * angle),
    )


async def main():
    async with Nova() as nova, NovaRerunBridge(nova) as bridge:
        cell = nova.cell()

        # Initialize robots
        fanuc = await cell.ensure_virtual_robot_controller(
            "fanuc",
            models.VirtualControllerTypes.FANUC_MINUS_LR_MATE_200I_D7_L,
            models.Manufacturer.FANUC,
        )
        kuka = await cell.ensure_virtual_robot_controller(
            "kuka", models.VirtualControllerTypes.KUKA_MINUS_KR6_R700_2, models.Manufacturer.KUKA
        )
        abb = await cell.ensure_virtual_robot_controller(
            "abb", models.VirtualControllerTypes.ABB_MINUS_IRB1200_7, models.Manufacturer.ABB
        )
        yaskawa = await cell.ensure_virtual_robot_controller(
            "yaskawa", models.VirtualControllerTypes.YASKAWA_MINUS_GP7, models.Manufacturer.YASKAWA
        )

        # Set robot mountings
        for robot, pos in [
            (fanuc, ROBOT_POSITIONS["FANUC"]),
            (kuka, ROBOT_POSITIONS["KUKA"]),
            (abb, ROBOT_POSITIONS["ABB"]),
            (yaskawa, ROBOT_POSITIONS["YASKAWA"]),
        ]:
            await nova._api_client.virtual_robot_setup_api.set_virtual_robot_mounting(
                cell="cell",
                controller=robot.controller_id,
                id=pos.motion_group_id,
                coordinate_system=CoordinateSystem(
                    coordinate_system="world",
                    name="mounting",
                    reference_uid="",
                    position=pos.mounting,
                    rotation=pos.rotation,  # Use the robot-specific rotation
                ),
            )

        await asyncio.sleep(3)  # Wait for setup
        await bridge.setup_blueprint()

        robots = {"FANUC": fanuc, "KUKA": kuka, "ABB": abb, "YASKAWA": yaskawa}
        robot_sequence = ["FANUC", "KUKA", "YASKAWA", "ABB", "FANUC"]  # Complete circle
        await move_to_initial_positions(robots, bridge, ROBOT_POSITIONS)
        bridge.continue_after_sync()

        for i in range(len(robot_sequence) - 1):
            current_robot = robots[robot_sequence[i]]
            next_robot = robots[robot_sequence[i + 1]]
            current_pos = ROBOT_POSITIONS[robot_sequence[i]]
            next_pos = ROBOT_POSITIONS[robot_sequence[i + 1]]

            # Step 1: First robot picks up cube and moves to handover
            await asyncio.gather(
                pick_and_pass_cube(
                    current_robot,
                    bridge,
                    current_pos,
                    "pickup_and_handover",
                    current_pos.motion_group_id,
                    sync=True,
                ),
                # Step 2: Next robot moves to handover position
                pick_and_pass_cube(
                    next_robot,
                    bridge,
                    current_pos,  # Use current robot's handover position
                    "go_to_handover",
                    next_pos.motion_group_id,
                    sync=True,
                ),
            )

            bridge.continue_after_sync()

            # Step 3: Both robots move to home (after handover)
            await asyncio.gather(
                pick_and_pass_cube(
                    current_robot,
                    bridge,
                    current_pos,
                    "go_home_with_cube",
                    current_pos.motion_group_id,
                    sync=True,
                ),
                pick_and_pass_cube(
                    next_robot,
                    bridge,
                    next_pos,
                    "go_home_with_cube",
                    next_pos.motion_group_id,
                    sync=True,
                ),
            )

            # Step 4: Next robot places cube and returns home
            await pick_and_pass_cube(
                next_robot, bridge, next_pos, "place_cube", next_pos.motion_group_id
            )

            await asyncio.sleep(1)  # Small delay between transfers


if __name__ == "__main__":
    asyncio.run(main())
