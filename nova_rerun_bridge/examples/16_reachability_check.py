import asyncio

import numpy as np
import rerun as rr
import trimesh
from wandelbots_api_client.models.all_joint_positions_request import AllJointPositionsRequest
from wandelbots_api_client.models.all_joint_positions_response import AllJointPositionsResponse

from nova import MotionSettings
from nova.actions import cartesian_ptp
from nova.api import models
from nova.core.exceptions import PlanTrajectoryFailed
from nova.core.nova import Nova
from nova.types import Pose
from nova_rerun_bridge import NovaRerunBridge


def log_mesh_to_rerun(scene: trimesh.Trimesh) -> None:
    """Log mesh to rerun visualization."""
    vertices = scene.vertices
    faces = scene.faces
    vertex_normals = scene.vertex_normals
    vertex_colors = np.ones((len(vertices), 3), dtype=np.float32)

    rr.log(
        "motion/welding_benchmark",
        rr.Mesh3D(
            vertex_positions=vertices,
            triangle_indices=faces,
            vertex_normals=vertex_normals,
            albedo_factor=vertex_colors,
        ),
        static=True,
    )


async def test():
    async with Nova() as nova, NovaRerunBridge(nova) as bridge:
        await bridge.setup_blueprint()

        cell = nova.cell()
        controller = await cell.ensure_virtual_robot_controller(
            "ur10",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )

        scene = trimesh.load_mesh(
            "nova_rerun_bridge/example_data/Welding_Benchmark_USA_01.stl", file_type="stl"
        )

        # Define position for the welding part
        mesh_pose = Pose((1100, 0, 0, 0, 0, 0))  # in front of robot, on floor

        # Create transformation matrix from Pose2
        transform = np.eye(4)
        transform[:3, 3] = np.array(mesh_pose.position, dtype=np.float64)
        scene.apply_transform(transform)

        N = 200
        points, face_index = trimesh.sample.sample_surface_even(scene, count=N, radius=1)
        normals = scene.face_normals[face_index]

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            await bridge.log_saftey_zones(motion_group)

            tcp = "Flange"

            # Store points and their configurations count
            tested_points = []
            colors = []

            for point, normal in zip(points, normals, strict=False):
                # Convert normal to rotation vector
                rotation = np.cross([0, 0, 1], normal)  # cross product with Z axis
                if np.any(rotation):  # if not parallel to Z
                    angle = np.arccos(normal[2])  # angle with Z axis
                    rotation = rotation / np.linalg.norm(rotation) * angle
                else:
                    rotation = np.zeros(3)  # no rotation needed

                tested_points.append(point)
                try:
                    response: AllJointPositionsResponse = await nova._api_client.motion_group_kinematic_api.calculate_all_inverse_kinematic(
                        cell=cell.cell_id,
                        motion_group=motion_group.motion_group_id,
                        all_joint_positions_request=AllJointPositionsRequest(
                            motion_group=motion_group.motion_group_id,
                            tcp_pose=models.TcpPose(
                                position=models.Vector3d(x=point[0], y=point[1], z=point[2]),
                                orientation=models.Vector3d(
                                    x=rotation[0], y=rotation[1], z=rotation[2]
                                ),
                                tcp=tcp,
                            ),
                        ),
                    )
                    valid_configs = len(response.joint_positions)
                except Exception:
                    valid_configs = 0

                # Red if unreachable, green gradient based on number of configurations
                if valid_configs == 0:
                    colors.append([1.0, 0.0, 0.0, 1.0])  # Red
                else:
                    intensity = valid_configs / 8.0  # Normalize to [0,1]
                    colors.append([0.0, intensity, 0.0, 1.0])  # Green gradient

            # Log points with colors
            rr.log(
                "motion/reachability", rr.Points3D(tested_points, colors=np.array(colors), radii=3)
            )
            log_mesh_to_rerun(scene)

            home = await motion_group.tcp_pose(tcp)

            actions = [cartesian_ptp(home)]

            for action in actions:
                action.settings = MotionSettings(tcp_velocity_limit=200)

            try:
                joint_trajectory = await motion_group.plan(actions, tcp)
                await bridge.log_actions(actions)
                await bridge.log_trajectory(joint_trajectory, tcp, motion_group)
            except PlanTrajectoryFailed as e:
                await bridge.log_actions(actions)
                await bridge.log_trajectory(e.error.joint_trajectory, tcp, motion_group)
                await bridge.log_error_feedback(e.error.error_feedback)
                return


if __name__ == "__main__":
    asyncio.run(test())
