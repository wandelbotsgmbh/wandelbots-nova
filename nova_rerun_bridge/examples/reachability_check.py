import asyncio

import numpy as np
import rerun as rr
import trimesh
from wandelbots_api_client.models.all_joint_positions_request import AllJointPositionsRequest
from wandelbots_api_client.models.all_joint_positions_response import AllJointPositionsResponse

import nova
from nova import Nova, api
from nova.actions import cartesian_ptp
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import MotionSettings, Pose


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


@nova.program(
    name="reachability_check",
    viewer=nova.viewers.Rerun(application_id="reachability-check"),
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            )
        ],
        cleanup_controllers=False,
    ),
)
async def test():
    async with Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("ur10")

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
                            tcp_pose=api.models.TcpPose(
                                position=api.models.Vector3d(x=point[0], y=point[1], z=point[2]),
                                orientation=api.models.Vector3d(
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

            await motion_group.plan(actions, tcp)


if __name__ == "__main__":
    asyncio.run(test())
