import asyncio

import numpy as np
import rerun as rr
import trimesh

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
                type=api.models.VirtualControllerTypes.UNIVERSALROBOTS_UR10E,
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
        transform[:3, 3] = list(mesh_pose.position)
        scene.apply_transform(transform)

        N = 20000
        points, face_index = trimesh.sample.sample_surface_even(scene, count=N, radius=1)
        normals = scene.face_normals[face_index]

        # Connect to the controller and activate motion groups
        async with controller[0] as motion_group:
            tcp = "Flange"

            # Build all poses first
            poses = []
            for point, normal in zip(points, normals, strict=False):
                # Convert normal to rotation vector
                rotation = np.cross([0, 0, 1], normal)  # cross product with Z axis
                if np.any(rotation):  # if not parallel to Z
                    angle = np.arccos(normal[2])  # angle with Z axis
                    rotation = rotation / np.linalg.norm(rotation) * angle
                else:
                    rotation = np.zeros(3)  # no rotation needed

                poses.append(
                    Pose((point[0], point[1], point[2], rotation[0], rotation[1], rotation[2]))
                )

            # Batch call inverse kinematics for all poses at once
            joint_solutions = await motion_group._inverse_kinematics(poses=poses, tcp=tcp)

            # Process results and assign colors
            colors = []
            for solutions in joint_solutions:
                valid_configs = len(solutions)
                # Red if unreachable, green gradient based on number of configurations
                if valid_configs == 0:
                    colors.append([1.0, 0.0, 0.0, 1.0])  # Red
                else:
                    intensity = valid_configs / 8.0  # Normalize to [0,1]
                    colors.append([0.0, intensity, 0.0, 1.0])  # Green gradient

            home = await motion_group.tcp_pose(tcp)

            actions = [cartesian_ptp(home)]

            for action in actions:
                action.settings = MotionSettings(tcp_velocity_limit=200)

            await motion_group.plan(actions, tcp)

            # Log points with colors
            rr.log(
                "motion/reachability",
                rr.Points3D(positions=points, colors=np.array(colors), radii=3),
            )
            log_mesh_to_rerun(scene)


if __name__ == "__main__":
    asyncio.run(test())
