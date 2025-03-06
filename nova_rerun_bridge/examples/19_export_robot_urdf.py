import asyncio
import os
from pathlib import Path

import rerun as rr

from nova.api import models
from nova.core.nova import Nova
from nova.urdf_exporter import export_urdf_from_optimizer_setup
from nova_rerun_bridge import NovaRerunBridge

"""
Example showing how to export the current robot model to URDF and SRDF formats.
This is useful for visualization in external tools or for motion planning.
"""


async def export_current_robot():
    async with Nova() as nova, NovaRerunBridge(nova) as bridge:
        # Setup cell and controller
        cell = nova.cell()
        controller = await cell.ensure_virtual_robot_controller(
            "ur",
            models.VirtualControllerTypes.UNIVERSALROBOTS_MINUS_UR10E,
            models.Manufacturer.UNIVERSALROBOTS,
        )

        # Connect to the controller and get robot configuration
        async with controller[0] as motion_group:
            motion_groups = await nova._api_client.motion_group_api.list_motion_groups(
                nova.cell()._cell_id
            )
            motion_motion_group = next(
                (
                    mg
                    for mg in motion_groups.instances
                    if mg.motion_group == motion_group.motion_group_id
                ),
                None,
            )

            # Get tcp and optimizer setup
            tcp = "Flange"
            robot_setup = await motion_group._get_optimizer_setup(tcp=tcp)

            # Define export directory
            export_dir = os.path.join(os.path.expanduser("~"), "robot_exports")
            os.makedirs(export_dir, exist_ok=True)

            print(f"Exporting robot model to: {export_dir}")

            # Export URDF and SRDF files - model path will be resolved automatically
            urdf_path, srdf_path = export_urdf_from_optimizer_setup(
                optimizer_setup=robot_setup,
                model_name=motion_motion_group.model_from_controller,
                export_path=export_dir,
                export_srdf=True,
            )

            print(f"URDF exported to: {urdf_path}")
            print(f"SRDF exported to: {srdf_path}")

            # Log paths in rerun as text
            rr.log("exported_files/urdf", rr.TextDocument(f"URDF file: {Path(urdf_path).name}"))
            rr.log("exported_files/srdf", rr.TextDocument(f"SRDF file: {Path(srdf_path).name}"))


if __name__ == "__main__":
    asyncio.run(export_current_robot())
