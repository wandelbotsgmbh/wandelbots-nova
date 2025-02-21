import asyncio
import logging

import requests
import wandelbots_api_client as wb
from pydantic import BaseModel, Field
from wandelbots_api_client.models.all_joint_positions_request import AllJointPositionsRequest
from wandelbots_api_client.models.all_joint_positions_response import AllJointPositionsResponse

from nova.api import models
from nova_rerun_bridge.benchmark.benchmark_base import BenchmarkStrategy, run_benchmark

logger = logging.getLogger(__name__)


class CollisionFreeP2PRequest(BaseModel):
    """Request model for collision-free point-to-point movement planning."""

    plan: wb.models.PlanCollisionFreePTPRequest = Field(
        ..., description="NOVA plan request for collision-free p2p movement"
    )
    n_control_points: int = Field(
        default=200, description="Number of control points for the trajectory"
    )
    n_eval_points: int = Field(
        default=1000, description="Number of evaluation points for the trajectory"
    )
    n_iterations: int = Field(
        default=10_000, description="Number of iterations for the optimization"
    )
    n_logging_interval: int = Field(default=250, description="Number of iterations between logs")
    collision_margin: int = Field(
        default=10, description="Collision margin between colliders in mm"
    )


class CollisionFreeMagmaStrategy(BenchmarkStrategy):
    """Strategy for collision-free motion planning using Magma P2P."""

    name = "collision_free_magma"

    async def _get_valid_configurations(
        self, nova, motion_group, target, tcp
    ) -> list[AllJointPositionsResponse] | None:
        """Calculate valid inverse kinematic configurations."""
        try:
            response = (
                await nova._api_client.motion_group_kinematic_api.calculate_all_inverse_kinematic(
                    cell=nova.cell().cell_id,
                    motion_group=motion_group.motion_group_id,
                    all_joint_positions_request=AllJointPositionsRequest(
                        motion_group=motion_group.motion_group_id,
                        tcp_pose=models.TcpPose(
                            position=models.Vector3d(
                                x=target.position[0], y=target.position[1], z=target.position[2]
                            ),
                            orientation=models.Vector3d(
                                x=target.orientation[0],
                                y=target.orientation[1],
                                z=target.orientation[2],
                            ),
                            tcp=tcp,
                        ),
                    ),
                )
            )
            return response.joint_positions
        except (wb.ApiException, requests.RequestException, ValueError, Exception) as e:
            logger.exception("Failed to calculate inverse kinematics: %s", e)
            return None

    async def plan(
        self,
        motion_group,
        target,
        collision_scene,
        tcp,
        optimizer_setup,
        nova,
        start_joint_position,
    ):
        """Plan a collision-free trajectory."""
        valid_configs = await self._get_valid_configurations(nova, motion_group, target, tcp)
        if not valid_configs:
            raise ValueError("No valid configurations found")

        collision_motion_group = None
        if collision_scene and collision_scene.colliders:
            if (
                collision_scene.motion_groups
                and optimizer_setup.motion_group_type in collision_scene.motion_groups
            ):
                collision_motion_group = collision_scene.motion_groups[
                    optimizer_setup.motion_group_type
                ]

        headers = {"Accept": "application/json", "Content-Type": "application/json"}

        for config in valid_configs:
            try:
                plan_request = wb.models.PlanCollisionFreePTPRequest(
                    robot_setup=optimizer_setup.model_dump(),
                    start_joint_position=start_joint_position,
                    static_colliders=collision_scene.colliders,
                    collision_motion_group=collision_motion_group,
                    target=wb.models.PlanCollisionFreePTPRequestTarget(config.joints),
                )

                request = CollisionFreeP2PRequest(
                    plan=plan_request,
                    n_control_points=50,
                    n_eval_points=200,
                    n_iterations=2000,
                    n_logging_interval=25,
                    collision_margin=5,
                )

                response = requests.post(
                    f"{nova._api_client._host}/{nova.cell().cell_id}/magma-p2p/planCollisionFreeP2P",
                    headers=headers,
                    json=request.model_dump(mode="json"),
                )

                if response.status_code == 200:
                    return wb.models.PlanTrajectoryResponse.model_validate(
                        response.json()
                    ).response.actual_instance

                logger.error(f"Planning failed with status {response.status_code}: {response.text}")
            except (wb.ApiException, requests.RequestException, ValueError, Exception) as e:
                logger.exception("Error during planning: %s", e)
                continue

        raise RuntimeError("Failed to plan trajectory for all configurations")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    asyncio.run(run_benchmark(CollisionFreeMagmaStrategy()))
