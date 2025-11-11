import asyncio
import logging

import requests
from pydantic import BaseModel, Field

from nova import Nova, api
from nova_rerun_bridge.benchmark.benchmark_base import BenchmarkStrategy, run_benchmark

logger = logging.getLogger(__name__)


class CollisionFreeP2PRequest(BaseModel):
    """Request model for collision-free point-to-point movement planning."""

    plan: api.models.PlanCollisionFreeRequest = Field(
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
        self, nova: Nova, motion_group, target, tcp
    ) -> list[AllJointPositionsResponse] | None:
        """Calculate valid inverse kinematic configurations."""
        try:
            response = await nova._api_client.kinematics_api.inverse_kinematics(
                cell=nova.cell().cell_id,
                inverse_kinematics_request=api.models.InverseKinematicsRequest(
                    motion_group=motion_group.motion_group_id,
                    tcp_pose=api.models.Pose(
                        position=api.models.Vector3d(target.position.to_tuple()),
                        orientation=api.models.RotationVector(target.orientation.to_tuple()),
                    ),
                ),
            )

            # response = (
            #     await nova._api_client.motion_group_kinematic_api.calculate_all_inverse_kinematic(
            #         cell=nova.cell().cell_id,
            #         motion_group=motion_group.motion_group_id,
            #         all_joint_positions_request=api.models. AllJointPositionsRequest(
            #             motion_group=motion_group.motion_group_id,
            #             tcp_pose=api.models.Pose(
            #                 position=api.models.Vector3d(target.position.to_tuple()),
            #                 orientation=api.models.RotationVector(target.orientation.to_tuple()),
            #             ),
            #         ),
            #     )
            # )
            return response.joints
        except (api.ApiException, requests.RequestException, ValueError, Exception) as e:
            logger.exception("Failed to calculate inverse kinematics: %s", e)
            return None

    async def plan(
        self,
        motion_group,
        target,
        collision_scene,
        tcp,
        motion_group_setup,
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
                and motion_group_setup.motion_group_model in collision_scene.motion_groups
            ):
                collision_motion_group = collision_scene.motion_groups[
                    motion_group_setup.motion_group_model
                ]

        headers = {"Accept": "application/json", "Content-Type": "application/json"}

        for config in valid_configs:
            try:
                plan_request = api.models.PlanCollisionFreeRequest(
                    motion_group_setup=motion_group_setup,
                    start_joint_position=start_joint_position,
                    # static_colliders=collision_scene.colliders,
                    # collision_motion_group=collision_motion_group,
                    target=api.models.DoubleArray(list(config.joints)),
                    algorithm=api.models.CollisionFreeAlgorithm(api.models.RRTConnectAlgorithm()),
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
                    return api.models.PlanCollisionFreeResponse.model_validate(
                        response.json()
                    ).response.actual_instance

                logger.error(f"Planning failed with status {response.status_code}: {response.text}")
            except (api.ApiException, requests.RequestException, ValueError, Exception) as e:
                logger.exception("Error during planning: %s", e)
                continue

        raise RuntimeError("Failed to plan trajectory for all configurations")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    asyncio.run(run_benchmark(CollisionFreeMagmaStrategy()))
