from __future__ import annotations

import asyncio
import logging
import os
from typing import Protocol, cast

from nova import Nova
from nova.cell.motion_group import MotionGroup
from nova_policy import PolicyExecutionOptions, PolicyRunState, enable_motion_group_policy_extension

POLICY_SERVICE_URL = os.getenv(
    "POLICY_SERVICE_URL",
    "https://nova-policy-service.ai.gpucluster-dev.wandelbots.io",
)
POLICY_PATH = os.getenv(
    "POLICY_PATH",
    "StefanWagnerWandelbots/act_virtual_teleop_pickplace_easy",
)
CONTROLLER_NAME = os.getenv("NOVA_CONTROLLER", "ur10e")
MOTION_GROUP_INDEX = int(os.getenv("NOVA_MOTION_GROUP", "0"))
TASK = os.getenv("POLICY_TASK", "pick the cube and place it in the box")
TCP = os.getenv("NOVA_TCP")
MAX_OBSERVATIONS = int(os.getenv("MAX_OBSERVATIONS", "3"))
TIMEOUT_S = float(os.getenv("POLICY_TIMEOUT_S", "20"))
EXECUTE_ACTIONS = os.getenv("EXECUTE_ACTIONS", "false").lower() in {"1", "true", "yes"}
ALLOW_MOCK_IMAGES = os.getenv("ALLOW_MOCK_IMAGES", "true").lower() in {"1", "true", "yes"}
USE_GRIPPER = os.getenv("USE_GRIPPER", "true").lower() in {"1", "true", "yes"}

logger = logging.getLogger(__name__)


enable_motion_group_policy_extension()


class _HasModelDump(Protocol):
    def model_dump(self, *, mode: str, exclude_none: bool) -> dict[str, object]: ...


async def main() -> None:
    if EXECUTE_ACTIONS:
        logger.warning(
            "EXECUTE_ACTIONS=true will command NOVA jogging velocities. "
            "Use only against a virtual/safe controller with validated limits."
        )

    async with Nova() as nova:
        controller = await nova.cell().controller(CONTROLLER_NAME)
        motion_group: MotionGroup = controller[MOTION_GROUP_INDEX]
        tcp = TCP or await motion_group.active_tcp_name() or "flange"
        motion_group_setup = await motion_group.get_setup(tcp)

        options = PolicyExecutionOptions(
            tcp=tcp,
            policy_api_url=POLICY_SERVICE_URL,
            realtime=True,
            execute_actions=EXECUTE_ACTIONS,
            max_observations=MAX_OBSERVATIONS,
            low_water_mark=1,
            allow_mock_images=ALLOW_MOCK_IMAGES,
            use_gripper=USE_GRIPPER,
            motion_group_setup=cast("_HasModelDump", motion_group_setup),
            joint_velocity_limit=float(os.getenv("JOINT_VELOCITY_LIMIT", "0.25")),
            joint_position_gain=float(os.getenv("JOINT_POSITION_GAIN", "1.0")),
            joint_position_tolerance=float(os.getenv("JOINT_POSITION_TOLERANCE", "0.01")),
            setup_velocity_limit_scale=float(os.getenv("SETUP_VELOCITY_LIMIT_SCALE", "0.1")),
        )

        async for state in motion_group.stream_policy(
            policy_path=POLICY_PATH,
            task=TASK,
            timeout_s=TIMEOUT_S,
            options=options,
        ):
            _log_state(state)


def _log_state(state: PolicyRunState) -> None:
    realtime_metadata = (state.metadata or {}).get("realtime")
    logger.info(
        "run=%s state=%s elapsed=%s realtime=%s",
        state.run,
        state.state,
        state.elapsed_s,
        realtime_metadata,
    )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    asyncio.run(main())
