from __future__ import annotations

import asyncio
import logging
from typing import TYPE_CHECKING, Protocol, cast

from nova import Nova

from motion_group_extensions import PolicyExecutionOptions, enable_motion_group_policy_extension

if TYPE_CHECKING:
    from collections.abc import AsyncIterator

    from nova.cell.motion_group import MotionGroup

    from motion_group_extensions import PolicyRunState

JsonPrimitive = str | int | float | bool | None
JsonValue = JsonPrimitive | dict[str, "JsonValue"] | list["JsonValue"]

POLICY_API_URL = "https://spjhrikg.instance.wandelbots.io/cell/policy-service"
CONTROLLER_NAME = "ur10e"
STOP_AFTER_SECONDS = 1.5

logger = logging.getLogger(__name__)


enable_motion_group_policy_extension()


class _HasModelDump(Protocol):
    def model_dump(self, *, mode: str, exclude_none: bool) -> dict[str, JsonValue]: ...


class _SupportsPolicy(Protocol):
    async def execute_policy(
        self,
        policy_path: str,
        task: str,
        timeout_s: float,
        *,
        options: PolicyExecutionOptions | None = None,
    ) -> PolicyRunState: ...

    def stream_policy(
        self,
        policy_path: str,
        task: str,
        timeout_s: float,
        *,
        options: PolicyExecutionOptions | None = None,
    ) -> AsyncIterator[PolicyRunState]: ...


async def main() -> None:
    async with Nova() as nova:
        controller = await nova.cell().controller(CONTROLLER_NAME)
        motion_group: MotionGroup = controller[0]
        policy_motion_group = cast("_SupportsPolicy", motion_group)

        tcp_names = await motion_group.tcp_names()
        tcp = tcp_names[0] if tcp_names else "flange"
        motion_group_setup = await motion_group.get_setup(tcp)

        logger.info("[1] execute_policy() until terminal state")
        result = await policy_motion_group.execute_policy(
            policy_path="demo-policy-ur10e",
            task="pick the cube and place it in the box",
            timeout_s=5.0,
            options=PolicyExecutionOptions(
                n_action_steps=10,
                device="cpu",
                cameras={
                    "flange": {
                        "type": "webrtc",
                        "api_url": "http://camera-server",
                        "device_id": "cam1",
                        "width": 640,
                        "height": 480,
                        "fps": 30,
                    }
                },
                use_gripper=True,
                gripper_io_key="digital_out[0]",
                motion_group_setup=cast("_HasModelDump", motion_group_setup),
                policy_api_url=POLICY_API_URL,
            ),
        )
        logger.info("  final state: %s", result.state)
        logger.info("  final metadata: %s", result.metadata)

        logger.info("[2] stream_policy() with external stop condition")
        async for state in policy_motion_group.stream_policy(
            policy_path="demo-policy-ur10e-stream",
            task="pick the cube and place it in the box",
            timeout_s=30.0,
            options=PolicyExecutionOptions(
                n_action_steps=10,
                policy_api_url=POLICY_API_URL,
            ),
        ):
            logger.info("  state=%s elapsed_s=%s metadata=%s", state.state, state.elapsed_s, state.metadata)
            if state.elapsed_s is not None and state.elapsed_s > STOP_AFTER_SECONDS:
                await state.stop()
                logger.info("  stop requested from client")
                break


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    asyncio.run(main())
