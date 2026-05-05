"""Integration test: Move both UR10e robots using PolicyRunner + MockActionSource.

Prerequisites:
- Set env variables (you can specify them in an .env file):
    - NOVA_API=<api>
    - NOVA_ACCESS_TOKEN=<token>
- Two UR10e controllers on the NOVA instance
"""

import asyncio
import logging

from nova import Nova
from policy import PolicyRunner
from policy.tests.mock_source import MockActionSource

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
logger = logging.getLogger(__name__)


async def main() -> None:
    async with Nova() as nova:
        cell = nova.cell()

        # Connect to both controllers
        ctrl1 = await cell.controller("ur10e")
        ctrl2 = await cell.controller("ur10e-2")
        mg1 = ctrl1[0]
        mg2 = ctrl2[0]

        home1 = list(await mg1.joints())
        home2 = list(await mg2.joints())
        logger.info("Home joints mg1: %s", [round(j, 3) for j in home1])
        logger.info("Home joints mg2: %s", [round(j, 3) for j in home2])

        # Create mock sources for each arm (each around its own home)
        source1 = MockActionSource(
            motion_group_ids=[mg1.id],
            num_joints=6,
            home_joints=home1,
            interval_ms=100,
            amplitude=0.05,
            frequency=0.3,
            max_steps=30,
        )
        source2 = MockActionSource(
            motion_group_ids=[mg2.id],
            num_joints=6,
            home_joints=home2,
            interval_ms=100,
            amplitude=0.05,
            frequency=0.3,
            max_steps=30,
        )

        # Run the policy
        runner = PolicyRunner(motion_groups=[mg1, mg2])

        logger.info("Starting PolicyRunner for both motion groups...")
        step = 0
        async with runner:
            async for chunk1 in source1:
                chunk2 = await source2.__anext__()
                from policy import ActionChunk

                merged = ActionChunk(joints={**chunk1.joints, **chunk2.joints})
                await runner.send(merged)
                step += 1

                if step % 10 == 0:
                    obs = await runner.observe()
                    for gid, state in obs.items():
                        logger.info(
                            "  [step %d] %s joints: %s",
                            step,
                            gid,
                            [round(j, 3) for j in state.joints[:3]],
                        )

        logger.info("PolicyRunner finished. Both robots should have moved gently.")

        # Verify final positions
        final1 = list(await mg1.joints())
        final2 = list(await mg2.joints())
        logger.info("Final joints mg1: %s", [round(j, 3) for j in final1])
        logger.info("Final joints mg2: %s", [round(j, 3) for j in final2])


if __name__ == "__main__":
    asyncio.run(main())
