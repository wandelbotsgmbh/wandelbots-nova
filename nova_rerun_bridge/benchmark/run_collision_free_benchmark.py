from nova.actions.motions import collision_free
from nova.core.motion_group import MotionGroup
from nova_rerun_bridge.benchmark.benchmark_base import BenchmarkStrategy, run_benchmark


class CollisionFreeStrategy(BenchmarkStrategy):
    name = "collision_free"

    async def plan(
        self,
        motion_group: MotionGroup,
        target,
        collision_scene,
        tcp,
        optimizer_setup,
        nova,
        start_joint_position,
    ):
        return await motion_group.plan(
            [collision_free(target=target, collision_scene=collision_scene)], tcp=tcp
        )


if __name__ == "__main__":
    import asyncio

    asyncio.run(run_benchmark(CollisionFreeStrategy()))
