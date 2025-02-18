from nova.actions.motions import collision_free
from nova_rerun_bridge.benchmark.benchmark_base import BenchmarkStrategy, run_benchmark


class CollisionFreeStrategy(BenchmarkStrategy):
    name = "collision_free"

    async def plan(self, motion_group, target, collision_scene, tcp):
        return await motion_group.plan(
            [collision_free(target=target, collision_scene=collision_scene)], tcp=tcp
        )


if __name__ == "__main__":
    import asyncio

    asyncio.run(run_benchmark(CollisionFreeStrategy()))
