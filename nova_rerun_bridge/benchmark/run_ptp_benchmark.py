from nova.actions.motions import ptp
from nova_rerun_bridge.benchmark.benchmark_base import BenchmarkStrategy, run_benchmark


class PtpStrategy(BenchmarkStrategy):
    name = "ptp"

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
        return await motion_group.plan(
            [ptp(target=target, collision_scene=collision_scene)], tcp=tcp
        )


if __name__ == "__main__":
    import asyncio

    asyncio.run(run_benchmark(PtpStrategy()))
