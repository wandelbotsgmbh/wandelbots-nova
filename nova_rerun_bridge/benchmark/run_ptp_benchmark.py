from nova.actions.motions import cartesian_ptp
from nova_rerun_bridge.benchmark.benchmark_base import BenchmarkStrategy, run_benchmark


class PtpStrategy(BenchmarkStrategy):
    name = "cartesian_ptp"

    async def plan(
        self,
        motion_group,
        target,
        collision_setup,
        tcp,
        motion_group_setup,
        nova,
        start_joint_position,
    ):
        return await motion_group.plan(
            [cartesian_ptp(target=target, collision_setup=collision_setup)], tcp=tcp
        )


if __name__ == "__main__":
    import asyncio

    asyncio.run(run_benchmark(PtpStrategy()))
