"""Single-arm joint jogging with 8-step chunks for smoother tracking."""

import math

import nova
from novapolicy import jog_joints

HOME = [1.169, -0.733, 1.745, -3.054, 0.872, 2.094]


@nova.program(
    id="jogging_single_joint_chunked",
    name="Single-Arm Joint Jogging (Chunked)",
    viewer=nova.viewers.Rerun(state_sample_interval_ms=1000.0 / 30.0),
)
async def main(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    mg = (await cell.controller("ur5e-left"))[0]

    duration = 5.0
    amplitude = 0.3
    frequency = 0.5
    ramp_s = 0.5
    settle_s = 0.5
    chunk_size = 8
    dt_ms = 33.0
    dt_s = dt_ms / 1000.0
    send_period_s = 2 * dt_s

    def smootherstep(value: float) -> float:
        x = min(1.0, max(0.0, value))
        return x**3 * (x * (x * 6 - 15) + 10)

    def target_at(t: float) -> list[float]:
        bounded_t = min(max(t, 0.0), duration)
        envelope = smootherstep(bounded_t / ramp_s) * smootherstep((duration - bounded_t) / ramp_s)
        target = list(HOME)
        target[0] += envelope * amplitude * math.sin(2 * math.pi * frequency * bounded_t)
        return target

    last_send_t = -math.inf
    async with jog_joints(mg, start_joint_position=HOME) as jogger:
        async for _ in jogger:
            t = jogger.elapsed
            if t >= duration + settle_s:
                break
            if t - last_send_t < send_period_s:
                continue
            chunk = [target_at(t + i * dt_s) for i in range(chunk_size)]
            jogger.set_target(chunk, dt_ms=dt_ms)
            last_send_t = t


if __name__ == "__main__":
    nova.run_program(main)
