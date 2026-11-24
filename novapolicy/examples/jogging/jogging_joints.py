"""Joint jogging: joint 0 oscillates for 5 s, as live targets or as chunks, on one arm or two.

    python jogging_joints.py                    # one arm, live targets
    python jogging_joints.py --chunked          # 8-step chunks
    python jogging_joints.py --dual [--chunked] # two arms, mirror-symmetric

Live targets (``set_target``) go through ``ServoHorizon``: a spring-damper follows the target and
its braking path goes out as a waypoint horizon, so the arm tracks the moving target smoothly, a
little behind it.

Chunks (``set_chunk``) carry their own horizon. Their motion is shaped by an amplitude envelope so
it starts and ends at rest; the last chunks look past the end and hold the envelope's end value,
and leaving the context manager waits for the waypoints already sent to run out.
"""

import math
import sys

import nova
from novapolicy import ServoHorizon, default_pipeline, jog_joints

DUAL = "--dual" in sys.argv
CHUNKED = "--chunked" in sys.argv
HOMES = {
    "ur5e-left": [1.169, -0.733, 1.745, -3.054, 0.872, 2.094],
    "ur5e-right": [-1.169, -2.3911, -1.8675, 0.0, -0.872, -2.094],
}
DURATION_S, AMPLITUDE, FREQUENCY, RAMP_S = 5.0, 0.3, 0.5, 0.5
CHUNK_SIZE, DT_MS = 8, 33.0


def smootherstep(value: float) -> float:
    """0 -> 1 with zero velocity and acceleration at both ends."""
    x = min(1.0, max(0.0, value))
    return x**3 * (x * (x * 6 - 15) + 10)


def offset(t: float) -> float:
    """Joint 0's offset from home at ``t``; enveloped for chunks."""
    t = min(max(t, 0.0), DURATION_S)
    gain = smootherstep(t / RAMP_S) * smootherstep((DURATION_S - t) / RAMP_S) if CHUNKED else 1.0
    return gain * AMPLITUDE * math.sin(2 * math.pi * FREQUENCY * t)


@nova.program(id="jogging_joints", name="Joint Jogging", viewer=nova.viewers.Rerun())
async def main(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    names = list(HOMES) if DUAL else ["ur5e-left"]
    groups = [(await cell.controller(name))[0] for name in names]
    homes = {mg: HOMES[name] for mg, name in zip(groups, names, strict=True)}
    sign = {mg: 1 - 2 * i for i, mg in enumerate(groups)}  # the second arm mirrors the first

    def target(mg, t: float) -> list[float]:
        joints = list(homes[mg])
        joints[0] += sign[mg] * offset(t)
        return joints

    pipeline = None if CHUNKED else [ServoHorizon(), *default_pipeline()]
    dt_s = DT_MS / 1000.0
    last_send_t = -math.inf
    async with jog_joints(groups, start_joint_position=homes, pipeline=pipeline) as jogger:
        async for _ in jogger:
            t = jogger.elapsed
            if t >= DURATION_S:
                break
            if not CHUNKED:
                jogger.set_target({mg: target(mg, t) for mg in groups})
            elif t - last_send_t >= 2 * dt_s:  # a chunk reaches 8 steps ahead: send every 2nd
                steps = {mg: [target(mg, t + i * dt_s) for i in range(CHUNK_SIZE)] for mg in groups}
                jogger.set_chunk(steps, dt_ms=DT_MS)
                last_send_t = t


if __name__ == "__main__":
    nova.run_program(main)
