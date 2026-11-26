"""TCP jogging: the TCP traces a 50 mm circle in the XZ plane, as live poses or as chunks.

    python jogging_tcp.py                    # one arm, live poses
    python jogging_tcp.py --chunked          # 100-step chunks at 10 ms
    python jogging_tcp.py --dual [--chunked] # two arms, circling in opposite directions

The circle starts and stops at rest by easing its *phase*, not its amplitude: an amplitude
envelope (right for an oscillation about home, as in jogging_joints.py) would drag the TCP to the
circle's centre and back. No ``EaseIn`` is needed: the phase warp starts at zero velocity, and
the circle starts at the arm's own pose.

Live poses go through ``ServoHorizon`` (see jogging_joints.py). Chunks are rebuilt and
re-sent every iteration; that is safe because they sit on one absolute timeline, so the same
point keeps the same timestamp and successive chunks stitch. ``set_chunk`` takes raw
``[x, y, z, rx, ry, rz]`` steps; ``set_target`` takes a ``Pose``.
"""

import math
import sys

import nova
from nova.types import Pose
from novapolicy import ServoHorizon, default_pipeline, jog_tcp

DUAL = "--dual" in sys.argv
CHUNKED = "--chunked" in sys.argv
STARTS = (
    {
        "ur5e-left": [1.169, -0.733, 1.745, -3.054, 0.872, 2.094],
        "ur5e-right": [-1.169, -2.3911, -1.8675, 0.0, -0.872, -2.094],
    }
    if DUAL
    else {"ur10e": [0.9484, -2.128, 2.2734, -1.7487, -1.5773, 2.552]}
)
DURATION_S, RADIUS_MM = 5.0, 50.0
CHUNK_SIZE, DT_MS = 100, 10.0


def smootherstep(value: float) -> float:
    """0 -> 1 with zero velocity and acceleration at both ends (smoothstep leaves an
    acceleration step, which the arm executes as a stumble)."""
    x = min(1.0, max(0.0, value))
    return x**3 * (x * (x * 6 - 15) + 10)


@nova.program(id="jogging_tcp", name="TCP Jogging", viewer=nova.viewers.Rerun())
async def main(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    groups = [(await cell.controller(name))[0] for name in STARTS]
    tcps = {mg: (await mg.tcp_names())[0] for mg in groups}
    starts = dict(zip(groups, STARTS.values(), strict=True))
    direction = {mg: 1 - 2 * i for i, mg in enumerate(groups)}  # the second arm circles back
    centres: dict = {}

    def point(mg, t: float) -> list[float]:
        start = centres[mg]
        angle = direction[mg] * 2 * math.pi * smootherstep(t / DURATION_S)
        return [
            start.position[0] - RADIUS_MM + RADIUS_MM * math.cos(angle),
            start.position[1],
            start.position[2] + RADIUS_MM * math.sin(angle),
            *start.orientation,
        ]

    pipeline = None if CHUNKED else [ServoHorizon(), *default_pipeline()]
    dt_s = DT_MS / 1000.0
    async with jog_tcp(tcps, start_joint_position=starts, pipeline=pipeline) as jogger:
        async for state in jogger:
            t = jogger.elapsed
            if t >= DURATION_S:
                break
            if not centres:
                states = state if isinstance(state, dict) else {groups[0]: state}
                centres.update({mg: states[mg].pose for mg in groups})
            if CHUNKED:
                steps = {mg: [point(mg, t + i * dt_s) for i in range(CHUNK_SIZE)] for mg in groups}
                jogger.set_chunk(steps, dt_ms=DT_MS)
            else:
                jogger.set_target({mg: Pose(*point(mg, t)) for mg in groups})


if __name__ == "__main__":
    nova.run_program(main)
