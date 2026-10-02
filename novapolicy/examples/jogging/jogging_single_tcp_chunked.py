"""Single-arm TCP jogging: trace a 50mm circle in XZ plane (chunked, smooth).

Uses 100-step chunks at 10ms for smooth motion with 1s lookahead.
"""

import math

import nova
from novapolicy import jog_tcp

START_JOINTS = [0.9484, -2.128, 2.2734, -1.7487, -1.5773, 2.552]


@nova.program(
    id="jogging_single_tcp_chunked",
    name="Single-Arm TCP Jogging (Chunked)",
    viewer=nova.viewers.Rerun(),
)
async def main(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    mg = (await cell.controller("ur10e"))[0]
    tcp_name = (await mg.tcp_names())[0]

    duration = 10.0
    radius = 50.0
    chunk_size = 100
    dt_ms = 10.0
    dt_s = dt_ms / 1000.0

    def smootherstep(x: float) -> float:
        """Monotone 0->1 warp with zero velocity *and* zero acceleration at both ends.

        Warping the circle's phase, rather than enveloping its amplitude, keeps the
        TCP exactly on the circle and simply slows it at each end. An amplitude
        envelope would drag it to the circle's centre and back out.
        """
        x = min(1.0, max(0.0, x))
        return x**3 * (x * (x * 6 - 15) + 10)

    # No ease_in_s: the phase warp already starts at zero velocity.
    async with jog_tcp(mg, tcp=tcp_name, start_joint_position=START_JOINTS) as jogger:
        start_pose = None
        center_x = 0.0
        center_z = 0.0
        async for state in jogger:
            t = jogger.elapsed
            if t >= duration:
                break
            if start_pose is None:
                start_pose = state.pose
                center_x = start_pose.position[0] - radius
                center_z = start_pose.position[2]
            # Rebuilt and re-sent every iteration. That is safe because chunks are
            # placed on one absolute timeline: the same trajectory point keeps the
            # same timestamp, so successive chunks stitch instead of restarting.
            # jogging_single_joint_chunked.py shows the throttled alternative,
            # which sends the same motion with far less traffic.
            chunk = []
            for i in range(chunk_size):
                future_t = t + i * dt_s
                angle = 2 * math.pi * smootherstep(future_t / duration)
                chunk.append([
                    center_x + radius * math.cos(angle),
                    start_pose.position[1],
                    center_z + radius * math.sin(angle),
                    *start_pose.orientation,
                ])
            # set_chunk takes raw [x, y, z, rx, ry, rz] steps, not Pose objects —
            # unlike set_target, which takes a Pose.
            jogger.set_chunk(chunk, dt_ms=dt_ms)


if __name__ == "__main__":
    nova.run_program(main)
