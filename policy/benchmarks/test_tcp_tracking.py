"""TCP tracking accuracy tests — matches joint test depth.

Sends TCP pose targets at ~100Hz and measures Euclidean position error.

Usage:
    NOVA_API=http://172.31.11.129 PYTHONPATH=. python policy/examples/test_tcp_tracking.py
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time

import nova
from nova import api, run_program
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from nova.types import Pose
from policy import EmergencyStopError, MotionError, jog_joints, jog_tcp

HOME = [0.0, -1.571, 1.571, -1.571, -1.571, 0.0]


@dataclass
class Sample:
    t: float
    expected_pos: tuple[float, float, float]
    actual_pos: tuple[float, float, float]


@dataclass
class TcpResult:
    name: str
    samples: list[Sample] = field(default_factory=list)

    @property
    def errors_mm(self) -> list[float]:
        return [
            math.sqrt(sum((a - e) ** 2 for a, e in zip(s.actual_pos, s.expected_pos)))
            for s in self.samples
        ]

    @property
    def max_mm(self) -> float:
        e = self.errors_mm
        return max(e) if e else 0

    @property
    def mean_mm(self) -> float:
        e = self.errors_mm
        return sum(e) / len(e) if e else 0

    def print_summary(self) -> None:
        print(f"    Samples:    {len(self.samples)}")
        print(f"    Max error:  {self.max_mm:.2f} mm")
        print(f"    Mean error: {self.mean_mm:.2f} mm")


async def go_home(mg: object) -> None:
    async with jog_joints(mg) as jogger:
        t0 = time.monotonic()
        async for state in jogger:
            if time.monotonic() - t0 > 3.0:
                break
            if all(abs(a - h) < 0.01 for a, h in zip(state.joints, HOME, strict=False)):
                break
            jogger.set_target(HOME)


async def run_tcp_test(
    mg: object,
    tcp_name: str,
    name: str,
    traj_fn: object,
    *,
    duration: float = 6.0,
) -> TcpResult:
    print(f"\n{'=' * 60}")
    print(f"  {name}")
    print(f"{'=' * 60}")

    result = TcpResult(name=name)

    async with jog_tcp(mg, tcp=tcp_name, tcp_velocity_limit=500.0) as jogger:
        t0 = time.monotonic()
        home_pose = None
        async for state in jogger:
            t = time.monotonic() - t0
            if t >= duration:
                break
            if home_pose is None:
                home_pose = state.pose
                continue

            target = traj_fn(t, home_pose)
            jogger.set_target(target)

            if t > 0.5:
                result.samples.append(Sample(
                    t=t,
                    expected_pos=tuple(target.position),
                    actual_pos=tuple(state.pose.position),
                ))

    result.print_summary()
    return result


# ---------------------------------------------------------------------------
# Trajectory generators: (t, home_pose) -> Pose
# ---------------------------------------------------------------------------

def hold_position(t: float, home: Pose) -> Pose:
    return Pose(*home.position, *home.orientation)


def linear_x(t: float, home: Pose) -> Pose:
    speed = 20.0  # mm/s
    return Pose(
        home.position[0] + speed * t,
        home.position[1],
        home.position[2],
        *home.orientation,
    )


def circle_xy(t: float, home: Pose) -> Pose:
    radius, freq = 20.0, 0.3
    angle = 2 * math.pi * freq * t
    return Pose(
        home.position[0] + radius * math.cos(angle),
        home.position[1] + radius * math.sin(angle),
        home.position[2],
        *home.orientation,
    )


def sin_z(t: float, home: Pose) -> Pose:
    amplitude, freq = 15.0, 0.5
    return Pose(
        home.position[0],
        home.position[1],
        home.position[2] + amplitude * math.sin(2 * math.pi * freq * t),
        *home.orientation,
    )


def step_x(t: float, home: Pose) -> Pose:
    offset, period = 30.0, 3.0
    phase = (t % period) / period
    dx = offset if phase < 0.5 else 0.0
    return Pose(home.position[0] + dx, home.position[1], home.position[2], *home.orientation)


def figure_eight(t: float, home: Pose) -> Pose:
    size, freq = 15.0, 0.25
    angle = 2 * math.pi * freq * t
    return Pose(
        home.position[0] + size * math.sin(angle),
        home.position[1] + size * math.sin(2 * angle),
        home.position[2],
        *home.orientation,
    )


def diagonal_ramp(t: float, home: Pose) -> Pose:
    speed = 15.0
    return Pose(
        home.position[0] + speed * t,
        home.position[1] + speed * t * 0.7,
        home.position[2] + speed * t * 0.3,
        *home.orientation,
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

@nova.program(
    id="test_tcp_tracking",
    name="TCP Tracking Tests",
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(
                name="ur10e",
                manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
            ),
        ],
        cleanup_controllers=False,
    ),
)
async def test_tcp_tracking(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()
    try:
        ctrl = await cell.controller("ur10")
    except Exception:
        ctrl = await cell.controller("ur10e")
    mg = ctrl[0]
    tcps = await mg.tcp_names()
    tcp_name = tcps[0] if tcps else "Flange"
    print(f"Using TCP: {tcp_name}")

    try:
        results: list[TcpResult] = []

        await go_home(mg)
        r = await run_tcp_test(mg, tcp_name, "Hold position", hold_position, duration=3.0)
        results.append(r)

        await go_home(mg)
        r = await run_tcp_test(mg, tcp_name, "Linear X (20 mm/s)", linear_x, duration=4.0)
        results.append(r)

        await go_home(mg)
        r = await run_tcp_test(mg, tcp_name, "Circle XY (r=20mm, 0.3Hz)", circle_xy)
        results.append(r)

        await go_home(mg)
        r = await run_tcp_test(mg, tcp_name, "Sinusoidal Z (A=15mm, 0.5Hz)", sin_z)
        results.append(r)

        await go_home(mg)
        r = await run_tcp_test(mg, tcp_name, "Step X (30mm, T=3s)", step_x)
        results.append(r)

        await go_home(mg)
        r = await run_tcp_test(mg, tcp_name, "Figure-eight XY (15mm, 0.25Hz)", figure_eight)
        results.append(r)

        await go_home(mg)
        r = await run_tcp_test(mg, tcp_name, "Diagonal XYZ ramp (15 mm/s)", diagonal_ramp, duration=4.0)
        results.append(r)

        print(f"\n{'=' * 60}")
        print("  TCP TRACKING SUMMARY")
        print(f"{'=' * 60}")
        print(f"  {'Test':<45} {'Max mm':>7} {'Mean mm':>8}")
        print(f"  {'-' * 45} {'-' * 7} {'-' * 8}")
        for r in results:
            print(f"  {r.name:<45} {r.max_mm:>7.2f} {r.mean_mm:>8.2f}")

    except MotionError as e:
        print(f"\nMotion error: {e}")
    except EmergencyStopError as e:
        print(f"\nEmergency stop: {e}")


if __name__ == "__main__":
    run_program(test_tcp_tracking)
