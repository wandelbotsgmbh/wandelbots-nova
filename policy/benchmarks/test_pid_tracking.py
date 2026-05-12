"""
PID tracking accuracy tests for action-chunk jogging.

Sends pre-computed trajectories as ActionChunks with dt_ms spacing and measures
how faithfully the PID controller tracks them.

Requirements for a good PID controller:
  - Never overshoot the target path
  - Follow chunk timing faithfully (position at specified time)
  - Smooth motion across profiles (circle, linear, sin, seesaw, zigzag)
  - No stops between waypoints — continuous motion
  - Hold position when chunk contains repeated targets
  - No oscillation around the target
  - Smooth transition when a new chunk arrives mid-execution

Prerequisites:
    NOVA_API=http://<instance-ip> with a virtual UR10e named 'ur10'

Usage:
    NOVA_API=http://172.31.11.129 PYTHONPATH=. python policy/examples/test_pid_tracking.py
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import time

import nova
from nova import api, run_program
from nova.cell import virtual_controller
from nova.program import ProgramPreconditions
from policy import EmergencyStopError, MotionError, jog_joints

# ---------------------------------------------------------------------------
# Settings
# ---------------------------------------------------------------------------

HOME = [0.0, -1.571, 1.571, -1.571, -1.571, 0.0]
CHUNK_SIZE = 16
DT_MS = 50.0            # 50ms between steps → chunk covers 800ms
DT_S = DT_MS / 1000.0
JOINT = 0               # primary joint to move
JOINT2 = 2              # secondary joint for 2D patterns (circle)
DURATION = 8.0           # seconds per test


# ---------------------------------------------------------------------------
# Tracking recorder
# ---------------------------------------------------------------------------

@dataclass
class Sample:
    t: float                 # wall-clock time since test start
    expected: list[float]    # expected joint positions at this moment
    actual: list[float]      # actual joint positions at this moment


@dataclass
class TrackingResult:
    name: str
    samples: list[Sample] = field(default_factory=list)

    @property
    def errors(self) -> list[float]:
        """Per-sample absolute error on the primary joint."""
        return [abs(s.actual[JOINT] - s.expected[JOINT]) for s in self.samples]

    @property
    def max_error_rad(self) -> float:
        e = self.errors
        return max(e) if e else 0.0

    @property
    def mean_error_rad(self) -> float:
        e = self.errors
        return sum(e) / len(e) if e else 0.0

    @property
    def max_error_deg(self) -> float:
        return math.degrees(self.max_error_rad)

    @property
    def mean_error_deg(self) -> float:
        return math.degrees(self.mean_error_rad)

    def overshoot_count(self, threshold_deg: float = 0.5) -> int:
        """Count samples where the joint crossed past the target by more than threshold.

        Only counts sign changes in error where both sides exceed the threshold,
        filtering out measurement noise.
        """
        count = 0
        threshold = math.radians(threshold_deg)
        for i in range(2, len(self.samples)):
            prev_err = self.samples[i - 1].expected[JOINT] - self.samples[i - 1].actual[JOINT]
            curr_err = self.samples[i].expected[JOINT] - self.samples[i].actual[JOINT]
            if (
                abs(prev_err) > threshold
                and abs(curr_err) > threshold
                and (prev_err > 0) != (curr_err > 0)
            ):
                count += 1
        return count

    def print_summary(self) -> None:
        print(f"    Samples:    {len(self.samples)}")
        print(f"    Max error:  {self.max_error_deg:.3f}° ({self.max_error_rad:.5f} rad)")
        print(f"    Mean error: {self.mean_error_deg:.3f}° ({self.mean_error_rad:.5f} rad)")
        print(f"    Overshoots: {self.overshoot_count()}")


# ---------------------------------------------------------------------------
# Trajectory generators
# All are pure functions: (base, t) → list of CHUNK_SIZE joint targets
# ---------------------------------------------------------------------------

def linear_ramp(base: list[float], t: float, *, speed: float = 0.1) -> list[list[float]]:
    """Constant velocity ramp on JOINT."""
    steps = []
    for i in range(CHUNK_SIZE):
        s = list(base)
        s[JOINT] = base[JOINT] + speed * (t + i * DT_S)
        steps.append(s)
    return steps


def sinusoidal(
    base: list[float], t: float, *, amplitude: float = 0.3, freq: float = 0.5,
) -> list[list[float]]:
    """Sinusoidal oscillation on JOINT."""
    steps = []
    for i in range(CHUNK_SIZE):
        s = list(base)
        s[JOINT] = base[JOINT] + amplitude * math.sin(2 * math.pi * freq * (t + i * DT_S))
        steps.append(s)
    return steps


def seesaw(
    base: list[float], t: float, *, amplitude: float = 0.3, period: float = 2.0,
) -> list[list[float]]:
    """Triangle / seesaw wave on JOINT. Linear ramp up then down."""
    steps = []
    for i in range(CHUNK_SIZE):
        phase = ((t + i * DT_S) % period) / period
        value = amplitude * (4 * phase - 1) if phase < 0.5 else amplitude * (3 - 4 * phase)
        s = list(base)
        s[JOINT] = base[JOINT] + value
        steps.append(s)
    return steps


def zigzag(
    base: list[float], t: float, *, amplitude: float = 0.15, period: float = 2.0,
) -> list[list[float]]:
    """Fast zigzag — square-ish wave with sharp corners. Tests reversal behavior."""
    steps = []
    for i in range(CHUNK_SIZE):
        phase = ((t + i * DT_S) % period) / period
        # Spend 40% at +amp, 10% transitioning, 40% at -amp, 10% transitioning
        if phase < 0.4:
            value = amplitude
        elif phase < 0.5:
            value = amplitude * (1 - (phase - 0.4) / 0.1 * 2)
        elif phase < 0.9:
            value = -amplitude
        else:
            value = -amplitude + amplitude * 2 * (phase - 0.9) / 0.1
        s = list(base)
        s[JOINT] = base[JOINT] + value
        steps.append(s)
    return steps


def circle_2d(
    base: list[float], t: float, *, radius: float = 0.15, freq: float = 0.4,
) -> list[list[float]]:
    """Circular motion on JOINT and JOINT2 simultaneously."""
    steps = []
    for i in range(CHUNK_SIZE):
        angle = 2 * math.pi * freq * (t + i * DT_S)
        s = list(base)
        s[JOINT] = base[JOINT] + radius * math.cos(angle)
        s[JOINT2] = base[JOINT2] + radius * math.sin(angle)
        steps.append(s)
    return steps


def hold_position(base: list[float], _t: float) -> list[list[float]]:
    """All steps are identical — tests that the robot holds still without oscillation."""
    target = list(base)
    target[JOINT] = base[JOINT] + 0.2  # offset from home
    return [list(target) for _ in range(CHUNK_SIZE)]


def step_function(
    base: list[float], t: float, *, amplitude: float = 0.3, period: float = 3.0,
) -> list[list[float]]:
    """Step function — jumps between two positions. Tests smooth transition to new target."""
    steps = []
    for i in range(CHUNK_SIZE):
        phase = ((t + i * DT_S) % period) / period
        value = amplitude if phase < 0.5 else 0.0
        s = list(base)
        s[JOINT] = base[JOINT] + value
        steps.append(s)
    return steps


# ---------------------------------------------------------------------------
# Expected position calculator
# ---------------------------------------------------------------------------

def expected_position(chunk_fn, base: list[float], t: float) -> list[float]:
    """What the joint positions SHOULD be at time t according to the trajectory function."""
    # Generate a 1-step chunk at exactly time t
    single = chunk_fn(base, t)
    return single[0]


# ---------------------------------------------------------------------------
# Test runner
# ---------------------------------------------------------------------------

async def go_home(mg) -> None:
    """Move the robot back to HOME between tests."""
    async with jog_joints(mg) as jogger:
        jogger.set_target(HOME)
        t0 = time.monotonic()
        async for state in jogger:
            if time.monotonic() - t0 > 3.0:
                break
            if all(abs(a - h) < 0.01 for a, h in zip(state.joints, HOME, strict=False)):
                break


async def run_test(
    mg,
    name: str,
    chunk_fn,
    *,
    duration: float = DURATION,
    inference_hz: float = 10.0,
) -> TrackingResult:
    """Run one trajectory test and record tracking error at every jogger tick."""
    print(f"\n{'=' * 60}")
    print(f"  {name}")
    print(f"  chunk={CHUNK_SIZE} x {DT_MS}ms = {CHUNK_SIZE * DT_MS:.0f}ms, inference={inference_hz}Hz")
    print(f"{'=' * 60}")

    result = TrackingResult(name=name)
    inference_interval = 1.0 / inference_hz

    async with jog_joints(mg) as jogger:
        t0 = time.monotonic()
        last_inference = -inference_interval  # send first chunk immediately

        # Use the robot's actual position as base (not HOME)
        first_state = True
        base = list(HOME)

        async for state in jogger:
            if first_state:
                base = list(state.joints)
                first_state = False

            t = time.monotonic() - t0
            if t >= duration:
                break

            # Send a new chunk at inference_hz
            if t - last_inference >= inference_interval:
                chunk = chunk_fn(base, t)
                jogger.set_target(chunk, dt_ms=DT_MS)
                last_inference = t

            # Record tracking error
            expected = expected_position(chunk_fn, base, t)
            result.samples.append(Sample(
                t=t,
                expected=expected,
                actual=list(state.joints),
            ))

    result.print_summary()
    return result


# ---------------------------------------------------------------------------
# Single-step vs chunked comparison
# ---------------------------------------------------------------------------

async def run_single_vs_chunked(mg) -> tuple[TrackingResult, TrackingResult]:
    """Compare single-step (no interpolation) vs chunked (with interpolation)."""
    amplitude = 0.3
    freq = 0.5

    def expected_fn(t: float) -> float:
        return HOME[JOINT] + amplitude * math.sin(2 * math.pi * freq * t)

    print(f"\n{'=' * 60}")
    print("  COMPARISON: Single-step vs Chunked (sin wave)")
    print(f"{'=' * 60}")

    # --- Single-step ---
    r_single = TrackingResult(name="Single-step (sin)")
    async with jog_joints(mg) as jogger:
        t0 = time.monotonic()
        last_send = -0.1
        base = list(HOME)
        first = True
        async for state in jogger:
            if first:
                base = list(state.joints)
                first = False
            t = time.monotonic() - t0
            if t >= DURATION:
                break
            if t - last_send >= 0.1:
                target = list(base)
                target[JOINT] = expected_fn(t)
                jogger.set_target(target)
                last_send = t
            exp = list(base)
            exp[JOINT] = expected_fn(t)
            r_single.samples.append(Sample(t=t, expected=exp, actual=list(state.joints)))

    # --- Chunked ---
    r_chunked = TrackingResult(name="Chunked (sin)")
    async with jog_joints(mg) as jogger:
        t0 = time.monotonic()
        last_send = -0.1
        base = list(HOME)
        first = True
        async for state in jogger:
            if first:
                base = list(state.joints)
                first = False
            t = time.monotonic() - t0
            if t >= DURATION:
                break
            if t - last_send >= 0.1:
                chunk = sinusoidal(base, t, amplitude=amplitude, freq=freq)
                jogger.set_target(chunk, dt_ms=DT_MS)
                last_send = t
            exp = list(base)
            exp[JOINT] = expected_fn(t)
            r_chunked.samples.append(Sample(t=t, expected=exp, actual=list(state.joints)))

    print("\n  Single-step:")
    r_single.print_summary()
    print("\n  Chunked:")
    r_chunked.print_summary()

    if r_single.mean_error_rad > 0:
        improvement = (1 - r_chunked.mean_error_rad / r_single.mean_error_rad) * 100
        print(f"\n  Chunking improvement: {improvement:.0f}% lower mean error")

    return r_single, r_chunked


# ---------------------------------------------------------------------------
# Chunk overlap stress test
# ---------------------------------------------------------------------------

async def run_overlap_test(mg) -> TrackingResult:
    """Chunks arrive at 20Hz but each covers 800ms — heavy overlap.

    Tests that the PID controller smoothly transitions between overlapping
    chunks without jerks or oscillation.
    """
    return await run_test(
        mg,
        "Chunk overlap (sin, 20Hz inference, 800ms chunks → 94% overlap)",
        lambda base, t: sinusoidal(base, t, amplitude=0.2, freq=0.3),
        inference_hz=20.0,
    )


# ---------------------------------------------------------------------------
# Chunk gap test: chunk → 2s pause → chunk
# ---------------------------------------------------------------------------


async def run_chunk_gap_test(mg) -> TrackingResult:
    """Send a ramp chunk, wait 2s with no new chunks, then send another.

    During the gap, the robot must hold the last target without overshoot.
    After the gap, it must smoothly resume tracking the new chunk.
    """
    print(f"\n{'=' * 60}")
    print("  Chunk gap (ramp, 2s pause, ramp)")
    print(f"{'=' * 60}")

    result = TrackingResult(name="Chunk gap (2s pause)")
    offset = 0.2  # ramp target offset from base

    async with jog_joints(mg) as jogger:
        t0 = time.monotonic()
        base = None

        async for state in jogger:
            if base is None:
                base = list(state.joints)

            t = time.monotonic() - t0
            if t >= 6.0:
                break

            # Phase 1 (t < 1s): send ramp chunk to offset
            if t < 0.2:
                target = list(base)
                target[JOINT] = base[JOINT] + offset * (t / 0.2)  # ramp over 200ms
                jogger.set_target([target])  # single step
            elif t < 1.0:
                target = list(base)
                target[JOINT] = base[JOINT] + offset
                jogger.set_target([target])

            # Phase 2 (1s < t < 3s): NO chunks sent — robot should hold
            # (jogger receives no set_target calls)

            # Phase 3 (t > 3s): send new ramp chunk moving back toward base
            elif t >= 3.0 and t < 3.2:
                target = list(base)
                target[JOINT] = base[JOINT] + offset * (1.0 - (t - 3.0) / 0.2)
                jogger.set_target([target])
            elif t >= 3.2:
                jogger.set_target([list(base)])

            # Expected position for tracking
            expected = list(base)
            if t < 0.2:
                expected[JOINT] = base[JOINT] + offset * (t / 0.2)
            elif t < 3.0:
                expected[JOINT] = base[JOINT] + offset  # should hold here
            elif t < 3.2:
                expected[JOINT] = base[JOINT] + offset * (1.0 - (t - 3.0) / 0.2)
            # else: back at base

            result.samples.append(Sample(t=t, expected=expected, actual=list(state.joints)))

    # Check specifically the hold phase (1s - 3s) for overshoot
    hold_errors = [abs(s.actual[JOINT] - s.expected[JOINT]) for s in result.samples if 1.5 < s.t < 2.8]
    if hold_errors:
        hold_max = max(hold_errors)
        hold_mean = sum(hold_errors) / len(hold_errors)
        print("\n  Hold phase (1.5-2.8s):")
        print(f"    Max error:  {math.degrees(hold_max):.3f}\u00b0")
        print(f"    Mean error: {math.degrees(hold_mean):.3f}\u00b0")
        if hold_max > 0.02:  # > ~1 degree
            print("    WARNING: robot drifted during hold!")

    result.print_summary()
    return result


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

@nova.program(
    id="test_pid_tracking",
    name="PID Tracking Tests",
    description="Measures PID tracking accuracy across trajectory profiles.",
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
async def test_pid_tracking(ctx: nova.ProgramContext):
    cell = ctx.nova.cell()

    # Use existing controller or the one created by preconditions
    try:
        ctrl = await cell.controller("ur10")
    except Exception:
        ctrl = await cell.controller("ur10e")
    mg = ctrl[0]

    try:
        results: list[TrackingResult] = []

        # 1. Linear ramp — constant velocity
        await go_home(mg)
        r = await run_test(mg, "Linear ramp (0.1 rad/s)", linear_ramp)
        results.append(r)

        # 2. Sinusoidal — smooth acceleration changes
        await go_home(mg)
        r = await run_test(mg, "Sinusoidal (A=0.3, f=0.5Hz)", sinusoidal)
        results.append(r)

        # 3. Seesaw / triangle — linear segments with sharp corners
        await go_home(mg)
        r = await run_test(mg, "Seesaw / triangle (A=0.3, T=2s)", seesaw)
        results.append(r)

        # 4. Zigzag — hold + sharp reversals
        await go_home(mg)
        r = await run_test(mg, "Zigzag (A=0.15, T=2s)", zigzag)
        results.append(r)

        # 5. Circle — coordinated 2-joint motion
        await go_home(mg)
        r = await run_test(mg, "Circle 2D (r=0.15, f=0.4Hz)", circle_2d)
        results.append(r)

        # 6. Hold position — should be perfectly still
        await go_home(mg)
        r = await run_test(mg, "Hold position (static target)", hold_position, duration=4.0)
        results.append(r)

        # 7. Step function — instantaneous target change
        await go_home(mg)
        r = await run_test(mg, "Step function (A=0.3, T=3s)", step_function)
        results.append(r)

        # 8. Chunk overlap stress
        await go_home(mg)
        r = await run_overlap_test(mg)
        results.append(r)

        # 9. Chunk gap (hold position during 2s pause)
        await go_home(mg)
        r = await run_chunk_gap_test(mg)
        results.append(r)

        # 10. Single-step vs chunked comparison
        await go_home(mg)
        r_single, r_chunked = await run_single_vs_chunked(mg)
        results.append(r_single)
        results.append(r_chunked)

        # ---- Summary ----
        print(f"\n{'=' * 70}")
        print("  SUMMARY")
        print(f"{'=' * 70}")
        print(f"  {'Test':<50} {'Max °':>7} {'Mean °':>8} {'Overshoot':>10}")
        print(f"  {'-' * 50} {'-' * 7} {'-' * 8} {'-' * 10}")
        for r in results:
            print(
                f"  {r.name:<50} "
                f"{r.max_error_deg:>7.3f} "
                f"{r.mean_error_deg:>8.3f} "
                f"{r.overshoot_count():>10}"
            )

    except MotionError as e:
        print(f"\nMotion error: {e}")
    except EmergencyStopError as e:
        print(f"\nEmergency stop: {e}")


if __name__ == "__main__":
    run_program(test_pid_tracking)
