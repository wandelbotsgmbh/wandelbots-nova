"""Teleoperation with a Meta Quest controller, mocked: the TCP follows the controller's pose.

    NOVA_API=http://<instance> PYTHONPATH=. uv run python \\
        novapolicy/examples/jogging/jogging_quest.py [--script within|beyond|all|seam] [--speed 0.5]

While the grip is held, the TCP moves by the controller's motion since the grip was pressed:
its translation, turned into the robot's base frame, and its rotation, about the TCP. Releasing
the grip holds the TCP; pressing it again re-anchors, so the hand can be repositioned (a clutch).

``QuestMock`` stands in for the headset: controller poses at 72 Hz, arriving with a few ms of
jitter, along a scripted motion of fast flicks, sudden reversals and quick wrist rotations. The
``within`` part stays under the UR10e's speeds (TCP about 1.5 m/s, wrist about 180 deg/s) but its
reversals need up to 11 m/s2, more than the arm can brake; the ``beyond`` part exceeds the speeds
too. ``--speed 0.5`` plays a script at half speed, a quarter of the acceleration. ``seam`` turns
the tool slowly across the half-turn where its rotation vector flips sign. A real headset only
has to provide ``latest()``: the newest controller pose and grip.

Each pose goes out as a live target (``set_target``) on the default pipeline, one waypoint due
the jogger's lead (100 ms) ahead, the way ur_rtde's ``servoL`` streams the newest pose. Measured
on a virtual UR10e over a remote network:

- ``within`` at half speed (about 2.8 m/s2): the TCP lags the controller by 100 ms, 1.8 mm rms
  and 16 mm at most after that lag, 0.26 deg rms.
- ``within`` at full speed: the arm falls behind the waypoints' times where they ask for more
  acceleration than it has and overshoots stops by 20 to 30 mm; 30 to 80 mm rms.
- A ``LiveBuffer`` (200 to 500 ms) halves the error at half speed and adds its window to the lag;
  at full speed it does not help. Neither does a longer braking delay or a shorter lead.
"""

import argparse
from dataclasses import dataclass
import math
from typing import cast

import numpy as np
from scipy.spatial.transform import Rotation

import nova
from nova import api
from nova.cell import virtual_controller
from nova.types import Pose
from novapolicy import jog_tcp

CONTROLLER = "ur10e-quest"
START = [0.9484, -2.128, 2.2734, -1.7487, -1.5773, 2.552]
RATE_HZ = 72.0  # the Quest's controller tracking rate
ARRIVAL_JITTER_MS = 4.0
# Quest (OpenXR) axes: x right, y up, -z forward. Robot base: x forward, y left, z up.
QUEST_TO_ROBOT = Rotation.from_matrix([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])


@dataclass(frozen=True)
class ControllerSample:
    """One controller pose in the headset's frame: metres, a rotation, the grip."""

    t: float
    position: np.ndarray
    rotation: Rotation
    grip: bool


def _compose(*rotations: Rotation) -> Rotation:
    """``a * b * ...``: apply the last first."""
    out = rotations[0]
    for rotation in rotations[1:]:
        out = cast("Rotation", out * rotation)
    return out


def min_jerk(u: float) -> float:
    """0 -> 1 with zero velocity and acceleration at both ends."""
    u = min(1.0, max(0.0, u))
    return u**3 * (u * (u * 6 - 15) + 10)


# A move: (duration s, translation m in the headset frame, rotation about the controller's own
# axes in deg as (axis, angle)). Consecutive moves meet with no pause: a reversal is sudden.
_X, _Y, _Z = np.eye(3)
SCRIPTS: dict[str, list[tuple[float, np.ndarray, tuple[str, float]]]] = {
    "within": [
        (1.0, 0 * _X, ("z", 0)),  # rest
        (0.4, -0.15 * _Z, ("z", 0)),  # flick forward, 150 mm, peak 0.7 m/s
        (0.4, 0.15 * _Z, ("z", 0)),  # and straight back
        (0.3, 0 * _X, ("z", 0)),
        *[(0.25, s * 0.06 * _X, ("z", 0)) for s in (1, -2, 2, -2, 2, -1)],  # zigzag, 0.9 m/s
        (0.3, 0 * _X, ("z", 0)),
        (0.35, 0.08 * _Y, ("z", 0)),  # up and down
        (0.35, -0.08 * _Y, ("z", 0)),
        (0.3, 0 * _X, ("z", 0)),
        (0.4, 0 * _X, ("z", 25)),  # roll 25 deg, peak 120 deg/s
        (0.6, 0 * _X, ("z", -50)),
        (0.4, 0 * _X, ("z", 25)),
        (0.4, 0 * _X, ("x", 25)),  # pitch
        (0.4, 0 * _X, ("x", -25)),
        (0.5, np.array([0.08, 0.05, -0.1]), ("y", 25)),  # translate and turn at once
        (0.5, np.array([-0.08, -0.05, 0.1]), ("y", -25)),
        (1.0, 0 * _X, ("z", 0)),
    ],
    "beyond": [
        (0.5, 0 * _X, ("z", 0)),
        (0.2, -0.2 * _Z, ("z", 0)),  # 200 mm in 0.2 s: peak 1.9 m/s
        (0.2, 0.2 * _Z, ("z", 0)),
        (0.3, 0 * _X, ("z", 0)),
        (0.2, 0 * _X, ("z", 70)),  # 70 deg in 0.2 s: peak 650 deg/s
        (0.2, 0 * _X, ("z", -70)),
        (1.0, 0 * _X, ("z", 0)),
    ],
    "seam": [  # slow: the tool turns through the half-turn where its rotation vector flips
        (1.0, 0 * _X, ("z", 0)),
        (2.0, 0 * _X, ("z", 10)),  # the start is a 178.7 deg turn about the base x axis
        (2.0, 0 * _X, ("z", -20)),
        (2.0, 0 * _X, ("z", 10)),
        (1.0, 0 * _X, ("z", 0)),
    ],
}
SCRIPTS["all"] = SCRIPTS["within"] + SCRIPTS["beyond"]


class QuestMock:
    """Controller poses along a script, as a headset delivers them: at ``RATE_HZ``, each
    arriving a few ms late, the newest one available at any moment."""

    def __init__(self, script: str, speed: float = 1.0, seed: int = 0) -> None:
        self._moves = [(duration / speed, move, turn) for duration, move, turn in SCRIPTS[script]]
        self.duration = sum(d for d, _, _ in self._moves)
        self._jitter = np.random.default_rng(seed).uniform(0, ARRIVAL_JITTER_MS / 1000, 100_000)
        self._t0: float | None = None

    def pose_at(self, t: float) -> tuple[np.ndarray, Rotation]:
        """The scripted pose at ``t`` seconds into the script."""
        position, rotation, start = np.array([0.0, 0.0, 0.3]), Rotation.identity(), 0.0
        for duration, translation, (axis, angle) in self._moves:
            u = min_jerk((t - start) / duration) if t < start + duration else 1.0
            position += u * translation
            rotation = _compose(rotation, Rotation.from_euler(axis, u * angle, degrees=True))
            if t < start + duration:
                break
            start += duration
        return position, rotation

    def latest(self, now: float) -> ControllerSample:
        """The newest sample that has arrived by ``now`` (seconds, any monotonic clock)."""
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0
        k = math.floor(t * RATE_HZ)
        while k > 0 and k / RATE_HZ + self._jitter[k % len(self._jitter)] > t:
            k -= 1  # this one is still in flight
        sample_t = k / RATE_HZ
        position, rotation = self.pose_at(sample_t)
        return ControllerSample(sample_t, position, rotation, grip=True)


class Teleop:
    """Controller motion since the grip was pressed, applied to the TCP pose at that moment."""

    def __init__(self) -> None:
        self._anchor: tuple[ControllerSample, np.ndarray, Rotation] | None = None
        self._held: Pose | None = None
        self._rotvec: np.ndarray | None = None

    def target(self, sample: ControllerSample, tcp: Pose) -> Pose:
        """The TCP pose to send for ``sample``; ``tcp`` is where the TCP is now."""
        if not sample.grip:
            self._anchor = None
            self._held = self._held or tcp
            return self._held
        if self._anchor is None:
            start = self._held or tcp
            self._anchor = (
                sample,
                np.array(start.position),
                Rotation.from_rotvec(start.orientation),
            )
            self._held = None
        controller0, position0, rotation0 = self._anchor
        moved = QUEST_TO_ROBOT.apply(sample.position - controller0.position) * 1000.0
        turned = _compose(
            QUEST_TO_ROBOT, sample.rotation, controller0.rotation.inv(), QUEST_TO_ROBOT.inv()
        )
        return Pose(*(position0 + moved), *self._continuous(_compose(turned, rotation0)))

    def _continuous(self, rotation: Rotation) -> np.ndarray:
        """``rotation`` as the rotation vector closest to the previous one: near a half-turn the
        shortest vector flips sign, a jump of 2 pi for the same orientation."""
        rotvec = rotation.as_rotvec()
        angle = np.linalg.norm(rotvec)
        if self._rotvec is not None and angle > 1e-9:
            other = rotvec * (1.0 - 2.0 * math.pi / angle)  # the same rotation, the long way
            if np.linalg.norm(other - self._rotvec) < np.linalg.norm(rotvec - self._rotvec):
                rotvec = other
        self._rotvec = rotvec
        return rotvec


def make_program(script: str, speed: float):
    @nova.program(
        id="jogging_quest",
        name="Quest Teleoperation (mocked)",
        viewer=nova.viewers.Rerun(),
        preconditions=nova.ProgramPreconditions(
            controllers=[
                virtual_controller(
                    name=CONTROLLER,
                    manufacturer=api.models.Manufacturer.UNIVERSALROBOTS,
                    type="universalrobots-ur10e",
                )
            ],
            cleanup_controllers=True,
        ),
    )
    async def main(ctx: nova.ProgramContext):
        mg = (await ctx.nova.cell().controller(CONTROLLER))[0]
        tcp = (await mg.tcp_names())[0]
        quest, teleop = QuestMock(script, speed), Teleop()
        async with jog_tcp(mg, tcp=tcp, start_joint_position=START) as jogger:
            async for state in jogger:
                now = jogger.elapsed
                if now > quest.duration + 0.5:
                    break
                jogger.set_target(teleop.target(quest.latest(now), state.pose))
        print(f"Played the '{script}' script ({quest.duration:.1f} s).")

    return main


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--script", choices=sorted(SCRIPTS), default="all")
    parser.add_argument("--speed", type=float, default=1.0, help="play the script this fast")
    args = parser.parse_args()
    nova.run_program(make_program(args.script, args.speed))


if __name__ == "__main__":
    main()
