"""Keyboard TCP jogging on a throwaway virtual UR10e: hold a key and the TCP moves.

    NOVA_API=http://<instance> PYTHONPATH=. uv run python \\
        novapolicy/examples/jogging/jogging_keyboard.py

    w / s   +x / -x        a / d   +y / -y        r / f   +z / -z
    + / -   faster / slower          space   stop          q   quit

Run it in a terminal: keys are read one at a time without Enter. Holding a key repeats it, and the
TCP moves until 150 ms after the last repeat; a first press counts for 0.6 s, the pause before the
terminal starts repeating. The speed ramps at a fixed acceleration, so taps and releases do not
jerk. The TCP stays inside a box around its start.

Each pose goes out as a live target (``set_target``) on the default pipeline. For a steadier
speed at the cost of more delay, add a ``LiveBuffer`` to the pipeline (see jogging_joints.py).
"""

import contextlib
import math
import os
import queue
import sys
import threading
import time

import nova
from nova.cell import virtual_controller
from nova.types import Pose
from novapolicy import jog_tcp

CONTROLLER = "ur10e-keyboard"
START = [0.9484, -2.128, 2.2734, -1.7487, -1.5773, 2.552]
KEYS = {  # key -> (axis, sign)
    "w": (0, 1),
    "s": (0, -1),
    "a": (1, 1),
    "d": (1, -1),
    "r": (2, 1),
    "f": (2, -1),
}
FIRST_HOLD_S = 0.6  # a first press: covers the pause before the terminal repeats the key
REPEAT_HOLD_S = 0.15  # a repeated key: covers the time between repeats
ACCEL_MM_S2 = 800.0
SPEEDS_MM_S = (10.0, 25.0, 50.0, 100.0, 150.0)
BOX_MM = 300.0  # the TCP stays within this of its start on each axis


class KeyboardJog:
    """The target position, moved by the keys held, at a speed ramped by ``ACCEL_MM_S2``."""

    def __init__(self, start: Pose) -> None:
        self.start = start
        self.target = list(start.position)
        self.velocity = [0.0, 0.0, 0.0]
        self.speed_index = 1
        self.held: dict[int, tuple[int, float, float]] = {}  # axis -> (sign, last press, hold)

    def press(self, key: str, now: float) -> None:
        if key == " ":
            self.held.clear()
        elif key in "+=":
            self.speed_index = min(self.speed_index + 1, len(SPEEDS_MM_S) - 1)
        elif key in "-_":
            self.speed_index = max(self.speed_index - 1, 0)
        elif key in KEYS:
            axis, sign = KEYS[key]
            repeated = axis in self.held and self.held[axis][0] == sign
            self.held[axis] = (sign, now, REPEAT_HOLD_S if repeated else FIRST_HOLD_S)

    def advance(self, now: float, dt: float) -> Pose:
        """Move the target by ``dt``; the pose to send."""
        self.held = {a: h for a, h in self.held.items() if now - h[1] < h[2]}
        step = ACCEL_MM_S2 * dt
        for axis in range(3):
            sign = self.held[axis][0] if axis in self.held else 0
            wanted = sign * SPEEDS_MM_S[self.speed_index]
            self.velocity[axis] += max(-step, min(step, wanted - self.velocity[axis]))
            position = self.target[axis] + self.velocity[axis] * dt
            if self._inside(axis, position) != position:
                position, self.velocity[axis] = self._inside(axis, position), 0.0
            self.target[axis] = position
        return Pose(*self.target, *self.start.orientation)

    def _inside(self, axis: int, value: float) -> float:
        """``value`` held inside the box around the start on ``axis``."""
        centre = self.start.position[axis]
        return min(centre + BOX_MM, max(centre - BOX_MM, value))

    def status(self) -> str:
        x, y, z = (self.target[i] - self.start.position[i] for i in range(3))
        return (
            f"\r{SPEEDS_MM_S[self.speed_index]:5.0f} mm/s max | "
            f"moving {math.hypot(*self.velocity):5.1f} mm/s | "
            f"offset x {x:+6.1f}  y {y:+6.1f}  z {z:+6.1f} mm "
        )


def read_keys(keys: queue.Queue[str]) -> None:
    """Put each key typed on stdin into ``keys``; the end of a piped input is a ``q``."""
    while char := os.read(sys.stdin.fileno(), 1).decode(errors="ignore"):
        keys.put(char)
    keys.put("q")


@contextlib.contextmanager
def keys_without_enter():
    """The terminal hands over each key as it is typed (cbreak), restored on exit."""
    fd = sys.stdin.fileno()
    if not os.isatty(fd):
        yield
        return
    import termios  # POSIX only
    import tty

    restore = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, restore)


@nova.program(
    id="jogging_keyboard",
    name="Keyboard Jogging",
    viewer=nova.viewers.Rerun(),
    preconditions=nova.ProgramPreconditions(
        controllers=[
            virtual_controller(
                name=CONTROLLER,
                manufacturer=nova.api.models.Manufacturer.UNIVERSALROBOTS,
                type="universalrobots-ur10e",
            )
        ],
        cleanup_controllers=True,
    ),
)
async def main(ctx: nova.ProgramContext):
    mg = (await ctx.nova.cell().controller(CONTROLLER))[0]
    tcp = (await mg.tcp_names())[0]
    keys: queue.Queue[str] = queue.Queue()
    threading.Thread(target=read_keys, args=(keys,), daemon=True).start()
    print("Moving to the start pose and starting the jogging session ...", flush=True)

    jog: KeyboardJog | None = None
    last_t = 0.0
    async with jog_tcp(mg, tcp=tcp, start_joint_position=START) as jogger:
        async for state in jogger:
            now = time.monotonic()
            pressed = []
            while not keys.empty():
                pressed.append(keys.get())
            if "q" in pressed:
                break
            if jog is None:  # keys typed while it was starting do not count
                jog, pressed = KeyboardJog(state.pose), []
                print("Ready.\n" + __doc__.split("\n\n")[2], flush=True)
            for key in pressed:
                jog.press(key, now)
            dt, last_t = jogger.elapsed - last_t, jogger.elapsed
            jogger.set_target(jog.advance(now, dt))
            print(jog.status(), end="", flush=True)
    print()


if __name__ == "__main__":
    with keys_without_enter(), contextlib.suppress(KeyboardInterrupt):
        nova.run_program(main)
