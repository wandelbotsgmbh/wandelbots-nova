"""
Pose transformations: what `@` and `~` actually do, and when you need them.

A `Pose` is both a *place* (where something is) and a *transform* (how to get from one
coordinate system to another). That double meaning is what makes the two operators work:

    a @ b   apply `b` inside `a`'s coordinate system  -> a new pose in `a`'s parent system
    ~a      the inverse transform                     -> undoes `a`

Everything below is pure local math - no robot, no NOVA connection. Run it with:

    PYTHONPATH=. uv run python examples/pose_transformations.py

Every step asserts its expected numbers, so the file doubles as a specification.
"""

import numpy as np

from nova.types import Pose


def assert_pose(actual: Pose, expected: tuple[float, ...], label: str) -> None:
    assert np.allclose(actual.to_tuple(), expected, atol=1e-6), f"{label}: {actual.to_tuple()}"
    print(f"  {label:<34} {tuple(round(v, 3) for v in actual)}")


def translation_composes_by_adding() -> None:
    """The simple case: without rotation, `@` just adds the positions.

    Use case: nothing special yet - this is the baseline to compare the rotated cases against.
    """
    print("\ntranslation only")
    station = Pose((1000, 500, 300, 0, 0, 0))
    offset = Pose((100, 0, 0, 0, 0, 0))

    assert_pose(station @ offset, (1100, 500, 300, 0, 0, 0), "station @ offset")


def composition_is_relative_to_the_left_pose() -> None:
    """`a @ b` moves along `a`'s axes, not the world's. This is the whole point of `@`.

    Use case: "move 100 mm along the part's length" when the part is rotated on the table.
    Why: you teach the offset once, in part coordinates, and it stays correct no matter how
    the part is oriented. Adding 100 to the world X would only work for an unrotated part.
    """
    print("\ncomposition is relative to the left pose")
    # Rotated 90 deg about Z, so the station's +X axis points along the world's +Y.
    station = Pose((1000, 500, 300, 0, 0, np.pi / 2))
    forward = Pose((100, 0, 0, 0, 0, 0))

    # 100 mm along the STATION's X -> world +Y.
    assert_pose(station @ forward, (1000, 600, 300, 0, 0, np.pi / 2), "station @ forward (local)")

    # The same offset on the left instead: 100 mm along the WORLD's X.
    assert_pose(forward @ station, (1100, 500, 300, 0, 0, np.pi / 2), "forward @ station (world)")

    # So the order is not interchangeable - `@` is not commutative.
    assert station @ forward != forward @ station


def approach_and_retreat_follow_the_tool() -> None:
    """Back off along the tool's own approach axis with `target @ Pose((0, 0, -d, 0, 0, 0))`.

    Use case: every pick and place - you need a collision free pose just before the target.
    Why: the retreat has to follow the tool direction. Here the tool points down (180 deg about
    X), so -100 along the tool's Z comes out as +100 in world Z, i.e. 100 mm above the part.
    Hard-coding "+100 in world Z" breaks the moment the tool is tilted.
    """
    print("\napproach and retreat")
    grip = Pose((800, 0, 400, np.pi, 0, 0))
    retreat = Pose((0, 0, -100, 0, 0, 0))

    assert_pose(grip @ retreat, (800, 0, 500, np.pi, 0, 0), "grip @ retreat")


def inverse_converts_world_back_to_local() -> None:
    """`~frame @ world_pose` answers "where is this, seen from the frame?".

    Use case: a camera reports a part in world coordinates and you want to store it relative
    to the pallet it sits on, so the value survives the pallet being moved.
    Why: `frame @ local` goes one way, `~frame @ world` goes back. They are exact inverses.
    """
    print("\ninverse converts world back to local")
    pallet = Pose((1000, 500, 300, 0, 0, np.pi / 2))
    slot_in_pallet = Pose((200, 100, 0, 0, 0, 0))

    slot_in_world = pallet @ slot_in_pallet
    assert_pose(slot_in_world, (900, 700, 300, 0, 0, np.pi / 2), "pallet @ slot")

    # ... and straight back again.
    assert_pose(~pallet @ slot_in_world, (200, 100, 0, 0, 0, 0), "~pallet @ slot_in_world")
    assert ~pallet @ slot_in_world == slot_in_pallet


def a_moved_frame_carries_everything_with_it() -> None:
    """Teach once in frame coordinates, then re-apply against wherever the frame actually is.

    Use case: a pallet is dropped roughly into place and a camera measures its true position.
    Every slot you taught has to move with it.
    Why: this is exactly what dataset frames do - see examples/datasets.py. The slots stay
    constant in pallet coordinates; only the one frame pose changes per cycle.
    """
    print("\na moved frame carries everything with it")
    nominal = Pose((1000, 500, 300, 0, 0, 0))
    slots = [Pose((0, 0, 0, 0, 0, 0)), Pose((150, 0, 0, 0, 0, 0)), Pose((300, 0, 0, 0, 0, 0))]

    # The camera finds the pallet 5 mm further out and rotated by 90 deg.
    measured = Pose((1005, 500, 300, 0, 0, np.pi / 2))

    assert_pose(measured @ slots[0], (1005, 500, 300, 0, 0, np.pi / 2), "slot 0")
    assert_pose(measured @ slots[1], (1005, 650, 300, 0, 0, np.pi / 2), "slot 1")
    assert_pose(measured @ slots[2], (1005, 800, 300, 0, 0, np.pi / 2), "slot 2")

    # If you only have the slots in WORLD coordinates from the nominal setup, localize them
    # first and then re-apply - `~nominal @ world` recovers the pallet-relative value.
    taught_world = nominal @ slots[1]
    assert measured @ (~nominal @ taught_world) == measured @ slots[1]


def main() -> None:
    translation_composes_by_adding()
    composition_is_relative_to_the_left_pose()
    approach_and_retreat_follow_the_tool()
    inverse_converts_world_back_to_local()
    a_moved_frame_carries_everything_with_it()
    print("\nAll transformations matched their expected values.")


if __name__ == "__main__":
    main()
