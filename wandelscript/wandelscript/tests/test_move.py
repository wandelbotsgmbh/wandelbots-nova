import asyncio

import numpy as np
import pytest

from nova.actions import CombinedActions, cartesian_ptp, linear
from nova.types import MotionSettings, Pose
from wandelscript.metamodel import run_program


@pytest.mark.parametrize(
    "code, expected",
    [
        (
            """
velocity(200)
tcp = frame("Flange")
move tcp via ptp() to (150, -355, 389, 0, pi, 0)
move tcp via line() to (150, -355, 392, 0, pi, 0) with velocity(10)
move tcp via ptp() to (150, -355, 389, 0, pi, 0)
move tcp via line() to (-95, -363, 387, 0, pi, 0) with velocity(250)
move tcp via ptp() to (150, -355, 389, 0, pi, 0)
""",
            [
                CombinedActions(
                    items=(
                        cartesian_ptp((150, -355, 389), MotionSettings(tcp_velocity_limit=200)),
                        linear((150, -355, 392), MotionSettings(tcp_velocity_limit=10)),
                        cartesian_ptp((150, -355, 389), MotionSettings(tcp_velocity_limit=200)),
                        linear((-95, -363, 387), MotionSettings(tcp_velocity_limit=250)),
                        cartesian_ptp((150, -355, 389), MotionSettings(tcp_velocity_limit=200)),
                    )
                )
            ],
        ),
        (
            """
velocity(200)
move via ptp() to (150, -355, 389, 0, pi, 0)
move via line() to (150, -355, 392, 0, pi, 0) with velocity(10)
move via ptp() to (150, -355, 389, 0, pi, 0)
move via line() to (-95, -363, 387) with velocity(250)
move via ptp() to (150, -355, 389, 0, pi, 0)
""",
            [
                CombinedActions(
                    items=(
                        cartesian_ptp((150, -355, 389), MotionSettings(tcp_velocity_limit=200)),
                        linear((150, -355, 392), MotionSettings(tcp_velocity_limit=10)),
                        cartesian_ptp((150, -355, 389), MotionSettings(tcp_velocity_limit=200)),
                        linear((-95, -363, 387), MotionSettings(tcp_velocity_limit=250)),
                        cartesian_ptp((150, -355, 389), MotionSettings(tcp_velocity_limit=200)),
                    )
                )
            ],
        ),
        (
            """
move via ptp() to (150, -355, 389, 0, pi, 0)
move via line() to (150, -355, 392, 0, pi, 0) with blending(2)
move via ptp() to (150, -355, 389, 0, pi, 0) with velocity(100)
move via line() to (-95, -363, 387) with blending(4)
move via ptp() to (150, -355, 389, 0, pi, 0)
""",
            [
                CombinedActions(
                    items=(
                        cartesian_ptp((150, -355, 389), MotionSettings()),
                        linear((150, -355, 392), MotionSettings(position_zone_radius=2)),
                        cartesian_ptp((150, -355, 389), MotionSettings(tcp_velocity_limit=100)),
                        linear((-95, -363, 387), MotionSettings(position_zone_radius=4)),
                        cartesian_ptp((150, -355, 389), MotionSettings()),
                    )
                )
            ],
        ),
    ],
    ids=["with_tcp", "with_velocity", "with_blending"],
)
# TODO: rebuild this method to check context.motion_group_recordings and remove action_queue._path_history
def test_move(code, expected):
    context = asyncio.run(run_program(code, default_robot="0@controller", default_tcp="Flange"))
    paths = context.action_queue._path_history  # pylint: disable=protected-access

    for path, expected_path in zip(paths, expected):
        assert len(path.motions) == len(expected_path.motions)
        for motion, expected_motion in zip(path.motions, expected_path.motions):
            target_np = (
                motion.target.position
                if isinstance(motion.target, Pose)
                else np.array(motion.target)
            )
            expected_target_np = (
                expected_motion.target.position
                if isinstance(expected_motion.target, Pose)
                else np.array(expected_motion.target)
            )

            assert np.allclose(target_np, expected_target_np)
            print(motion.settings)
            print(expected_motion.settings)
            assert motion.settings == expected_motion.settings
            assert motion.type == expected_motion.type
