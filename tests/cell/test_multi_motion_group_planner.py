"""Tests for MultiMotionGroupPlanner: how it batches an ensemble action list into
the multi-RRT request(s) and interprets the response, over a faked gateway."""

from unittest.mock import AsyncMock, MagicMock

import pytest

from nova import api
from nova.actions.io import ReadAction, io_write
from nova.actions.mock import wait
from nova.actions.motions import multi_collision_free
from nova.cell.multi_motion_group_planner import MultiMotionGroupPlanner
from nova.exceptions import NoInverseKinematicsSolutionFound, PlanTrajectoryFailed
from nova.types import Pose

pytestmark = pytest.mark.asyncio


def _setup() -> api.models.MotionGroupSetup:
    return api.models.MotionGroupSetup(motion_group_model="UniversalRobots_UR5e", cycle_time=4)


def _multi_trajectory(*keys: str, sample: float = 0.0) -> api.models.MultiJointTrajectory:
    return api.models.MultiJointTrajectory(
        joint_positions_by_motion_group_key={key: [[sample] * 6] * 3 for key in keys},
        times=[0.0, 1.0, 2.0],
        locations=[0.0, 1.0, 2.0],
    )


def _gateway(
    response: api.models.MultiSearchCollisionFreeResponse | None = None,
    responses: list[api.models.MultiSearchCollisionFreeResponse] | None = None,
) -> MagicMock:
    gateway = MagicMock()
    gateway.trajectory_planning_api = MagicMock()
    gateway.trajectory_planning_api.search_collision_free_multi_motion_group = (
        AsyncMock(side_effect=responses)
        if responses is not None
        else AsyncMock(return_value=response)
    )
    return gateway


def _ok(*keys: str, sample: float = 0.0) -> api.models.MultiSearchCollisionFreeResponse:
    return api.models.MultiSearchCollisionFreeResponse(
        response=_multi_trajectory(*keys, sample=sample)
    )


def _motion_group(
    api_client: MagicMock,
    group_id: str,
    current_joints: tuple[float, ...] = (0.0,) * 6,
    ik_solutions: list[tuple[float, ...]] | None = None,
    cell: str = "cell",
) -> MagicMock:
    group = MagicMock()
    group.id = group_id
    group._cell = cell
    group._api_client = api_client
    group.get_setup = AsyncMock(return_value=_setup())
    group.joints = AsyncMock(return_value=current_joints)
    group._inverse_kinematics = AsyncMock(return_value=[ik_solutions or []])
    return group


def _request_of(api_client: MagicMock, call_index: int = 0):
    return (
        api_client.trajectory_planning_api.search_collision_free_multi_motion_group.call_args_list[
            call_index
        ].kwargs["multi_search_collision_free_request"]
    )


class TestValidation:
    async def test_no_groups__raises(self):
        with pytest.raises(ValueError, match="At least one motion group"):
            MultiMotionGroupPlanner({})

    async def test_groups_span_cells__raises(self):
        api_client = MagicMock()
        with pytest.raises(ValueError, match="cell-scoped"):
            MultiMotionGroupPlanner(
                {
                    "a": _motion_group(api_client, "a", cell="cell-1"),
                    "b": _motion_group(api_client, "b", cell="cell-2"),
                }
            )

    async def test_no_actions__raises(self):
        api_client = _gateway(_ok("a"))
        planner = MultiMotionGroupPlanner({"a": _motion_group(api_client, "a")})
        with pytest.raises(ValueError, match="No actions"):
            await planner.plan([])

    async def test_targets_not_matching_groups__raises(self):
        api_client = _gateway(_ok("a", "b"))
        planner = MultiMotionGroupPlanner(
            {"a": _motion_group(api_client, "a"), "b": _motion_group(api_client, "b")}
        )
        with pytest.raises(ValueError, match="must match the planner"):
            await planner.plan(multi_collision_free({"a": (1.0,) * 6}))

    async def test_unsupported_action__raises(self):
        api_client = _gateway(_ok("a"))
        planner = MultiMotionGroupPlanner({"a": _motion_group(api_client, "a")})
        with pytest.raises(ValueError, match="not supported"):
            await planner.plan(ReadAction(key="IN#1", device_id="ctrl"))

    async def test_write_action_is_ignored__no_raise_and_pure_trajectory(self):
        # A write is skipped, leaving the trajectory identical to the motion alone.
        api_client = _gateway(_ok("a", sample=1.0))
        planner = MultiMotionGroupPlanner({"a": _motion_group(api_client, "a")})

        with_write = await planner.plan(
            [io_write("OUT#1", True, device_id="ctrl"), multi_collision_free({"a": (1.0,) * 6})]
        )

        assert isinstance(with_write, api.models.MultiJointTrajectory)
        # one RRT call, no extra samples from the write
        assert (
            api_client.trajectory_planning_api.search_collision_free_multi_motion_group.await_count
            == 1
        )
        assert len(with_write.joint_positions_by_motion_group_key["a"]) == 3

    async def test_only_writes__raises(self):
        api_client = _gateway(_ok("a"))
        planner = MultiMotionGroupPlanner({"a": _motion_group(api_client, "a")})
        with pytest.raises(ValueError, match="no motion or wait"):
            await planner.plan([io_write("OUT#1", True, device_id="ctrl")])


class TestRequestAssembly:
    async def test_joint_targets_build_path_definitions(self):
        api_client = _gateway(_ok("a", "b"))
        planner = MultiMotionGroupPlanner(
            {
                "a": _motion_group(api_client, "a", current_joints=(0.1,) * 6),
                "b": _motion_group(api_client, "b", current_joints=(0.2,) * 6),
            }
        )

        result = await planner.plan(
            multi_collision_free({"a": (1.0,) * 6, "b": (2.0,) * 6}),
            start_joint_position={"a": (0.5,) * 6},
        )

        assert isinstance(result, api.models.MultiJointTrajectory)
        paths = _request_of(api_client).path_definitions_by_motion_group_key
        assert set(paths) == {"a", "b"}
        # explicit start honored for "a", current-joints fallback for "b"
        assert tuple(paths["a"].start_joint_position) == (0.5,) * 6
        assert tuple(paths["a"].target_joint_position) == (1.0,) * 6
        assert tuple(paths["b"].start_joint_position) == (0.2,) * 6
        assert tuple(paths["b"].target_joint_position) == (2.0,) * 6
        assert set(_request_of(api_client).motion_group_setups_by_motion_group_key) == {"a", "b"}

    async def test_shared_tcp_and_algorithm_on_action(self):
        api_client = _gateway(_ok("a"))
        group = _motion_group(api_client, "a")
        planner = MultiMotionGroupPlanner({"a": group})

        algorithm = api.models.RRTConnectAlgorithm(max_iterations=42)
        await planner.plan(
            multi_collision_free({"a": (1.0,) * 6}, algorithm=algorithm), tcp="flange"
        )

        group.get_setup.assert_awaited_once_with(tcp_name="flange")
        assert _request_of(api_client).algorithm_settings.max_iterations == 42

    async def test_pose_target_resolves_ik_nearest_to_start(self):
        api_client = _gateway(_ok("a"))
        far, near = (3.0,) * 6, (0.1,) * 6
        group = _motion_group(api_client, "a", current_joints=(0.0,) * 6, ik_solutions=[far, near])
        planner = MultiMotionGroupPlanner({"a": group})

        await planner.plan(multi_collision_free({"a": Pose((1, 2, 3, 4, 5, 6))}), tcp="flange")

        group._inverse_kinematics.assert_awaited_once()
        target = tuple(
            _request_of(api_client).path_definitions_by_motion_group_key["a"].target_joint_position
        )
        assert target == near

    async def test_pose_target_without_tcp__raises(self):
        api_client = _gateway(_ok("a"))
        planner = MultiMotionGroupPlanner({"a": _motion_group(api_client, "a")})
        with pytest.raises(ValueError, match="TCP is required"):
            await planner.plan(multi_collision_free({"a": Pose((1, 2, 3, 4, 5, 6))}))

    async def test_pose_target_no_ik_solution__raises(self):
        api_client = _gateway(_ok("a"))
        group = _motion_group(api_client, "a", ik_solutions=[])
        planner = MultiMotionGroupPlanner({"a": group})
        with pytest.raises(NoInverseKinematicsSolutionFound):
            await planner.plan(multi_collision_free({"a": Pose((1, 2, 3, 4, 5, 6))}), tcp="flange")


class TestBatching:
    async def test_wait_between_motions_concatenates_segments(self):
        # Two distinct RRT segments plus a hold in the middle.
        api_client = _gateway(responses=[_ok("a", sample=1.0), _ok("a", sample=2.0)])
        group = _motion_group(api_client, "a", current_joints=(0.0,) * 6)
        planner = MultiMotionGroupPlanner({"a": group})

        result = await planner.plan(
            [
                multi_collision_free({"a": (1.0,) * 6}),
                wait(0.1),
                multi_collision_free({"a": (2.0,) * 6}),
            ]
        )

        # two RRT calls, one per motion; the wait is planned locally
        assert (
            api_client.trajectory_planning_api.search_collision_free_multi_motion_group.await_count
            == 2
        )
        # 3 + (3 hold - 1 seam) + (3 - 1 seam) = 3 + 2 + 2 = 7 samples on the shared timeline
        samples = result.joint_positions_by_motion_group_key["a"]
        assert len(samples) == 7
        assert len(result.times) == 7
        # second motion's start joints = end of the hold = end of the first segment
        second_start = tuple(
            _request_of(api_client, 1)
            .path_definitions_by_motion_group_key["a"]
            .start_joint_position
        )
        assert second_start == (1.0,) * 6

    async def test_wait_holds_current_joints(self):
        api_client = _gateway(_ok("a"))
        planner = MultiMotionGroupPlanner(
            {"a": _motion_group(api_client, "a", current_joints=(0.7,) * 6)}
        )
        result = await planner.plan(wait(0.1))
        samples = result.joint_positions_by_motion_group_key["a"]
        assert all(tuple(s) == (0.7,) * 6 for s in samples)
        assert all(loc == 0.0 for loc in result.locations)
        api_client.trajectory_planning_api.search_collision_free_multi_motion_group.assert_not_awaited()


class TestResponse:
    async def test_failed_response__raises(self):
        failure = api.models.PlanCollisionFreeFailedResponse(
            error_feedback=api.models.ErrorMaxIterationsExceeded()
        )
        api_client = _gateway(api.models.MultiSearchCollisionFreeResponse(response=failure))
        planner = MultiMotionGroupPlanner(
            {"a": _motion_group(api_client, "a"), "b": _motion_group(api_client, "b")}
        )
        with pytest.raises(PlanTrajectoryFailed, match="a, b"):
            await planner.plan(multi_collision_free({"a": (1.0,) * 6, "b": (1.0,) * 6}))

    async def test_empty_response__raises(self):
        api_client = _gateway(api.models.MultiSearchCollisionFreeResponse(response=None))
        planner = MultiMotionGroupPlanner({"a": _motion_group(api_client, "a")})
        with pytest.raises(ValueError, match="no trajectory"):
            await planner.plan(multi_collision_free({"a": (1.0,) * 6}))
