"""Integration test guarding the local frame math against the NOVA API.

`nova.datasets` resolves frame chains locally instead of calling the API, so nothing in the
unit tests would notice if the SDK and the server disagreed on how poses compose. This test
is the only thing that does.
"""

import pytest

from nova import Nova, api
from nova import datasets as ds
from nova.config import CELL_NAME
from nova.types import Pose

pytestmark = pytest.mark.integration

# "default" / revision 1 is the dataset seeded on a fresh cell. Its contents differ between
# NOVA versions, so the test picks the pose to check from the dataset itself.
DATASET = "default"
REVISION = 1


def _chain_depth(dataset: ds.Dataset, frame: str) -> int:
    depth = 0
    current: str | None = frame
    while current is not None and current in dataset.frames and depth <= len(dataset.frames):
        depth += 1
        current = dataset.frames[current].reference_frame
    return depth


def _frame_relative_pose(dataset: ds.Dataset) -> tuple[str, str]:
    """The (pose, frame) whose frame has the deepest chain, to exercise composition."""
    candidates = [
        (name, pose.frame.frame)
        for name, pose in dataset.poses.items()
        if isinstance(pose.frame, api.models.DatasetFrameReference)
    ]
    if not candidates:
        pytest.skip(f"dataset {DATASET!r} has no pose taught in a dataset frame")
    return max(candidates, key=lambda candidate: _chain_depth(dataset, candidate[1]))


async def test_as_world_matches_the_api():
    async with Nova() as nova:
        dataset = await ds.fetch(nova, DATASET, revision=REVISION)
        name, frame = _frame_relative_pose(dataset)
        pose = dataset.poses[name]

        resolved = await nova.api.datasets_api.resolve_dataset_frame_pose(
            cell=CELL_NAME,
            dataset=DATASET,
            revision=REVISION,
            frame=frame,
            poses=[pose.pose.to_api_model()],
        )

    assert pose.as_world() == Pose.from_api_model(resolved[0])


async def test_localizing_matches_the_api():
    async with Nova() as nova:
        dataset = await ds.fetch(nova, DATASET, revision=REVISION)
        name, frame = _frame_relative_pose(dataset)
        world = dataset.poses[name].as_world()

        localized = await nova.api.datasets_api.localize_dataset_frame_pose(
            cell=CELL_NAME,
            dataset=DATASET,
            revision=REVISION,
            frame=frame,
            poses=[world.to_api_model()],
        )

    assert ~dataset.frames[frame].as_world() @ world == Pose.from_api_model(localized[0])
