"""Integration test guarding the local frame math against the NOVA API.

`nova.datasets` resolves frame chains locally instead of calling the API, so nothing in the
unit tests would notice if the SDK and the server disagreed on how poses compose. This test
is the only thing that does.
"""

import pytest

from nova import Nova
from nova import datasets as ds
from nova.config import CELL_NAME
from nova.types import Pose

pytestmark = pytest.mark.integration

# "default" / revision 1 is the dataset seeded on a fresh cell; "fixture" sits on "table",
# so resolving it exercises a two level chain.
DATASET = "default"
REVISION = 1
FRAME = "fixture"
POSE = "fixture-slot-a"


async def test_as_world_matches_the_api():
    async with Nova() as nova:
        dataset = await ds.fetch(nova, DATASET, revision=REVISION)
        pose = dataset.poses[POSE]

        resolved = await nova.api.datasets_api.resolve_dataset_frame_pose(
            cell=CELL_NAME,
            dataset=DATASET,
            revision=REVISION,
            frame=FRAME,
            poses=[pose.pose.to_api_model()],
        )

    assert pose.as_world() == Pose.from_api_model(resolved[0])


async def test_localizing_matches_the_api():
    async with Nova() as nova:
        dataset = await ds.fetch(nova, DATASET, revision=REVISION)
        world = dataset.poses[POSE].as_world()

        localized = await nova.api.datasets_api.localize_dataset_frame_pose(
            cell=CELL_NAME,
            dataset=DATASET,
            revision=REVISION,
            frame=FRAME,
            poses=[world.to_api_model()],
        )

    assert ~dataset.frames[FRAME].as_world() @ world == Pose.from_api_model(localized[0])
