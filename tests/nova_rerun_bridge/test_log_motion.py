from pathlib import Path
from unittest.mock import patch

import pytest

from nova_rerun_bridge.trajectory import log_motion
from tests.fixtures.motion_loader import load_motion_data

FIXTURES_DIR = Path(__file__).parent.parent.parent / "tests" / "fixtures" / "recorded_motions"


@pytest.fixture
def motion_data():
    """Load recorded motion data."""
    fixture_path = FIXTURES_DIR / "example_motion.json"
    return load_motion_data(fixture_path)


@pytest.mark.asyncio
async def test_log_motion_with_recorded_data(motion_data):
    """Test log_motion using recorded data."""
    with patch("rerun.log") as mock_log:
        # Call log_motion with recorded data
        log_motion(
            motion_id=motion_data["motion_id"],
            model_from_controller=motion_data["model_from_controller"],
            motion_group=motion_data["motion_group"],
            optimizer_config=motion_data["optimizer_config"],
            trajectory=motion_data["trajectory"],
            collision_scenes=motion_data["collision_scenes"],
            time_offset=0.0,
        )

        # Verify rerun calls
        mock_log.assert_called()
