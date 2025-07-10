"""
Tests for pause/resume functionality in the movement controller.

These tests verify that the movement controller properly detects and responds to
runtime pause/resume state changes from external tools like VS Code extensions.
"""

import asyncio
import unittest
from unittest.mock import AsyncMock, Mock

import pytest
import wandelbots_api_client as wb

from nova.actions import CombinedActions, MovementControllerContext
from nova.actions.motions import cartesian_ptp
from nova.core.movement_controller import move_forward
from nova.core.playback_control import (
    MotionGroupId,
    PlaybackSpeedPercent,
    PlaybackState,
    get_playback_manager,
)


class TestMovementControllerPauseResume(unittest.TestCase):
    """Test pause/resume functionality in the movement controller."""

    def setUp(self):
        """Set up test fixtures."""
        actions = [cartesian_ptp((100, 200, 300, 0, 0, 0))]
        self.context = MovementControllerContext(
            motion_group_id="test_robot",
            motion_id="test_motion",
            effective_speed=50,
            method_speed=None,
            combined_actions=CombinedActions(items=tuple(actions)),
        )
        self.manager = get_playback_manager()
        self.robot_id = MotionGroupId(self.context.motion_group_id)

        # Clear any existing state
        self.manager.clear_external_override(self.robot_id)

    def tearDown(self):
        """Clean up after tests."""
        self.manager.clear_external_override(self.robot_id)

    @pytest.mark.asyncio
    async def test_pause_during_execution_detected(self):
        """Test that pause requests are detected and queued during execution."""
        # Create the movement controller
        controller_fn = move_forward(self.context)

        # Mock response stream with responses that keep the controller running
        mock_response_stream = AsyncMock()
        mock_responses = [
            # Initialization success
            Mock(
                actual_instance=wb.models.InitializeMovementResponse(
                    init_response=wb.models.InitializeMovementResponseInitResponse(succeeded=True)
                )
            ),
            # Speed confirmation
            Mock(
                actual_instance=wb.models.PlaybackSpeedResponse(
                    playback_speed_response=wb.models.PlaybackSpeedResponsePlaybackSpeedResponse(
                        requested_value=50
                    )
                )
            ),
            # Keep running for a bit, then standstill
            Mock(
                actual_instance=wb.models.Standstill(
                    standstill=wb.models.StandstillStandstill(
                        reason=wb.models.StandstillReason.REASON_MOTION_ENDED,
                        location=1.0,
                        state=wb.models.MovementState(),
                    )
                )
            ),
        ]
        mock_response_stream.__aiter__ = AsyncMock(return_value=iter(mock_responses))

        # Collect requests from the controller
        requests = []

        async def collect_requests():
            async for request in controller_fn(mock_response_stream):
                requests.append(request)

        # Create task for controller
        controller_task = asyncio.create_task(collect_requests())

        # Wait a bit for controller to initialize
        await asyncio.sleep(0.1)

        # Send pause command
        self.manager.pause(self.robot_id)

        # Wait a bit more for pause to be processed
        await asyncio.sleep(0.1)

        # Complete the controller
        await controller_task

        # Verify we got expected requests
        self.assertGreater(len(requests), 2)  # At least init, speed, start

        # Check for pause request
        pause_requests = [r for r in requests if isinstance(r, wb.models.PauseMovementRequest)]
        self.assertEqual(len(pause_requests), 1)

    @pytest.mark.asyncio
    async def test_resume_after_pause_detected(self):
        """Test that resume requests are detected and queued after pause."""
        controller_fn = move_forward(self.context)

        # Mock response stream with pause response
        mock_response_stream = AsyncMock()
        mock_responses = [
            # Initialization success
            Mock(
                actual_instance=wb.models.InitializeMovementResponse(
                    init_response=wb.models.InitializeMovementResponseInitResponse(succeeded=True)
                )
            ),
            # Pause confirmation
            Mock(
                actual_instance=wb.models.PauseMovementResponse(
                    pause_response=wb.models.PauseMovementResponsePauseResponse(succeeded=True)
                )
            ),
            # Final standstill
            Mock(
                actual_instance=wb.models.Standstill(
                    standstill=wb.models.StandstillStandstill(
                        reason=wb.models.StandstillReason.REASON_MOTION_ENDED,
                        location=1.0,
                        state=wb.models.MovementState(),
                    )
                )
            ),
        ]
        mock_response_stream.__aiter__ = AsyncMock(return_value=iter(mock_responses))

        # Collect requests
        requests = []

        async def collect_requests():
            async for request in controller_fn(mock_response_stream):
                requests.append(request)

        controller_task = asyncio.create_task(collect_requests())

        # Wait for initialization
        await asyncio.sleep(0.1)

        # Pause first
        self.manager.pause(self.robot_id)
        await asyncio.sleep(0.05)

        # Then resume
        self.manager.resume(self.robot_id)
        await asyncio.sleep(0.05)

        # Complete controller
        await controller_task

        # Check for both pause and resume requests
        pause_requests = [r for r in requests if isinstance(r, wb.models.PauseMovementRequest)]
        start_requests = [r for r in requests if isinstance(r, wb.models.StartMovementRequest)]

        self.assertGreaterEqual(len(pause_requests), 1)
        self.assertGreaterEqual(len(start_requests), 2)  # Original start + resume

    def test_effective_state_changes_detected(self):
        """Test that effective state changes are properly detected."""
        # Initial state should be PLAYING
        self.assertEqual(self.manager.get_effective_state(self.robot_id), PlaybackState.PLAYING)

        # Pause should change state
        self.manager.pause(self.robot_id)
        self.assertEqual(self.manager.get_effective_state(self.robot_id), PlaybackState.PAUSED)

        # Resume should restore PLAYING state
        self.manager.resume(self.robot_id)
        self.assertEqual(self.manager.get_effective_state(self.robot_id), PlaybackState.PLAYING)

    def test_pause_preserves_speed(self):
        """Test that pause preserves current effective speed."""
        # Set a specific speed
        speed = PlaybackSpeedPercent(75)
        self.manager.set_external_override(self.robot_id, speed)
        self.assertEqual(self.manager.get_effective_speed(self.robot_id), speed)

        # Pause should preserve speed
        self.manager.pause(self.robot_id)
        self.assertEqual(self.manager.get_effective_speed(self.robot_id), speed)
        self.assertEqual(self.manager.get_effective_state(self.robot_id), PlaybackState.PAUSED)

        # Resume should maintain speed
        self.manager.resume(self.robot_id)
        self.assertEqual(self.manager.get_effective_speed(self.robot_id), speed)
        self.assertEqual(self.manager.get_effective_state(self.robot_id), PlaybackState.PLAYING)


if __name__ == "__main__":
    unittest.main()
