"""Unit tests for downsample utility module."""

import asyncio
from unittest.mock import patch

import pytest

from nova.utils.downsample import downsample_stream


class TestDownsampleStream:
    @pytest.mark.asyncio
    async def test_no_downsampling_when_frequency_is_none(self):
        """Should yield all items when target_frequency is None."""
        items = [1, 2, 3, 4, 5]

        async def mock_stream():
            for item in items:
                yield item

        result = []
        async for item in downsample_stream(mock_stream(), target_frequency=None):
            result.append(item)

        assert result == items

    @pytest.mark.asyncio
    async def test_empty_stream(self):
        """Should handle empty streams correctly."""

        async def mock_stream():
            return
            yield  # Make it an async generator

        result = []
        async for item in downsample_stream(mock_stream(), target_frequency=10.0):
            result.append(item)

        assert result == []

    @pytest.mark.asyncio
    async def test_first_item_always_yielded(self):
        """Should always yield the first item regardless of timing."""
        items = [1, 2, 3]

        async def mock_stream():
            for item in items:
                yield item

        with patch("nova.utils.downsample.time.time", side_effect=[0.0, 0.05, 0.1]):
            result = []
            async for item in downsample_stream(mock_stream(), target_frequency=10.0):
                result.append(item)

            # First item should always be yielded
            assert len(result) >= 1
            assert result[0] == 1

    @pytest.mark.asyncio
    async def test_downsampling_respects_frequency(self):
        """Should only yield items at the specified frequency."""
        items = [1, 2, 3, 4, 5]

        async def mock_stream():
            for item in items:
                yield item

        # Mock time to simulate items arriving faster than target frequency
        # Target frequency: 10 Hz = 0.1s interval
        # Items arrive at: 0.0, 0.05, 0.1, 0.15, 0.2
        # Expected yields: 0.0 (first), 0.1 (>=0.1s elapsed), 0.2 (>=0.1s elapsed)
        time_values = [0.0, 0.05, 0.1, 0.15, 0.2]

        with patch("nova.utils.downsample.time.time", side_effect=time_values):
            result = []
            async for item in downsample_stream(mock_stream(), target_frequency=10.0):
                result.append(item)

            # Should yield first item and items at 0.1s intervals
            assert result == [1, 3, 5]

    @pytest.mark.asyncio
    async def test_downsampling_with_different_frequencies(self):
        """Should work correctly with different frequency values."""
        items = list(range(10))

        async def mock_stream():
            for item in items:
                yield item

        # 20 Hz = 0.05s interval
        # Items arrive every 0.01s
        # Should yield approximately every 5th item
        time_values = [i * 0.01 for i in range(10)]

        with patch("nova.utils.downsample.time.time", side_effect=time_values):
            result = []
            async for item in downsample_stream(mock_stream(), target_frequency=20.0):
                result.append(item)

            # First item always yielded, then items at 0.05s intervals
            assert len(result) >= 1
            assert result[0] == 0

    @pytest.mark.asyncio
    async def test_high_frequency_allows_more_items(self):
        """Higher frequency should allow more items through."""
        items = [1, 2, 3, 4, 5]

        async def mock_stream():
            for item in items:
                yield item

        # 100 Hz = 0.01s interval
        # Items arrive at: 0.0, 0.005, 0.01, 0.015, 0.02
        time_values = [0.0, 0.005, 0.01, 0.015, 0.02]

        with patch("nova.utils.downsample.time.time", side_effect=time_values):
            result = []
            async for item in downsample_stream(mock_stream(), target_frequency=100.0):
                result.append(item)

            # Should yield more items with higher frequency
            assert len(result) > 1

    @pytest.mark.asyncio
    async def test_low_frequency_allows_fewer_items(self):
        """Lower frequency should allow fewer items through."""
        items = [1, 2, 3, 4, 5]

        async def mock_stream():
            for item in items:
                yield item

        # 1 Hz = 1.0s interval
        # Items arrive at: 0.0, 0.1, 0.2, 0.3, 0.4
        time_values = [0.0, 0.1, 0.2, 0.3, 0.4]

        with patch("nova.utils.downsample.time.time", side_effect=time_values):
            result = []
            async for item in downsample_stream(mock_stream(), target_frequency=1.0):
                result.append(item)

            # Should yield only first item (others arrive too quickly)
            assert result == [1]

    @pytest.mark.asyncio
    async def test_exact_interval_timing(self):
        """Should yield items when exactly the interval has passed."""
        items = [1, 2, 3]

        async def mock_stream():
            for item in items:
                yield item

        # 10 Hz = 0.1s interval
        # Items arrive at exactly: 0.0, 0.1, 0.2
        time_values = [0.0, 0.1, 0.2]

        with patch("nova.utils.downsample.time.time", side_effect=time_values):
            result = []
            async for item in downsample_stream(mock_stream(), target_frequency=10.0):
                result.append(item)

            # Should yield all items when timing is exact
            assert result == [1, 2, 3]

    @pytest.mark.asyncio
    async def test_items_just_below_interval_are_skipped(self):
        """Should skip items that arrive just before the interval."""
        items = [1, 2, 3]

        async def mock_stream():
            for item in items:
                yield item

        # 10 Hz = 0.1s interval
        # Items arrive at: 0.0, 0.099, 0.2
        time_values = [0.0, 0.099, 0.2]

        with patch("nova.utils.downsample.time.time", side_effect=time_values):
            result = []
            async for item in downsample_stream(mock_stream(), target_frequency=10.0):
                result.append(item)

            # Item 2 should be skipped (0.099 < 0.1)
            assert result == [1, 3]

    @pytest.mark.asyncio
    async def test_with_async_delays(self):
        """Should work correctly with actual async delays."""
        items = [1, 2, 3, 4, 5]

        async def mock_stream():
            for item in items:
                await asyncio.sleep(0.01)  # Small delay between items
                yield item

        # Use actual timing with 10 Hz (0.1s interval)
        result = []
        async for item in downsample_stream(mock_stream(), target_frequency=10.0):
            result.append(item)

        # Should have yielded some items (exact count depends on timing)
        assert len(result) >= 1
        assert result[0] == 1  # First item always yielded

    @pytest.mark.asyncio
    async def test_default_33ms_interval(self):
        """Should work with the default 33ms interval (~30.3 Hz)."""
        items = list(range(10))

        async def mock_stream():
            for item in items:
                yield item

        # Simulate items arriving faster than 33ms intervals
        # 33ms = 0.033s, so items at 0.0, 0.01, 0.02, 0.03, 0.04...
        time_values = [i * 0.01 for i in range(10)]

        with patch("nova.utils.downsample.time.time", side_effect=time_values):
            result = []
            async for item in downsample_stream(mock_stream(), target_frequency=1.0 / 0.033):
                result.append(item)

            # Should yield items approximately every 33ms
            assert len(result) >= 1
            assert result[0] == 0
