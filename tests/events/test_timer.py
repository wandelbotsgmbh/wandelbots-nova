from datetime import datetime, timedelta

import pytest
from freezegun import freeze_time

from nova.events import Timer


class TestTimer:
    def test_init(self):
        """Test that a timer initializes with None values."""
        timer = Timer()
        assert timer.start_time is None
        assert timer.stop_time is None
        assert not timer.is_running()

    def test_start(self):
        """Test that start() sets start_time and returns it."""
        timer = Timer()
        test_time = datetime(2025, 1, 1, 0, 0, 0)
        with freeze_time(test_time):
            result = timer.start()
            assert result == test_time
            assert timer.start_time == test_time
            assert timer.is_running()

    def test_start_already_running(self):
        """Test that starting an already running timer raises an error."""
        timer = Timer()
        timer.start()
        with pytest.raises(RuntimeError, match="Timer is already running"):
            timer.start()

    def test_stop(self):
        """Test that stop() sets stop_time and returns it."""
        timer = Timer()
        timer.start()
        test_time = datetime(2025, 1, 1, 0, 0, 0)
        with freeze_time(test_time):
            result = timer.stop()
            assert result == test_time
            assert timer.stop_time == test_time
            assert not timer.is_running()

    def test_stop_not_running(self):
        """Test that stopping a non-running timer raises an error."""
        timer = Timer()
        with pytest.raises(RuntimeError, match="Timer is not running"):
            timer.stop()

    def test_reset(self):
        """Test that reset() resets both time values."""
        timer = Timer()
        timer.start()
        timer.stop()
        timer.reset()
        assert timer.start_time is None
        assert timer.stop_time is None
        assert not timer.is_running()

    def test_elapsed_while_running(self):
        """Test elapsed() while the timer is running."""
        timer = Timer()
        with freeze_time("2025-01-01 00:00:00") as frozen_time:
            timer.start()
            frozen_time.tick(timedelta(seconds=30))
            assert timer.elapsed() == timedelta(seconds=30)

    def test_elapsed_after_stopped(self):
        """Test elapsed() after the timer has been stopped."""
        timer = Timer()
        with freeze_time("2025-01-01 00:00:00") as frozen_time:
            timer.start()
            frozen_time.tick(timedelta(seconds=45))
            timer.stop()
            # Additional time passing shouldn't affect result
            frozen_time.tick(timedelta(seconds=15))
            assert timer.elapsed() == timedelta(seconds=45)

    def test_is_running(self):
        """Test the is_running() method in different states."""
        timer = Timer()
        assert not timer.is_running()

        timer.start()
        assert timer.is_running()

        timer.stop()
        assert not timer.is_running()

        timer.reset()
        assert not timer.is_running()
