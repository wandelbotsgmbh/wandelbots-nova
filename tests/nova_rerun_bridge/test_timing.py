"""Where a trajectory lands on the timeline when several plans follow each other."""

from nova_rerun_bridge.trajectory import MotionGroupTimeline, TimingMode

ARM = "0@robot"
OTHER = "1@robot"


class TestContinue:
    def test_the_first_trajectory_starts_at_zero(self):
        timeline = MotionGroupTimeline()
        assert timeline.start(ARM, TimingMode.CONTINUE, 0.0) == 0.0

    def test_the_next_one_starts_where_the_last_ended(self):
        timeline = MotionGroupTimeline()
        start = timeline.start(ARM, TimingMode.CONTINUE, 0.0)
        timeline.advance(ARM, start, 4.0, TimingMode.CONTINUE)

        assert timeline.start(ARM, TimingMode.CONTINUE, 0.0) == 4.0

    def test_an_offset_is_a_gap_after_the_last_one(self):
        timeline = MotionGroupTimeline()
        timeline.advance(ARM, 0.0, 4.0, TimingMode.CONTINUE)

        assert timeline.start(ARM, TimingMode.CONTINUE, 1.5) == 5.5

    def test_each_motion_group_keeps_its_own_clock(self):
        """Two robots moving at once both start at zero, and stay independent."""
        timeline = MotionGroupTimeline()
        timeline.advance(ARM, 0.0, 4.0, TimingMode.CONTINUE)

        assert timeline.start(OTHER, TimingMode.CONTINUE, 0.0) == 0.0


class TestTheOtherModes:
    def test_reset_starts_at_the_offset_and_carries_on(self):
        timeline = MotionGroupTimeline()
        timeline.advance(ARM, 0.0, 4.0, TimingMode.CONTINUE)

        start = timeline.start(ARM, TimingMode.RESET, 0.0)
        assert start == 0.0
        timeline.advance(ARM, start, 2.0, TimingMode.RESET)
        assert timeline.start(ARM, TimingMode.CONTINUE, 0.0) == 2.0

    def test_override_is_the_same_rule(self):
        timeline = MotionGroupTimeline()
        timeline.advance(ARM, 0.0, 4.0, TimingMode.CONTINUE)

        start = timeline.start(ARM, TimingMode.OVERRIDE, 1.0)
        assert start == 1.0
        timeline.advance(ARM, start, 2.0, TimingMode.OVERRIDE)
        assert timeline.start(ARM, TimingMode.CONTINUE, 0.0) == 3.0

    def test_sync_places_a_trajectory_without_moving_the_clock(self):
        """What lines two motion groups up at one instant."""
        timeline = MotionGroupTimeline()
        timeline.advance(ARM, 0.0, 4.0, TimingMode.CONTINUE)

        start = timeline.start(ARM, TimingMode.SYNC, 10.0)
        assert start == 10.0
        timeline.advance(ARM, start, 2.0, TimingMode.SYNC)
        assert timeline.start(ARM, TimingMode.CONTINUE, 0.0) == 4.0
