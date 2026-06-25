"""Tests for exact source-location capture on actions.

These tests run from a real source file, so the interpreter provides position
information (PEP 657) and a :class:`~nova.utils.SourceLocation` is captured.
"""

from nova import utils
from nova.actions.motions import Linear, circular, lin
from nova.types import Pose
from nova.utils import SourceLocation


def test_linear_captures_single_line_span():
    motion = lin((1, 2, 3, 4, 5, 6))

    loc = motion.source_location
    assert loc is not None
    assert loc.start_line == loc.end_line
    assert loc.start_column is not None and loc.end_column is not None
    assert loc.end_column > loc.start_column
    # line_number is kept for backward compatibility and matches the span start
    assert motion.metas["line_number"] == loc.start_line


def test_circular_call_spans_multiple_lines():
    # fmt: off so ruff (configured with skip-magic-trailing-comma) keeps this call
    # expanded across lines, exercising a genuine multi-line source span.
    # fmt: off
    motion = circular(
        target=(1, 2, 3, 4, 5, 6),
        intermediate=(7, 8, 9, 10, 11, 12),
    )
    # fmt: on

    loc = motion.source_location
    assert loc is not None
    # The whole circular(...) call is one selection spanning several lines,
    # not two separate targets for the intermediate and target poses.
    assert loc.end_line is not None and loc.start_line is not None
    assert loc.end_line > loc.start_line
    assert loc.start_column is not None and loc.end_column is not None


def test_source_location_stored_as_plain_dict_for_round_tripping():
    motion = lin((1, 2, 3))
    # Stored as a plain dict in metas so it survives model_dump/from_dict cleanly.
    assert isinstance(motion.metas["source_location"], dict)
    assert set(motion.metas["source_location"]) == {
        "start_line",
        "end_line",
        "start_column",
        "end_column",
    }


def test_source_location_none_when_not_captured():
    # Direct construction (not via a factory) has no captured source location.
    motion = Linear(target=Pose((1, 2, 3, 4, 5, 6)))
    assert motion.source_location is None


def _wrapper() -> dict:
    return utils.get_caller_metas()


def test_get_caller_metas_targets_the_call_site_of_the_wrapper():
    metas = _wrapper()
    assert "source_location" in metas
    assert metas["line_number"] == metas["source_location"]["start_line"]


def test_source_location_for_synthetic_frame_returns_none():
    # exec'd code has a synthetic filename ("<string>") with no real source.
    captured: dict = {}
    exec(
        "from nova import utils\n"
        "captured['loc'] = utils.source_location_for_frame(__import__('inspect').currentframe())",
        {"captured": captured},
    )
    assert captured["loc"] is None


def test_source_location_model_round_trips():
    loc = SourceLocation(start_line=3, end_line=5, start_column=4, end_column=10)
    assert SourceLocation.model_validate(loc.model_dump()) == loc
