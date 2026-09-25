"""The TCP-error views must show every logged series, and not mix units.

A ``TimeSeriesView`` shares one Y axis across everything in it, so grouping is
not cosmetic: millimetres and radians together leave the radian series flattened
onto zero by the millimetre scale, reading as "no error" whatever its real value.

Content filters are strings matched at view time, so a series logged under a path
no view covers simply never appears — no error, no warning, just a missing trace.
Both tests therefore drive the real producer and check what it emits, rather than
asserting filter strings against each other.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any
from unittest.mock import MagicMock

from novapolicy.rerun import target_tracking
import novapolicy.rerun.blueprint as blueprint_module
import rerun as rr
import rerun.blueprint as rrb

if TYPE_CHECKING:
    import pytest

MG_ID = "0@ur10e"

# Rerun escapes "@" when it stores an entity path, so a filter has to spell the
# motion-group id the escaped way while the producer passes it raw. Comparing the
# two means normalising one onto the other.
_ESCAPED_MG_ID = rr.escape_entity_path_part(MG_ID)


def _error_views(monkeypatch: pytest.MonkeyPatch) -> list[tuple[str, list[str]]]:
    """The TCP-error ``TimeSeriesView``s, as ``(name, contents)``."""
    captured: list[tuple[str, list[str]]] = []
    real_view = rrb.TimeSeriesView

    def spy(**kwargs: Any) -> Any:
        captured.append((str(kwargs.get("name")), list(kwargs.get("contents") or [])))
        return real_view(**kwargs)

    monkeypatch.setattr(blueprint_module.rrb, "TimeSeriesView", spy)
    monkeypatch.setattr(blueprint_module.rr, "send_blueprint", MagicMock())
    blueprint_module.send_blueprint([MG_ID], [], recording=None)
    return [(name, contents) for name, contents in captured if "error" in name.lower()]


def _logged_tcp_error_paths(monkeypatch: pytest.MonkeyPatch) -> set[str]:
    """Entity paths ``log_tcp_tracking`` really writes under ``tcp_error``."""
    spy = MagicMock()
    monkeypatch.setattr(target_tracking, "rr", spy)

    pose = MagicMock()
    pose.position = (1.0, 2.0, 3.0)
    pose.orientation = (0.1, 0.2, 0.3)
    target_tracking.log_tcp_tracking(
        MG_ID, [1.5, 2.5, 3.5, 0.11, 0.22, 0.33], pose, 0, start_time=0.0, recording=None
    )

    return {
        call.args[0].replace(MG_ID, _ESCAPED_MG_ID)
        for call in spy.log.call_args_list
        if call.args and "/tcp_error/" in call.args[0]
    }


def _covers(pattern: str, path: str) -> bool:
    """Rerun content-filter semantics, narrowed to what this blueprint uses."""
    if pattern.endswith("/**"):
        return path.startswith(pattern[: -len("**")])
    return path == pattern


def test_every_logged_tcp_error_series_appears_in_a_view(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A series no view covers is silently invisible in the viewer.

    The norms (``position_norm_mm`` / ``orientation_norm_rad``) are siblings of the
    ``position`` and ``orientation`` groups rather than children, so a filter of
    just ``position/**`` drops them — and the norm is the one series that shows a
    constant tracking lag as a constant. This also fails if the filters forget to
    escape the motion-group id, since then they match nothing at all.
    """
    patterns = [p for _, contents in _error_views(monkeypatch) for p in contents]
    logged = _logged_tcp_error_paths(monkeypatch)

    assert logged, "the producer logged nothing to check against"
    assert [p for p in sorted(logged) if not any(_covers(f, p) for f in patterns)] == []


def test_tcp_error_position_and_orientation_do_not_share_an_axis(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Millimetres and radians on one Y axis make the radian series unreadable."""
    views = _error_views(monkeypatch)
    logged = _logged_tcp_error_paths(monkeypatch)
    assert views, "no TCP error view was built"

    checked = 0
    for name, contents in views:
        shown = {p for p in logged if any(_covers(f, p) for f in contents)}
        if not shown:
            continue
        checked += 1
        units = {"mm" if "position" in p else "rad" for p in shown}
        assert len(units) == 1, f"{name} mixes units: {sorted(shown)}"
    assert checked == 2, "expected a position view and an orientation view"
