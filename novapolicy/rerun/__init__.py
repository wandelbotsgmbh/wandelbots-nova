"""Live Rerun visualization of policy execution and jogging (``docs/rerun.md``).

Active only when ``viewer=nova.viewers.Rerun()`` is passed to ``@nova.program``; otherwise
nothing here runs.
"""

from novapolicy.rerun.logger import PolicyRerunLogger


def _is_rerun_active() -> bool:
    """Check if a Rerun viewer is active."""
    try:
        from nova.viewers import Rerun, get_viewer_manager  # ruff: ignore[import-outside-top-level]

        viewer = get_viewer_manager().get_viewer(Rerun)
        return viewer is not None and viewer.is_configured
    except (ImportError, AttributeError):
        return False


__all__ = ["PolicyRerunLogger", "_is_rerun_active"]
