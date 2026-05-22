"""Rerun visualization for policy execution.

Visualizes in real-time:
- 3D robot meshes moving with actual joint positions (via RobotVisualizer)
- Action chunk targets as orange TCP line strips (via DH FK)
- Actual TCP trail as green path (tracking accuracy)
- Camera images in dedicated 2D panels
- Joint position timeseries
- Inspectable action chunk text logs

Usage: Pass ``viewer=nova.viewers.Rerun()`` to the ``@nova.program`` decorator.
Fully decoupled — zero overhead when no viewer is active.

Module structure:
- constants.py  — shared colors, widths, helpers
- blueprint.py  — Rerun blueprint layout
- observation.py — per-step state logging (robot mesh, trail, joints)
- action_chunk.py — action chunk visualization (line strips + text)
- streaming.py  — continuous state streaming between policy steps
- images.py     — camera image logging
- logger.py     — main PolicyRerunLogger orchestrator
"""

from policy.rerun.logger import PolicyRerunLogger


def _is_rerun_active() -> bool:
    """Check if a Rerun viewer is active."""
    try:
        from nova.viewers import get_viewer_manager  # noqa: PLC0415

        return get_viewer_manager().has_active_viewers
    except (ImportError, AttributeError):
        return False


__all__ = ["PolicyRerunLogger", "_is_rerun_active"]
