"""
Rerun integration for Nova.

This module provides automatic rerun logging functionality.
It integrates with Nova's motion planning to provide visualization
without requiring any code changes from users.
"""

import asyncio
import os
from typing import Any, Optional

_rerun_enabled = os.getenv("NOVA_RERUN_ENABLED", "true").lower() in ("true", "1", "yes")
_bridge_instance: Optional[Any] = None
_bridge_setup_done = False


def is_rerun_enabled() -> bool:
    """Check if rerun logging is enabled."""
    return _rerun_enabled


def enable_rerun(enabled: bool = True) -> None:
    """Enable or disable rerun logging globally."""
    global _rerun_enabled
    _rerun_enabled = enabled


def configure_rerun(enabled: bool = True) -> None:
    """Configure rerun logging (alias for enable_rerun for backward compatibility)."""
    enable_rerun(enabled)


def disable_rerun() -> None:
    """Disable rerun logging."""
    enable_rerun(False)


def _get_bridge_instance(motion_group: Any) -> Optional[Any]:
    """Get or create the NovaRerunBridge instance."""
    global _bridge_instance

    if not _rerun_enabled:
        return None

    if _bridge_instance is None:
        try:
            from nova.core.nova import Nova
            from nova_rerun_bridge import NovaRerunBridge

            # Create a Nova instance using the API gateway from the motion group
            api_gateway = getattr(motion_group, "_api_gateway", None)
            if api_gateway is None:
                return None

            # Create Nova instance with same configuration as the API gateway
            nova_instance = Nova(
                host=api_gateway._host,
                access_token=getattr(api_gateway, "_access_token", None),
                username=getattr(api_gateway, "_username", None),
                password=getattr(api_gateway, "_password", None),
                version=getattr(api_gateway, "_version", "v1"),
                verify_ssl=getattr(api_gateway, "_verify_ssl", True),
            )

            _bridge_instance = NovaRerunBridge(nova_instance, spawn=True)
        except ImportError:
            # rerun bridge not available - fail silently
            return None
        except Exception:
            # any other error creating bridge - fail silently
            return None

    return _bridge_instance


async def _ensure_blueprint_setup(bridge: Any) -> None:
    """Ensure the blueprint is set up for the bridge."""
    global _bridge_setup_done

    if not _bridge_setup_done and bridge is not None:
        try:
            await bridge.setup_blueprint()
            _bridge_setup_done = True
        except Exception:
            # Blueprint setup failed - fail silently
            pass


async def log_trajectory_async(trajectory: Any, motion_group: Any, tcp: str) -> None:
    """
    Log a trajectory to rerun asynchronously.

    Args:
        trajectory: The joint trajectory to log
        motion_group: The motion group that executed the trajectory
        tcp: The TCP name used for the trajectory
    """
    if not _rerun_enabled:
        return

    bridge = _get_bridge_instance(motion_group)
    if bridge is None:
        return

    try:
        # Ensure blueprint is set up
        await _ensure_blueprint_setup(bridge)

        # Log the trajectory
        await bridge.log_trajectory(joint_trajectory=trajectory, tcp=tcp, motion_group=motion_group)
    except Exception:
        # Logging failed - fail silently to not break user workflows
        pass


def log_trajectory(trajectory: Any, motion_group: Any, tcp: str) -> None:
    """
    Synchronous wrapper for log_trajectory_async.

    Args:
        trajectory: The joint trajectory to log
        motion_group: The motion group that executed the trajectory
        tcp: The TCP name used for the trajectory
    """
    if not _rerun_enabled:
        return

    try:
        # Create and run the async task
        asyncio.create_task(log_trajectory_async(trajectory, motion_group, tcp))
    except Exception:
        # Task creation failed - fail silently
        pass


__all__ = [
    "is_rerun_enabled",
    "enable_rerun",
    "configure_rerun",
    "disable_rerun",
    "log_trajectory_async",
    "log_trajectory",
]
