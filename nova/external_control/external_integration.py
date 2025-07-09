"""Nova External Integration API

This module provides clean interfaces for external tools (VS Code extensions,
future WebSocket/HTTP servers) to control Nova robots. Functions are registered
in the global namespace where external tools can discover and call them.

Design Note: Using global function registration instead of network protocols
avoids security concerns and simplifies integration for this initial implementation.
"""

import sys

from nova.core.playback_control import PlaybackSpeedPercent, RobotId, get_playback_manager


def nova_set_playback_speed(robot_id: str, speed_percent: int) -> dict:
    """Set playback speed for a robot (called from external tools)

    Args:
        robot_id: Unique robot identifier
        speed_percent: Playback speed percent (0-100, will be clamped if outside range)

    Returns:
        dict: Success/error response with standardized format

    Example:
        result = nova_set_playback_speed("robot1", 50)
        if result["success"]:
            print(f"Speed set to {result['speed']}%")
    """
    try:
        # Clamp speed to valid range (0-100)
        clamped_speed_percent = max(0, min(100, int(speed_percent)))

        manager = get_playback_manager()
        manager.set_external_override(
            RobotId(robot_id), PlaybackSpeedPercent(clamped_speed_percent)
        )

        return {
            "success": True,
            "robot_id": robot_id,
            "speed": clamped_speed_percent,
            "message": f"Playback speed set to {clamped_speed_percent}%",
        }

    except Exception as e:
        return {
            "success": False,
            "robot_id": robot_id,
            "error": str(e),
            "message": f"Failed to set playback speed: {e}",
        }


def nova_pause_robot(robot_id: str) -> dict:
    """Pause robot execution (called from external tools)

    Args:
        robot_id: Unique robot identifier

    Returns:
        dict: Success/error response with standardized format

    Example:
        result = nova_pause_robot("robot1")
        if result["success"]:
            print("Robot paused")
    """
    try:
        manager = get_playback_manager()
        manager.pause(RobotId(robot_id))

        return {
            "success": True,
            "robot_id": robot_id,
            "state": "paused",
            "message": f"Robot {robot_id} paused",
        }

    except Exception as e:
        return {
            "success": False,
            "robot_id": robot_id,
            "error": str(e),
            "message": f"Failed to pause robot: {e}",
        }


def nova_resume_robot(robot_id: str) -> dict:
    """Resume robot execution (called from external tools)

    Args:
        robot_id: Unique robot identifier

    Returns:
        dict: Success/error response with standardized format

    Example:
        result = nova_resume_robot("robot1")
        if result["success"]:
            print("Robot resumed")
    """
    try:
        manager = get_playback_manager()
        manager.resume(RobotId(robot_id))

        return {
            "success": True,
            "robot_id": robot_id,
            "state": "playing",
            "message": f"Robot {robot_id} resumed",
        }

    except Exception as e:
        return {
            "success": False,
            "robot_id": robot_id,
            "error": str(e),
            "message": f"Failed to resume robot: {e}",
        }


def nova_get_available_robots() -> dict:
    """Get list of available robots (for external UIs)

    Returns:
        dict: Success response with list of robot IDs and their current states

    Example:
        result = nova_get_available_robots()
        if result["success"]:
            for robot in result["robots"]:
                print(f"Robot {robot['id']}: {robot['speed']}% speed")
    """
    try:
        manager = get_playback_manager()
        robot_ids = manager.get_all_robots()

        robots = []
        for robot_id in robot_ids:
            current_speed = manager.get_effective_speed(RobotId(robot_id))
            current_state = manager.get_effective_state(RobotId(robot_id))

            robots.append(
                {
                    "id": robot_id,
                    "speed": int(current_speed),  # Now returning integer percent directly
                    "state": current_state.value,
                    "speed_percent": f"{int(current_speed)}%",
                }
            )

        return {
            "success": True,
            "robots": robots,
            "count": len(robots),
            "message": f"Found {len(robots)} robots with playback settings",
        }

    except Exception as e:
        return {
            "success": False,
            "robots": [],
            "error": str(e),
            "message": f"Failed to get available robots: {e}",
        }


def register_external_control_functions():
    """Register global functions for external control

    Makes Nova playback control functions available to external tools like
    VS Code extensions. Functions are registered in the global namespace
    where external tools can discover and call them.

    Design Note: Using global function registration instead of network protocols
    avoids security concerns and simplifies integration for this initial implementation.
    """

    # Get the global namespace of the main module
    import __main__

    if hasattr(__main__, "__dict__"):
        main_globals = __main__.__dict__
    else:
        main_globals = globals()

    # Register functions globally for external tool discovery
    main_globals["nova_set_playback_speed"] = nova_set_playback_speed
    main_globals["nova_pause_robot"] = nova_pause_robot
    main_globals["nova_resume_robot"] = nova_resume_robot
    main_globals["nova_get_available_robots"] = nova_get_available_robots

    # Also register in current module for Python extension access
    current_module = sys.modules[__name__]
    setattr(current_module, "nova_set_playback_speed", nova_set_playback_speed)
    setattr(current_module, "nova_pause_robot", nova_pause_robot)
    setattr(current_module, "nova_resume_robot", nova_resume_robot)
    setattr(current_module, "nova_get_available_robots", nova_get_available_robots)
