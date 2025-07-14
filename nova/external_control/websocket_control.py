"""Nova WebSocket Control Server

Real-time WebSocket communication for controlling Nova robots from external clients.
Maintains persistent connections and shared state across the Nova application.
Provides comprehensive robot lifecycle management and event broadcasting.
"""

import asyncio
import json
import logging
import threading
import time
from typing import Any, Optional

import websockets
import websockets.exceptions

from nova.core.playback_control import (
    MotionGroupId,
    PlaybackDirection,
    PlaybackEvent,
    PlaybackSpeedPercent,
    PlaybackState,
    get_all_active_robots,
    get_playback_manager,
    get_robot_status_summary,
)

logger = logging.getLogger(__name__)

# Global WebSocket server instance
_websocket_server: Optional["NovaWebSocketServer"] = None
_server_lock = threading.Lock()


class NovaWebSocketServer:
    """WebSocket server for Nova robot control with comprehensive event broadcasting"""

    def __init__(self, host: str = "localhost", port: int = 8765):
        self.host = host
        self.port = port
        self.clients: set[Any] = set()
        self.subscribed_clients: set[Any] = set()
        self.server: Optional[Any] = None
        self.running: bool = False
        self.loop: Optional[asyncio.AbstractEventLoop] = None
        self.thread: Optional[threading.Thread] = None
        self.paused_speeds: dict[str, int] = {}

        logging.getLogger("websockets").setLevel(logging.WARNING)
        self._setup_event_monitoring()

    def _setup_event_monitoring(self):
        """Setup comprehensive event monitoring"""
        try:
            manager = get_playback_manager()
            # Register for new event system
            manager.register_event_callback(self._on_playback_event)
            # Keep legacy callback for backward compatibility
            manager.register_state_change_callback(self._on_state_change)
            logger.info("Event monitoring registered")
        except Exception as e:
            logger.error(f"Failed to register event monitoring: {e}")

    def _on_playback_event(self, event: PlaybackEvent):
        """Handle playback events from the new event system"""
        if self.loop and self.running:
            asyncio.run_coroutine_threadsafe(self._broadcast_playback_event(event), self.loop)

    def _on_state_change(self, motion_group_id, state, speed, direction):
        """Handle state changes from playback manager (legacy callback)"""
        if self.loop and self.running:
            asyncio.run_coroutine_threadsafe(
                self._broadcast_state_update(
                    str(motion_group_id), state.value if hasattr(state, "value") else str(state)
                ),
                self.loop,
            )

    async def handle_client(self, websocket):
        """Handle WebSocket client connection"""
        self.clients.add(websocket)
        logger.info("Client connected")

        try:
            async for message in websocket:
                response = await self._process_message(json.loads(message), websocket)
                await websocket.send(json.dumps(response))
        except (websockets.exceptions.ConnectionClosed, json.JSONDecodeError):
            pass
        except Exception as e:
            logger.error(f"Client error: {e}")
        finally:
            self.clients.discard(websocket)
            self.subscribed_clients.discard(websocket)
            logger.info("Client disconnected")

    async def _process_message(self, data: dict, websocket) -> dict:
        """Process WebSocket message"""
        cmd_type = data.get("type")
        robot_id = data.get("robot_id")
        manager = get_playback_manager()

        try:
            if cmd_type == "subscribe_events":
                self.subscribed_clients.add(websocket)
                return {
                    "success": True,
                    "robots": self._get_robot_list(),
                    "status": get_robot_status_summary(),
                }

            elif cmd_type == "get_robots":
                return {
                    "success": True,
                    "robots": self._get_robot_list(),
                    "status": get_robot_status_summary(),
                }

            elif cmd_type == "get_status":
                return {"success": True, "status": get_robot_status_summary()}

            elif cmd_type == "set_speed" and robot_id:
                speed = max(0, min(100, int(data.get("speed", 100))))
                mgid = MotionGroupId(robot_id)

                # Store speed for paused robots, apply immediately for executing ones
                if self._is_paused(mgid):
                    # For paused robots, only store the speed - don't apply it yet
                    self.paused_speeds[robot_id] = speed
                else:
                    manager.set_external_override(mgid, PlaybackSpeedPercent(speed))
                    if robot_id in self.paused_speeds:
                        del self.paused_speeds[robot_id]

                await self._broadcast_to_subscribers(
                    {"type": "speed_change", "robot_id": robot_id, "speed": speed}
                )
                return {"success": True, "robot_id": robot_id, "speed": speed}

            elif cmd_type == "pause" and robot_id:
                mgid = MotionGroupId(robot_id)
                self.paused_speeds[robot_id] = int(manager.get_effective_speed(mgid))
                manager.pause(mgid)
                return {"success": True, "robot_id": robot_id}

            elif cmd_type == "resume" and robot_id:
                mgid = MotionGroupId(robot_id)
                if robot_id in self.paused_speeds:
                    speed = self.paused_speeds.pop(robot_id)
                    manager.set_external_override(mgid, PlaybackSpeedPercent(speed))
                manager.resume(mgid)
                return {"success": True, "robot_id": robot_id}

            elif cmd_type in ["step_forward", "step_backward"] and robot_id:
                mgid = MotionGroupId(robot_id)

                # Get current speed - use paused speed if available, otherwise current effective speed
                if robot_id in self.paused_speeds:
                    current_speed = PlaybackSpeedPercent(self.paused_speeds.pop(robot_id))
                else:
                    current_speed = manager.get_effective_speed(mgid)

                # Set both direction and state in a single external override
                if cmd_type == "step_forward":
                    direction = PlaybackDirection.FORWARD
                else:
                    direction = PlaybackDirection.BACKWARD

                manager.set_external_override(
                    mgid, current_speed, state=PlaybackState.PLAYING, direction=direction
                )
                return {"success": True, "robot_id": robot_id}

            else:
                return {
                    "success": False,
                    "error": f"Unknown command or missing robot_id: {cmd_type}",
                }

        except Exception as e:
            return {"success": False, "error": str(e)}

    def _get_robot_list(self) -> list[dict]:
        """Get current robot list with states and metadata"""
        robots = []
        manager = get_playback_manager()

        for mgid in get_all_active_robots():
            state = manager.get_execution_state(mgid)
            metadata = manager.get_robot_metadata(mgid)
            robots.append(
                {
                    "id": str(mgid),
                    "name": metadata.get("name", str(mgid)) if metadata else str(mgid),
                    "speed": int(manager.get_effective_speed(mgid)),
                    "state": state.value if state else "idle",
                    "can_pause": manager.can_pause(mgid),
                    "can_resume": manager.can_resume(mgid),
                    "is_executing": state.value == "executing" if state else False,
                    "registered_at": metadata["registered_at"].isoformat()
                    if metadata
                    and "registered_at" in metadata
                    and metadata["registered_at"]
                    and hasattr(metadata["registered_at"], "isoformat")
                    else None,
                }
            )
        return robots

    def _is_paused(self, mgid: MotionGroupId) -> bool:
        """Check if robot is paused"""
        state = get_playback_manager().get_effective_state(mgid)
        return state == PlaybackState.PAUSED

    async def _broadcast_playback_event(self, event: PlaybackEvent):
        """Broadcast playback events to subscribed clients"""
        # Convert event to WebSocket message format
        message: dict[str, Any] = {
            "type": "playback_event",
            "event_type": event.event_type,
            "robot_id": str(event.motion_group_id),
            "timestamp": event.timestamp.isoformat(),
        }

        # Add event-specific data based on event type
        if event.event_type == "speed_change":
            message["old_speed"] = int(getattr(event, "old_speed", 0))
            message["new_speed"] = int(getattr(event, "new_speed", 0))
            message["source"] = getattr(event, "source", "unknown")
        elif event.event_type == "state_change":
            old_state = getattr(event, "old_state", None)
            new_state = getattr(event, "new_state", None)
            direction = getattr(event, "direction", None)
            message["old_state"] = (
                old_state.value
                if old_state and hasattr(old_state, "value")
                else str(old_state)
                if old_state
                else ""
            )
            message["new_state"] = (
                new_state.value
                if new_state and hasattr(new_state, "value")
                else str(new_state)
                if new_state
                else ""
            )
            message["speed"] = int(getattr(event, "speed", 0))
            message["direction"] = (
                direction.value
                if direction and hasattr(direction, "value")
                else str(direction)
                if direction
                else ""
            )
        elif event.event_type == "execution_started":
            message["speed"] = int(getattr(event, "speed", 0))
        elif event.event_type == "robot_registered":
            message["robot_name"] = getattr(event, "robot_name", None)
            message["initial_speed"] = int(getattr(event, "initial_speed", 100))
        elif event.event_type in ["program_started", "program_stopped"]:
            message["program_name"] = getattr(event, "program_name", None)
            if hasattr(event, "total_robots"):
                message["total_robots"] = getattr(event, "total_robots", 0)

        await self._broadcast_to_subscribers(message)

    async def _broadcast_state_update(self, robot_id: str, state: str):
        """Broadcast state update to subscribed clients (legacy)"""
        await self._broadcast_to_subscribers(
            {"type": "state_change", "robot_id": robot_id, "state": state, "timestamp": time.time()}
        )

    async def _broadcast_to_subscribers(self, message: dict):
        """Broadcast message to subscribed clients"""
        if not self.subscribed_clients:
            return

        data = json.dumps(message)
        disconnected = set()

        for client in self.subscribed_clients.copy():
            try:
                await client.send(data)
            except Exception:
                disconnected.add(client)

        # Clean up disconnected clients
        for client in disconnected:
            self.clients.discard(client)
            self.subscribed_clients.discard(client)

    async def start_server(self):
        """Start the WebSocket server"""
        try:
            self.server = await websockets.serve(
                self.handle_client, self.host, self.port, ping_interval=20, ping_timeout=10
            )
            logger.info(f"WebSocket server started on {self.host}:{self.port}")
            self.running = True
            await self.server.wait_closed()
        except Exception as e:
            logger.error(f"Error starting WebSocket server: {e}")
            self.running = False

    def start_in_thread(self):
        """Start WebSocket server in background thread"""
        if self.thread and self.thread.is_alive():
            return

        def run_server():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            try:
                self.loop.run_until_complete(self.start_server())
            except Exception as e:
                logger.error(f"WebSocket server error: {e}")
            finally:
                self.loop.close()

        self.thread = threading.Thread(target=run_server, daemon=True)
        self.thread.start()
        time.sleep(0.1)  # Give server time to start

    def stop_in_thread(self):
        """Stop WebSocket server"""
        self.running = False
        if self.loop:
            self.loop.call_soon_threadsafe(self._stop_server)

    def _stop_server(self):
        """Internal stop method"""
        if self.server:
            self.server.close()


def get_websocket_server() -> Optional[NovaWebSocketServer]:
    """Get the global WebSocket server instance"""
    return _websocket_server


def start_websocket_server(host: str = "localhost", port: int = 8765) -> NovaWebSocketServer:
    """Start the WebSocket server if not already running"""
    global _websocket_server

    with _server_lock:
        if _websocket_server is None:
            _websocket_server = NovaWebSocketServer(host, port)
        if not _websocket_server.running:
            _websocket_server.start_in_thread()
        return _websocket_server


def stop_websocket_server():
    """Stop the WebSocket server if running"""
    global _websocket_server

    with _server_lock:
        if _websocket_server and _websocket_server.running:
            _websocket_server.stop_in_thread()
            _websocket_server = None
