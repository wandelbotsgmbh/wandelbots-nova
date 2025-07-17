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

from nova.playback import (
    PlaybackDirection,
    PlaybackEvent,
    PlaybackSpeedPercent,
    PlaybackState,
    StateChangeEvent,
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

        logging.getLogger("websockets").setLevel(logging.WARNING)
        self._setup_event_monitoring()

    def _setup_event_monitoring(self):
        """Setup comprehensive event monitoring"""
        try:
            manager = get_playback_manager()
            # Register for event system
            manager.register_event_callback(self._on_playback_event)
            logger.info("Event monitoring registered")
        except Exception as e:
            logger.error(f"Failed to register event monitoring: {e}")

    def _on_playback_event(self, event: PlaybackEvent):
        """Handle playback events from the event system"""
        if self.loop and self.running:
            # Send comprehensive state update for all relevant events
            robot_id = str(event.motion_group_id)

            # Handle state change events specifically with legacy support
            if isinstance(event, StateChangeEvent):
                asyncio.run_coroutine_threadsafe(self._broadcast_state_update(robot_id), self.loop)

            # For all events, send comprehensive state update
            asyncio.run_coroutine_threadsafe(
                self._broadcast_comprehensive_state_update(robot_id), self.loop
            )

            # Handle all events with general broadcast for compatibility
            asyncio.run_coroutine_threadsafe(self._broadcast_playback_event(event), self.loop)

    async def _broadcast_comprehensive_state_update(self, robot_id: str):
        """Broadcast comprehensive state update for a robot"""
        try:
            robot_state = self._get_robot_state(robot_id)
            await self._broadcast_to_subscribers(
                {"type": "robot_state_update", "robot_id": robot_id, "state": robot_state}
            )
        except Exception as e:
            logger.error(f"Failed to broadcast comprehensive state update for {robot_id}: {e}")

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
        """Process WebSocket message with simple state updates"""
        cmd_type = data.get("type")
        motion_group_id = data.get("robot_id")
        manager = get_playback_manager()

        try:
            if cmd_type == "subscribe_events":
                self.subscribed_clients.add(websocket)
                return {
                    "success": True,
                    "message": "Subscribed to events",
                    "robots": self._get_robot_list(),
                    "status": get_robot_status_summary(),
                }

            elif cmd_type == "get_robots":
                return {
                    "type": "robot_list",
                    "success": True,
                    "robots": self._get_robot_list(),
                    "status": get_robot_status_summary(),
                }

            elif cmd_type == "get_status":
                return {"success": True, "status": get_robot_status_summary()}

            elif cmd_type == "set_speed" and motion_group_id:
                speed = int(data.get("speed", 100))

                # Validate speed range
                if speed < 0 or speed > 100:
                    return {
                        "success": False,
                        "error": f"Invalid speed value: {speed}. Must be between 0 and 100",
                    }

                # Check if robot exists
                if not self._robot_exists(motion_group_id):
                    return {"success": False, "error": f"Robot not found: {motion_group_id}"}

                # Get current direction and state to preserve them
                current_direction = manager.get_effective_direction(motion_group_id)
                current_state = manager.get_effective_state(motion_group_id)

                # Apply speed change while preserving direction and state
                manager.set_external_override(
                    motion_group_id,
                    PlaybackSpeedPercent(value=speed),
                    direction=current_direction,
                    state=current_state,
                )

                # Broadcast state update to all subscribers
                await self._broadcast_state_update(motion_group_id)

                # Return simple success response
                return {"success": True}

            elif cmd_type == "pause" and motion_group_id:
                # Check if robot exists
                if not self._robot_exists(motion_group_id):
                    return {"success": False, "error": f"Robot not found: {motion_group_id}"}

                # Pause robot
                manager.pause(motion_group_id)

                # Broadcast state update to all subscribers
                await self._broadcast_state_update(motion_group_id)

                # Return simple success response
                return {"success": True}

            elif cmd_type == "resume" and motion_group_id:
                # Check if robot exists
                if not self._robot_exists(motion_group_id):
                    return {"success": False, "error": f"Robot not found: {motion_group_id}"}

                # Resume robot
                manager.resume(motion_group_id)

                # Broadcast state update to all subscribers
                await self._broadcast_state_update(motion_group_id)

                # Return simple success response
                return {"success": True}

            elif cmd_type in ["play_forward", "play_backward"] and motion_group_id:
                # Check if robot exists
                if not self._robot_exists(motion_group_id):
                    return {"success": False, "error": f"Robot not found: {motion_group_id}"}

                # Get current speed
                current_speed = manager.get_effective_speed(motion_group_id)

                # Set direction and state
                if cmd_type == "play_forward":
                    direction = PlaybackDirection.FORWARD
                else:
                    direction = PlaybackDirection.BACKWARD

                # Set to playing state with direction
                manager.set_external_override(
                    motion_group_id, current_speed, state=PlaybackState.PLAYING, direction=direction
                )

                # Broadcast state update to all subscribers
                await self._broadcast_state_update(motion_group_id)

                # Return simple success response
                return {"success": True}

            else:
                return {
                    "success": False,
                    "error": f"Unknown command or missing robot_id: {cmd_type}",
                }

        except Exception as e:
            return {"success": False, "error": str(e)}

    async def _broadcast_state_update(self, robot_id: str):
        """Broadcast current robot state to all subscribed clients"""
        try:
            robot_state = self._get_robot_state(robot_id)
            await self._broadcast_to_subscribers(
                {"type": "robot_state_update", "robot_id": robot_id, "state": robot_state}
            )
        except Exception as e:
            logger.error(f"Failed to broadcast state update for {robot_id}: {e}")

    def _get_robot_list(self) -> list[dict]:
        """Get current robot list with comprehensive states"""
        robots = []

        for motion_group_id in get_all_active_robots():
            robot_state = self._get_robot_state(motion_group_id)
            robots.append(robot_state)

        return robots

    def _get_robot_state(self, motion_group_id: str) -> dict:
        """Get complete robot state information"""
        manager = get_playback_manager()

        try:
            metadata = manager.get_robot_metadata(motion_group_id)
            effective_speed = manager.get_effective_speed(motion_group_id)
            effective_state = manager.get_effective_state(motion_group_id)
            direction = manager.get_effective_direction(motion_group_id)

            return {
                "id": str(motion_group_id),
                "name": metadata.get("name", str(motion_group_id))
                if metadata
                else str(motion_group_id),
                "speed": effective_speed.value,
                "state": effective_state.value if effective_state else "idle",
                "direction": direction.value if direction else "forward",
                "can_pause": manager.can_pause(motion_group_id),
                "can_resume": manager.can_resume(motion_group_id),
                "is_executing": (effective_state.value == "executing")
                if effective_state
                else False,
                "registered_at": metadata["registered_at"].isoformat()
                if metadata
                and "registered_at" in metadata
                and metadata["registered_at"]
                and hasattr(metadata["registered_at"], "isoformat")
                else None,
                "last_updated": time.time(),
            }
        except Exception as e:
            # Fallback state if robot info can't be retrieved
            return {
                "id": str(motion_group_id),
                "name": str(motion_group_id),
                "speed": 100,
                "state": "unknown",
                "direction": "forward",
                "can_pause": False,
                "can_resume": False,
                "is_executing": False,
                "registered_at": None,
                "last_updated": time.time(),
                "error": str(e),
            }

    def _robot_exists(self, motion_group_id: str) -> bool:
        """Check if robot exists in the system"""
        try:
            manager = get_playback_manager()
            # Try to get robot metadata - if it fails, robot doesn't exist
            metadata = manager.get_robot_metadata(motion_group_id)
            return metadata is not None
        except Exception:
            return False

    def _is_paused(self, motion_group_id: str) -> bool:
        """Check if robot is paused"""
        state = get_playback_manager().get_effective_state(motion_group_id)
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

    async def _broadcast_to_subscribers(self, message: dict):
        """Broadcast message to subscribed clients"""
        if not self.subscribed_clients:
            return

        data = json.dumps(message)
        disconnected = set()

        for client in self.subscribed_clients.copy():
            try:
                # For real WebSocket clients, send is async
                # For mock clients in tests, we just call it
                if hasattr(client, "send"):
                    import inspect

                    send_method = client.send
                    if inspect.iscoroutinefunction(send_method):
                        await send_method(data)
                    else:
                        # Handle sync send method (like in mocks)
                        send_method(data)
                else:
                    disconnected.add(client)
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
