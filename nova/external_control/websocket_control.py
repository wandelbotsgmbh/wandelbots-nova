"""Nova WebSocket Control Server

This module provides real-time WebSocket communication for controlling Nova robots
from VS Code extensions. Unlike subprocess control, this maintains persistent
connections and shared state across the Nova application.

Architecture:
1. Nova program starts WebSocket server automatically
2. VS Code extension connects via WebSocket client
3. Real-time bidirectional communication for robot control
4. Server runs in background thread, doesn't block Nova execution

Usage in Nova programs:
    # WebSocket server starts automatically when Nova is imported
    # No additional setup required!

Usage in VS Code extension:
    const ws = new WebSocket('ws://localhost:8765');
    ws.send(JSON.stringify({type: 'get_robots'}));
"""

import asyncio
import json
import logging
import threading
import time
from typing import Any, Optional

import websockets

from nova.core.playback_control import (
    MotionGroupId,
    PlaybackSpeedPercent,
    get_all_active_robots,
    get_playback_manager,
)

logger = logging.getLogger(__name__)

# Global WebSocket server instance
_websocket_server: Optional["NovaWebSocketServer"] = None
_server_lock = threading.Lock()


class NovaWebSocketServer:
    """WebSocket server for Nova robot control

    Provides real-time communication between Nova programs and external tools
    like VS Code extensions. Runs in a background thread to avoid blocking
    Nova execution.
    """

    def __init__(self, host: str = "localhost", port: int = 8765):
        self.host = host
        self.port = port
        self.clients: set[Any] = set()
        self.server = None
        self.server_task = None
        self.loop = None
        self.thread = None
        self.running = False
        self._setup_logging()

    def _setup_logging(self):
        """Setup WebSocket logging"""
        # Reduce websockets library verbosity
        logging.getLogger("websockets").setLevel(logging.WARNING)

    async def register_client(self, websocket):
        """Register a new WebSocket client"""
        self.clients.add(websocket)
        logger.info("Nova WebSocket client connected")

        # Send initial state to new client
        await self.send_robot_update(websocket)

    async def unregister_client(self, websocket):
        """Unregister a WebSocket client"""
        self.clients.discard(websocket)
        logger.info("Nova WebSocket client disconnected")

    async def handle_client(self, websocket):
        """Handle WebSocket client connection"""
        await self.register_client(websocket)
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    response = await self.process_command(data)
                    await websocket.send(json.dumps(response))
                except json.JSONDecodeError:
                    await websocket.send(
                        json.dumps({"success": False, "error": "Invalid JSON message"})
                    )
                except Exception as e:
                    logger.error(f"Error processing WebSocket message: {e}")
                    await websocket.send(json.dumps({"success": False, "error": str(e)}))
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            logger.error(f"WebSocket client error: {e}")
        finally:
            await self.unregister_client(websocket)

    async def process_command(self, command: dict[str, Any]) -> dict[str, Any]:
        """Process command from WebSocket client"""
        manager = get_playback_manager()
        cmd_type = command.get("type")
        robot_id = command.get("robot_id")

        try:
            if cmd_type == "detect_nova":
                robots = get_all_active_robots()
                return {
                    "success": True,
                    "nova_available": True,
                    "robots_count": len(robots),
                    "message": f"Nova detected with {len(robots)} robots",
                }

            elif cmd_type == "get_robots":
                robots = get_all_active_robots()
                robot_list = []

                for robot_mgid in robots:
                    current_speed = manager.get_effective_speed(robot_mgid)
                    current_state = manager.get_execution_state(robot_mgid)

                    robot_list.append(
                        {
                            "id": str(robot_mgid),
                            "speed": int(current_speed),
                            "state": current_state.value if current_state else "unknown",
                            "can_pause": manager.can_pause(robot_mgid),
                            "can_resume": manager.can_resume(robot_mgid),
                        }
                    )

                return {"success": True, "robots": robot_list, "count": len(robot_list)}

            elif cmd_type == "set_speed":
                if not robot_id:
                    return {"success": False, "error": "robot_id required for set_speed"}
                speed = max(0, min(100, int(command.get("speed", 100))))
                manager.set_external_override(MotionGroupId(robot_id), PlaybackSpeedPercent(speed))

                # Broadcast update to all clients
                await self.broadcast_robot_update()

                return {"success": True, "robot_id": robot_id, "speed": speed}

            elif cmd_type == "pause":
                if not robot_id:
                    return {"success": False, "error": "robot_id required for pause"}
                manager.pause(MotionGroupId(robot_id))

                # Broadcast update to all clients
                await self.broadcast_robot_update()

                return {"success": True, "robot_id": robot_id, "state": "paused"}

            elif cmd_type == "resume":
                if not robot_id:
                    return {"success": False, "error": "robot_id required for resume"}
                manager.resume(MotionGroupId(robot_id))

                # Broadcast update to all clients
                await self.broadcast_robot_update()

                return {"success": True, "robot_id": robot_id, "state": "executing"}

            elif cmd_type == "get_status":
                if not robot_id:
                    return {"success": False, "error": "robot_id required for get_status"}
                mgid = MotionGroupId(robot_id)
                speed = manager.get_effective_speed(mgid)
                state = manager.get_execution_state(mgid)

                return {
                    "success": True,
                    "robot_id": robot_id,
                    "speed": int(speed),
                    "state": state.value if state else "unknown",
                    "can_pause": manager.can_pause(mgid),
                    "can_resume": manager.can_resume(mgid),
                }

            else:
                return {"success": False, "error": f"Unknown command type: {cmd_type}"}

        except Exception as e:
            return {"success": False, "error": str(e), "robot_id": robot_id}

    async def send_robot_update(self, websocket):
        """Send robot status update to a specific client"""
        try:
            update = await self.process_command({"type": "get_robots"})
            update["type"] = "robot_update"
            await websocket.send(json.dumps(update))
        except Exception as e:
            logger.error(f"Error sending robot update: {e}")

    async def broadcast_robot_update(self):
        """Broadcast robot status update to all connected clients"""
        if not self.clients:
            return

        try:
            update = await self.process_command({"type": "get_robots"})
            update["type"] = "robot_update"
            message = json.dumps(update)

            # Send to all connected clients
            disconnected = set()
            for client in self.clients:
                try:
                    await client.send(message)
                except websockets.exceptions.ConnectionClosed:
                    disconnected.add(client)
                except Exception as e:
                    logger.error(f"Error broadcasting to client: {e}")
                    disconnected.add(client)

            # Remove disconnected clients
            self.clients -= disconnected

        except Exception as e:
            logger.error(f"Error broadcasting robot update: {e}")

    async def start_server(self):
        """Start the WebSocket server"""
        try:
            self.server = await websockets.serve(
                self.handle_client, self.host, self.port, ping_interval=20, ping_timeout=10
            )
            logger.info(f"Nova WebSocket server started on {self.host}:{self.port}")
            self.running = True

            # Keep server running
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
                logger.error(f"WebSocket server thread error: {e}")
            finally:
                self.loop.close()

        self.thread = threading.Thread(target=run_server, daemon=True)
        self.thread.start()

        # Give server time to start
        time.sleep(0.1)

    async def stop_server(self):
        """Stop the WebSocket server"""
        self.running = False
        if self.server:
            self.server.close()
            await self.server.wait_closed()
        logger.info("Nova WebSocket server stopped")

    def stop_in_thread(self):
        """Stop WebSocket server from main thread"""
        if self.loop and self.running:
            self.loop.call_soon_threadsafe(lambda: asyncio.create_task(self.stop_server()))


def start_websocket_server(host: str = "localhost", port: int = 8765) -> NovaWebSocketServer:
    """Start Nova WebSocket server (called automatically when Nova is imported)

    Args:
        host: Server host (default: localhost)
        port: Server port (default: 8765)

    Returns:
        NovaWebSocketServer: Server instance
    """
    global _websocket_server

    with _server_lock:
        if _websocket_server and _websocket_server.running:
            return _websocket_server

        _websocket_server = NovaWebSocketServer(host, port)
        _websocket_server.start_in_thread()

    logger.info(f"Nova WebSocket control available at ws://{host}:{port}")
    return _websocket_server


def stop_websocket_server():
    """Stop Nova WebSocket server"""
    global _websocket_server

    with _server_lock:
        if _websocket_server:
            _websocket_server.stop_in_thread()
            _websocket_server = None


def get_websocket_server() -> Optional[NovaWebSocketServer]:
    """Get current WebSocket server instance"""
    return _websocket_server
