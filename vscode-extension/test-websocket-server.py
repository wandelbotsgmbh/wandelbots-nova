#!/usr/bin/env python3
"""
Simple test script to verify VS Code extension compatibility with the new WebSocket API.
This script simulates the WebSocket server behavior based on the test files.
"""

import asyncio
import json
import logging
import websockets
from dataclasses import dataclass
from typing import Optional
from enum import Enum

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class PlaybackState(Enum):
    IDLE = "idle"
    EXECUTING = "executing"
    PAUSED = "paused"
    PLAYING = "playing"

class PlaybackDirection(Enum):
    FORWARD = "forward"
    BACKWARD = "backward"

@dataclass
class Robot:
    id: str
    name: str
    speed: int = 100
    state: PlaybackState = PlaybackState.IDLE
    direction: PlaybackDirection = PlaybackDirection.FORWARD
    can_pause: bool = True
    can_resume: bool = False

class MockWebSocketServer:
    def __init__(self):
        self.robots: dict[str, Robot] = {}
        self.clients: set = set()
        self.subscribed_clients: set = set()
        
        # Add some test robots
        self.robots["robot1"] = Robot("robot1", "Test Robot 1", 100, PlaybackState.EXECUTING)
        self.robots["robot2"] = Robot("robot2", "Test Robot 2", 50, PlaybackState.PAUSED)
        self.robots["robot2"].can_pause = False
        self.robots["robot2"].can_resume = True

    async def handle_client(self, websocket):
        """Handle a new WebSocket client connection"""
        logger.info(f"New client connected: {websocket.remote_address}")
        self.clients.add(websocket)
        
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    response = await self.process_message(data, websocket)
                    if response:
                        await websocket.send(json.dumps(response))
                except json.JSONDecodeError:
                    logger.error(f"Invalid JSON received: {message}")
                except Exception as e:
                    logger.error(f"Error processing message: {e}")
                    
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"Client disconnected: {websocket.remote_address}")
        finally:
            self.clients.discard(websocket)
            self.subscribed_clients.discard(websocket)

    async def process_message(self, message: dict, websocket) -> Optional[dict]:
        """Process a message from a client"""
        message_type = message.get("type")
        command_id = message.get("command_id")
        
        logger.info(f"Processing message: {message_type}")
        
        if message_type == "subscribe_events":
            self.subscribed_clients.add(websocket)
            return {
                "success": True,
                "message": "Subscribed to events",
                "command_id": command_id
            }
            
        elif message_type == "get_robots":
            robots_list = []
            for robot in self.robots.values():
                robots_list.append({
                    "id": robot.id,
                    "name": robot.name,
                    "speed": robot.speed,
                    "state": robot.state.value,
                    "direction": robot.direction.value,
                    "can_pause": robot.can_pause,
                    "can_resume": robot.can_resume
                })
            
            return {
                "type": "robot_list",
                "robots": robots_list,
                "command_id": command_id
            }
            
        elif message_type == "set_speed":
            robot_id = message.get("robot_id")
            speed = message.get("speed")
            
            if not robot_id:
                return {
                    "success": False,
                    "error": "robot_id is required",
                    "command_id": command_id
                }
                
            if robot_id not in self.robots:
                return {
                    "success": False,
                    "error": f"Robot {robot_id} not found",
                    "command_id": command_id,
                    "robot_id": robot_id
                }
                
            if speed is None or speed < 0 or speed > 100:
                return {
                    "success": False,
                    "error": "Speed must be between 0 and 100",
                    "command_id": command_id,
                    "robot_id": robot_id
                }
            
            # Update robot speed
            old_speed = self.robots[robot_id].speed
            self.robots[robot_id].speed = speed
            
            # Create command response
            response = {
                "success": True,
                "robot_id": robot_id,
                "command_id": command_id,
                "speed": speed
            }
            
            # Broadcast speed change event (but not to the originating client)
            await self.broadcast_event({
                "type": "playback_event",
                "event_type": "speed_changed",
                "robot_id": robot_id,
                "old_speed": old_speed,
                "new_speed": speed
            }, exclude_client=websocket)
            
            return response
            
        elif message_type == "pause":
            robot_id = message.get("robot_id")
            
            if not robot_id:
                return {
                    "success": False,
                    "error": "robot_id is required",
                    "command_id": command_id
                }
                
            if robot_id not in self.robots:
                return {
                    "success": False,
                    "error": f"Robot {robot_id} not found",
                    "command_id": command_id,
                    "robot_id": robot_id
                }
            
            robot = self.robots[robot_id]
            robot.state = PlaybackState.PAUSED
            robot.can_pause = False
            robot.can_resume = True
            
            # Create command response
            response = {
                "success": True,
                "robot_id": robot_id,
                "command_id": command_id,
                "state": {
                    "id": robot.id,
                    "name": robot.name,
                    "speed": robot.speed,
                    "state": robot.state.value,
                    "direction": robot.direction.value,
                    "can_pause": robot.can_pause,
                    "can_resume": robot.can_resume
                }
            }
            
            # Broadcast state change event
            await self.broadcast_event({
                "type": "playback_event",
                "event_type": "state_changed",
                "robot_id": robot_id,
                "old_state": "executing",
                "new_state": "paused"
            }, exclude_client=websocket)
            
            return response
            
        elif message_type == "resume":
            robot_id = message.get("robot_id")
            
            if not robot_id:
                return {
                    "success": False,
                    "error": "robot_id is required",
                    "command_id": command_id
                }
                
            if robot_id not in self.robots:
                return {
                    "success": False,
                    "error": f"Robot {robot_id} not found",
                    "command_id": command_id,
                    "robot_id": robot_id
                }
            
            robot = self.robots[robot_id]
            robot.state = PlaybackState.PLAYING
            robot.can_pause = True
            robot.can_resume = False
            
            # Create command response
            response = {
                "success": True,
                "robot_id": robot_id,
                "command_id": command_id,
                "state": {
                    "id": robot.id,
                    "name": robot.name,
                    "speed": robot.speed,
                    "state": robot.state.value,
                    "direction": robot.direction.value,
                    "can_pause": robot.can_pause,
                    "can_resume": robot.can_resume
                }
            }
            
            # Broadcast state change event
            await self.broadcast_event({
                "type": "playback_event",
                "event_type": "state_changed",
                "robot_id": robot_id,
                "old_state": "paused",
                "new_state": "playing"
            }, exclude_client=websocket)
            
            return response
            
        elif message_type == "step_forward":
            robot_id = message.get("robot_id")
            
            if not robot_id:
                return {
                    "success": False,
                    "error": "robot_id is required",
                    "command_id": command_id
                }
                
            if robot_id not in self.robots:
                return {
                    "success": False,
                    "error": f"Robot {robot_id} not found",
                    "command_id": command_id,
                    "robot_id": robot_id
                }
            
            robot = self.robots[robot_id]
            robot.direction = PlaybackDirection.FORWARD
            robot.state = PlaybackState.PLAYING
            robot.can_pause = True
            robot.can_resume = False
            
            return {
                "success": True,
                "robot_id": robot_id,
                "command_id": command_id,
                "state": {
                    "id": robot.id,
                    "name": robot.name,
                    "speed": robot.speed,
                    "state": robot.state.value,
                    "direction": robot.direction.value,
                    "can_pause": robot.can_pause,
                    "can_resume": robot.can_resume
                }
            }
            
        elif message_type == "step_backward":
            robot_id = message.get("robot_id")
            
            if not robot_id:
                return {
                    "success": False,
                    "error": "robot_id is required",
                    "command_id": command_id
                }
                
            if robot_id not in self.robots:
                return {
                    "success": False,
                    "error": f"Robot {robot_id} not found",
                    "command_id": command_id,
                    "robot_id": robot_id
                }
            
            robot = self.robots[robot_id]
            robot.direction = PlaybackDirection.BACKWARD
            robot.state = PlaybackState.PLAYING
            robot.can_pause = True
            robot.can_resume = False
            
            return {
                "success": True,
                "robot_id": robot_id,
                "command_id": command_id,
                "state": {
                    "id": robot.id,
                    "name": robot.name,
                    "speed": robot.speed,
                    "state": robot.state.value,
                    "direction": robot.direction.value,
                    "can_pause": robot.can_pause,
                    "can_resume": robot.can_resume
                }
            }
            
        else:
            return {
                "success": False,
                "error": f"Unknown command: {message_type}",
                "command_id": command_id
            }

    async def broadcast_event(self, event: dict, exclude_client=None):
        """Broadcast an event to all subscribed clients"""
        if not self.subscribed_clients:
            return
            
        logger.info(f"Broadcasting event: {event.get('event_type')}")
        
        # Send to all subscribed clients (except excluded one)
        disconnected_clients = set()
        for client in self.subscribed_clients:
            if client == exclude_client:
                continue
                
            try:
                await client.send(json.dumps(event))
            except websockets.exceptions.ConnectionClosed:
                disconnected_clients.add(client)
        
        # Remove disconnected clients
        for client in disconnected_clients:
            self.subscribed_clients.discard(client)
            self.clients.discard(client)

async def main():
    """Start the mock WebSocket server"""
    server = MockWebSocketServer()
    
    logger.info("Starting mock WebSocket server on localhost:8765")
    
    async with websockets.serve(server.handle_client, "localhost", 8765):
        logger.info("Mock WebSocket server is running. Press Ctrl+C to stop.")
        await asyncio.Future()  # Run forever

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Server stopped by user")
