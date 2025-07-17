"""Unit Tests for Nova WebSocket External Control with State Updates

This module tests the WebSocket external control functionality where commands
trigger state updates to all connected clients. No confirmations, no request IDs,
just simple commands and state broadcasts.
"""

import json
from unittest.mock import AsyncMock

import pytest

from nova.external_control.websocket_control import (
    NovaWebSocketServer,
    get_websocket_server,
    start_websocket_server,
    stop_websocket_server,
)
from nova.playback import PlaybackState, get_playback_manager


class TestWebSocketStateUpdates:
    """Test WebSocket commands trigger state updates to all clients"""

    @pytest.fixture
    def server(self):
        """Create a fresh WebSocket server for each test"""
        return NovaWebSocketServer(host="localhost", port=8766)

    @pytest.fixture
    def robot_id(self):
        """Standard robot ID for testing"""
        return "test_robot"

    @pytest.fixture
    def mock_clients(self):
        """Three mock WebSocket clients"""
        return [AsyncMock() for _ in range(3)]

    @pytest.fixture
    def manager(self):
        """Get the playback manager"""
        return get_playback_manager()

    def test_server_initialization(self, server):
        """Test that server initializes with correct defaults"""
        assert server.host == "localhost"
        assert server.port == 8766
        assert server.clients == set()
        assert server.subscribed_clients == set()
        assert server.running is False

    @pytest.mark.asyncio
    async def test_set_speed_broadcasts_state_update(self, server, mock_clients, manager, robot_id):
        """Test set_speed broadcasts state update to all clients"""
        # Subscribe all clients
        for client in mock_clients:
            server.subscribed_clients.add(client)

        # Register robot
        manager.register_robot(robot_id, "Test Robot")

        # Send command
        message = {"type": "set_speed", "robot_id": robot_id, "speed": 75}
        response = await server._process_message(message, mock_clients[0])

        # Simple success response
        assert response["success"] is True

        # All clients get state update
        for client in mock_clients:
            client.send.assert_called_once()
            sent_data = json.loads(client.send.call_args[0][0])
            assert sent_data["type"] == "robot_state_update"
            assert sent_data["robot_id"] == robot_id
            assert sent_data["state"]["speed"] == 75

        # Verify actual speed was set
        actual_speed = manager.get_effective_speed(robot_id)
        assert actual_speed.value == 75

    @pytest.mark.asyncio
    async def test_pause_broadcasts_state_update(self, server, mock_clients, manager, robot_id):
        """Test pause broadcasts state update to all clients"""
        # Subscribe all clients
        for client in mock_clients:
            server.subscribed_clients.add(client)

        # Register robot
        manager.register_robot(robot_id, "Test Robot")
        manager.set_execution_state(robot_id, PlaybackState.EXECUTING)

        # Send command
        message = {"type": "pause", "robot_id": robot_id}
        response = await server._process_message(message, mock_clients[0])

        # Simple success response
        assert response["success"] is True

        # All clients get state update
        for client in mock_clients:
            client.send.assert_called_once()
            sent_data = json.loads(client.send.call_args[0][0])
            assert sent_data["type"] == "robot_state_update"
            assert sent_data["robot_id"] == robot_id
            assert sent_data["state"]["state"] == "paused"

        # Verify robot was actually paused
        actual_state = manager.get_effective_state(robot_id)
        assert actual_state == PlaybackState.PAUSED

    @pytest.mark.asyncio
    async def test_resume_broadcasts_state_update(self, server, mock_clients, manager, robot_id):
        """Test resume broadcasts state update to all clients"""
        # Subscribe all clients
        for client in mock_clients:
            server.subscribed_clients.add(client)

        # Register robot and pause it
        manager.register_robot(robot_id, "Test Robot")
        manager.set_execution_state(robot_id, PlaybackState.EXECUTING)
        manager.pause(robot_id)

        # Send command
        message = {"type": "resume", "robot_id": robot_id}
        response = await server._process_message(message, mock_clients[0])

        # Simple success response
        assert response["success"] is True

        # All clients get state update
        for client in mock_clients:
            client.send.assert_called_once()
            sent_data = json.loads(client.send.call_args[0][0])
            assert sent_data["type"] == "robot_state_update"
            assert sent_data["robot_id"] == robot_id
            assert sent_data["state"]["state"] == "playing"

        # Verify robot was actually resumed
        actual_state = manager.get_effective_state(robot_id)
        assert actual_state == PlaybackState.PLAYING

    @pytest.mark.asyncio
    async def test_any_client_can_control_all_clients_get_updates(
        self, server, mock_clients, manager, robot_id
    ):
        """Test that any client can send commands, all clients get updates"""
        # Subscribe all clients
        for client in mock_clients:
            server.subscribed_clients.add(client)

        # Register robot
        manager.register_robot(robot_id, "Test Robot")

        # Client 0 sets speed
        message = {"type": "set_speed", "robot_id": robot_id, "speed": 25}
        await server._process_message(message, mock_clients[0])

        # Client 1 pauses
        message = {"type": "pause", "robot_id": robot_id}
        await server._process_message(message, mock_clients[1])

        # Client 2 resumes
        message = {"type": "resume", "robot_id": robot_id}
        await server._process_message(message, mock_clients[2])

        # All clients should have received 3 state updates each
        for client in mock_clients:
            assert client.send.call_count == 3

    @pytest.mark.asyncio
    async def test_error_handling_no_state_update(self, server, mock_clients, manager):
        """Test that errors don't trigger state updates"""
        # Subscribe client
        server.subscribed_clients.add(mock_clients[0])

        # Invalid command
        message = {"type": "invalid_command"}
        response = await server._process_message(message, mock_clients[0])

        # Should return error response
        assert response["success"] is False
        assert "error" in response

        # No state update should be sent for invalid commands
        mock_clients[0].send.assert_not_called()

    @pytest.mark.asyncio
    async def test_get_robots_command_still_works(self, server, mock_clients, manager, robot_id):
        """Test getting robot list still works (no state broadcast)"""
        # Register robot
        manager.register_robot(robot_id, "Test Robot")

        # Process get_robots message
        message = {"type": "get_robots"}
        response = await server._process_message(message, mock_clients[0])

        # Should return robot list
        assert response["type"] == "robot_list"
        assert "robots" in response
        assert len(response["robots"]) >= 1

        # Find our robot in the list
        robot_found = False
        for robot in response["robots"]:
            if robot["id"] == robot_id:
                robot_found = True
                assert robot["name"] == "Test Robot"
                assert "state" in robot
                assert "speed" in robot
                assert "direction" in robot
                break

        assert robot_found, f"Robot {robot_id} not found in robot list"

        # get_robots should not trigger state broadcasts
        mock_clients[0].send.assert_not_called()


class TestWebSocketGlobalServerManagement:
    """Test suite for global server management functions"""

    def test_start_websocket_server(self):
        """Test starting the global WebSocket server"""
        # Start server
        server = start_websocket_server(host="localhost", port=8773)

        # Verify server is created and running
        assert server is not None
        assert server.host == "localhost"
        assert server.port == 8773

        # Clean up
        stop_websocket_server()

    def test_get_websocket_server(self):
        """Test getting the global WebSocket server instance"""
        # Initially no server
        assert get_websocket_server() is None

        # Start server
        server = start_websocket_server(host="localhost", port=8774)

        # Should return same instance
        assert get_websocket_server() is server

        # Clean up
        stop_websocket_server()

    def test_stop_websocket_server(self):
        """Test stopping the global WebSocket server"""
        # Start server
        start_websocket_server(host="localhost", port=8775)
        assert get_websocket_server() is not None

        # Stop server
        stop_websocket_server()

        # Should be None after stopping
        assert get_websocket_server() is None
