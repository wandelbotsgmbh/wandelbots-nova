#!/usr/bin/env python3
"""
Simple WebSocket client test to verify the extension's WebSocket protocol implementation
"""

import asyncio
import json
import websockets
from websockets.exceptions import ConnectionClosed

async def test_websocket_protocol():
    """Test the WebSocket protocol with our mock server"""
    
    uri = "ws://localhost:8765"
    
    try:
        async with websockets.connect(uri) as websocket:
            print("✅ Connected to WebSocket server")
            
            # Test 1: Subscribe to events
            print("\n🔄 Test 1: Subscribe to events")
            subscribe_msg = {
                "type": "subscribe_events",
                "command_id": "test_sub_1"
            }
            await websocket.send(json.dumps(subscribe_msg))
            response = await websocket.recv()
            result = json.loads(response)
            print(f"Response: {result}")
            assert result["success"] is True
            assert result["command_id"] == "test_sub_1"
            print("✅ Subscribe to events successful")
            
            # Test 2: Get robots
            print("\n🔄 Test 2: Get robots")
            get_robots_msg = {
                "type": "get_robots",
                "command_id": "test_robots_1"
            }
            await websocket.send(json.dumps(get_robots_msg))
            response = await websocket.recv()
            result = json.loads(response)
            print(f"Response: {result}")
            assert result["type"] == "robot_list"
            assert "robots" in result
            assert len(result["robots"]) >= 2  # Our mock server has 2 robots
            print("✅ Get robots successful")
            
            # Test 3: Set speed
            print("\n🔄 Test 3: Set speed")
            set_speed_msg = {
                "type": "set_speed",
                "robot_id": "robot1",
                "speed": 75,
                "command_id": "test_speed_1"
            }
            await websocket.send(json.dumps(set_speed_msg))
            
            # We might receive an event broadcast first, then the command response
            response1 = await websocket.recv()
            result1 = json.loads(response1)
            
            # Check if this is an event or command response
            if "success" in result1:
                result = result1
            else:
                # This was an event, wait for the command response
                response2 = await websocket.recv()
                result = json.loads(response2)
                
            print(f"Response: {result}")
            assert result["success"] is True
            assert result["robot_id"] == "robot1"
            assert result["speed"] == 75
            assert result["command_id"] == "test_speed_1"
            print("✅ Set speed successful")
            
            # Test 4: Pause robot
            print("\n🔄 Test 4: Pause robot")
            pause_msg = {
                "type": "pause",
                "robot_id": "robot1",
                "command_id": "test_pause_1"
            }
            await websocket.send(json.dumps(pause_msg))
            response = await websocket.recv()
            result = json.loads(response)
            print(f"Response: {result}")
            assert result["success"] is True
            assert result["robot_id"] == "robot1"
            assert result["command_id"] == "test_pause_1"
            assert "state" in result
            assert result["state"]["state"] == "paused"
            print("✅ Pause robot successful")
            
            # Test 5: Resume robot
            print("\n🔄 Test 5: Resume robot")
            resume_msg = {
                "type": "resume",
                "robot_id": "robot1",
                "command_id": "test_resume_1"
            }
            await websocket.send(json.dumps(resume_msg))
            response = await websocket.recv()
            result = json.loads(response)
            print(f"Response: {result}")
            assert result["success"] is True
            assert result["robot_id"] == "robot1"
            assert result["command_id"] == "test_resume_1"
            assert "state" in result
            assert result["state"]["state"] == "playing"
            print("✅ Resume robot successful")
            
            # Test 6: Step forward
            print("\n🔄 Test 6: Step forward")
            step_msg = {
                "type": "step_forward",
                "robot_id": "robot1",
                "command_id": "test_step_1"
            }
            await websocket.send(json.dumps(step_msg))
            response = await websocket.recv()
            result = json.loads(response)
            print(f"Response: {result}")
            assert result["success"] is True
            assert result["robot_id"] == "robot1"
            assert result["command_id"] == "test_step_1"
            assert "state" in result
            assert result["state"]["direction"] == "forward"
            print("✅ Step forward successful")
            
            # Test 7: Step backward
            print("\n🔄 Test 7: Step backward")
            step_msg = {
                "type": "step_backward",
                "robot_id": "robot1",
                "command_id": "test_step_2"
            }
            await websocket.send(json.dumps(step_msg))
            response = await websocket.recv()
            result = json.loads(response)
            print(f"Response: {result}")
            assert result["success"] is True
            assert result["robot_id"] == "robot1"
            assert result["command_id"] == "test_step_2"
            assert "state" in result
            assert result["state"]["direction"] == "backward"
            print("✅ Step backward successful")
            
            # Test 8: Error handling - invalid robot
            print("\n🔄 Test 8: Error handling - invalid robot")
            invalid_msg = {
                "type": "set_speed",
                "robot_id": "invalid_robot",
                "speed": 50,
                "command_id": "test_error_1"
            }
            await websocket.send(json.dumps(invalid_msg))
            response = await websocket.recv()
            result = json.loads(response)
            print(f"Response: {result}")
            assert result["success"] is False
            assert "error" in result
            assert result["command_id"] == "test_error_1"
            print("✅ Error handling successful")
            
            # Test 9: Error handling - invalid speed
            print("\n🔄 Test 9: Error handling - invalid speed")
            invalid_msg = {
                "type": "set_speed",
                "robot_id": "robot1",
                "speed": 150,  # Invalid speed
                "command_id": "test_error_2"
            }
            await websocket.send(json.dumps(invalid_msg))
            response = await websocket.recv()
            result = json.loads(response)
            print(f"Response: {result}")
            assert result["success"] is False
            assert "error" in result
            assert result["command_id"] == "test_error_2"
            print("✅ Error handling successful")
            
            print("\n🎉 All tests passed! WebSocket protocol is working correctly.")
            
    except ConnectionClosed:
        print("❌ Connection closed unexpectedly")
    except Exception as e:
        print(f"❌ Test failed: {e}")
        raise

if __name__ == "__main__":
    print("🧪 Testing WebSocket protocol implementation...")
    print("Make sure the mock server is running on localhost:8765")
    asyncio.run(test_websocket_protocol())
