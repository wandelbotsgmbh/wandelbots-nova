#!/usr/bin/env node

/**
 * Test script for Nova VS Code Extension WebSocket functionality
 *
 * This script tests the WebSocket connection and message handling
 * that the VS Code extension uses to communicate with Nova.
 */

const WebSocket = require("ws");

const WEBSOCKET_URL = "ws://localhost:8765";
const TEST_TIMEOUT = 5000; // 5 seconds

console.log("🧪 Testing Nova WebSocket Extension Integration\n");

async function testWebSocketConnection() {
  return new Promise((resolve, reject) => {
    console.log(`📡 Connecting to ${WEBSOCKET_URL}...`);

    const ws = new WebSocket(WEBSOCKET_URL);
    let connected = false;

    const timeout = setTimeout(() => {
      if (!connected) {
        ws.close();
        reject(
          new Error(
            "Connection timeout - Make sure Nova program with WebSocketControl() is running"
          )
        );
      }
    }, TEST_TIMEOUT);

    ws.on("open", () => {
      connected = true;
      clearTimeout(timeout);
      console.log("✅ WebSocket connection established");

      // Test event subscription (new feature)
      console.log("📨 Subscribing to events...");
      ws.send(JSON.stringify({ type: "subscribe_events" }));
    });

    ws.on("message", (data) => {
      try {
        const message = JSON.parse(data.toString());
        console.log(`📥 Received: ${message.type || "unknown"}`);

        if (message.type === "playback_event") {
          console.log(
            `   Event: ${message.event_type} for robot ${message.robot_id}`
          );
        } else if (message.robots) {
          console.log(`   Found ${message.robots.length} robot(s):`);
          message.robots.forEach((robot) => {
            console.log(
              `     - ${robot.name || robot.id}: ${robot.state} (${
                robot.speed
              }%)`
            );
          });
        }

        // Test robot list request
        if (message.success && message.robots !== undefined) {
          console.log("📨 Testing robot status request...");
          ws.send(JSON.stringify({ type: "get_status" }));
        }

        // After a few messages, close and resolve
        setTimeout(() => {
          ws.close();
          resolve();
        }, 1000);
      } catch (error) {
        console.error("❌ Failed to parse message:", error);
      }
    });

    ws.on("close", () => {
      console.log("🔌 WebSocket connection closed");
      if (connected) {
        resolve();
      }
    });

    ws.on("error", (error) => {
      clearTimeout(timeout);
      console.error("❌ WebSocket error:", error.message);
      reject(error);
    });
  });
}

async function main() {
  try {
    await testWebSocketConnection();
    console.log("\n✅ WebSocket test completed successfully!");
    console.log(
      "🎉 VS Code extension should work correctly with Nova WebSocket interface"
    );
    console.log("\nTo test the full extension:");
    console.log(
      "1. Start a Nova program with external_control=WebSocketControl()"
    );
    console.log("2. Open VS Code with the Nova extension installed");
    console.log("3. Check the status bar for robot count");
    console.log('4. Use Command Palette: "Nova: Show Robot Controls"');
  } catch (error) {
    console.error("\n❌ WebSocket test failed:", error.message);
    console.log("\n🔧 Troubleshooting:");
    console.log(
      "1. Make sure a Nova program is running with WebSocketControl()"
    );
    console.log(
      "2. Verify the program includes: external_control=nova.external_control.WebSocketControl()"
    );
    console.log("3. Check that port 8765 is not blocked by firewall");
    console.log("4. Try running: python examples/execution_speed_control.py");
    process.exit(1);
  }
}

if (require.main === module) {
  main();
}
