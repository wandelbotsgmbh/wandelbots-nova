/**
 * Nova Robot Control VS Code Extension - WebSocket Version (Simplified)
 *
 * Simplified extension with state-based robot control:
 * - Commands return simple success/error responses (no confirmations)
 * - Robot state updates are broadcast to all clients via robot_state_update messages
 * - Multiple clients can control robots independently
 * - All clients receive state updates when any client makes changes
 */

const vscode = require("vscode");
const WebSocket = require("ws");

/**
 * Simple robot state object
 */
class RobotState {
  constructor(robotData) {
    this.id = robotData.id;
    this.name = robotData.name || robotData.id;
    this.speed = robotData.speed || 100;
    this.state = robotData.state || "idle";
    this.direction = robotData.direction || "forward";
    this.can_pause =
      robotData.can_pause !== undefined ? robotData.can_pause : true;
    this.can_resume =
      robotData.can_resume !== undefined ? robotData.can_resume : true;
    this.lastUpdated = new Date();
  }

  /**
   * Update robot state with new data
   */
  update(newData) {
    Object.keys(newData).forEach((key) => {
      if (this.hasOwnProperty(key) && key !== "id") {
        this[key] = newData[key];
      }
    });
    this.lastUpdated = new Date();
  }
}

class NovaController {
  constructor() {
    this.ws = null;
    this.isConnected = false;
    this.robots = new Map(); // Map<string, RobotState>
    this.statusBarItem = null;
    this.config = { host: "localhost", port: 8765 };
    this.reconnectTimer = null;

    // Load configuration
    this.loadConfiguration();

    this.connect();
    this.createStatusBar();
  }

  loadConfiguration() {
    const config = vscode.workspace.getConfiguration("nova.websocket");
    this.config.host = config.get("host", "localhost");
    this.config.port = config.get("port", 8765);
    this.autoReconnect = config.get("autoReconnect", true);
    this.reconnectInterval = config.get("reconnectInterval", 3000);
  }

  connect() {
    try {
      const wsUrl = `ws://${this.config.host}:${this.config.port}`;
      if (this.ws) {
        this.ws.close();
      }

      this.ws = new WebSocket(wsUrl);

      this.ws.on("open", () => {
        console.log("[Nova] WebSocket connected");
        this.isConnected = true;
        this.updateStatusBar();

        // Subscribe to events and get initial robot list
        this.sendCommand({ type: "subscribe_events" });
        this.sendCommand({ type: "get_robots" });
      });

      this.ws.on("message", (data) => {
        try {
          const message = JSON.parse(data.toString());
          this.handleMessage(message);
        } catch (error) {
          console.error("[Nova] Failed to parse message:", error);
        }
      });

      this.ws.on("close", () => {
        console.log("[Nova] WebSocket connection closed");
        this.isConnected = false;
        this.robots.clear();
        this.updateStatusBar();
        this.updateSidebar();

        if (this.autoReconnect) {
          this.scheduleReconnect();
        }
      });

      this.ws.on("error", (error) => {
        console.error("[Nova] WebSocket error:", error);
        this.isConnected = false;
        if (this.autoReconnect) {
          this.scheduleReconnect();
        }
      });
    } catch (error) {
      console.log("[Nova] Connection error:", error.message);
      if (this.autoReconnect) {
        this.scheduleReconnect();
      }
    }
  }

  scheduleReconnect() {
    if (!this.autoReconnect) return;

    this.clearReconnectTimer();
    this.reconnectTimer = setTimeout(() => {
      if (!this.isConnected) {
        this.connect();
      }
    }, this.reconnectInterval);
  }

  clearReconnectTimer() {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
  }

  sendCommand(command) {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(command));
      return true;
    }
    return false;
  }

  handleMessage(message) {
    console.log(`[Nova] Handling message type: ${message.type}`);

    switch (message.type) {
      case "robot_list":
        if (message.robots) {
          this.updateRobotList(message.robots);
        }
        break;

      case "robot_state_update":
        this.handleRobotStateUpdate(message);
        break;

      case "playback_event":
        this.handlePlaybackEvent(message);
        break;

      default:
        // Handle simple command responses (success/error messages)
        if (message.hasOwnProperty("success")) {
          this.handleCommandResponse(message);
        }
        break;
    }
  }

  handleRobotStateUpdate(message) {
    console.log(`[Nova] Robot state update for ${message.robot_id}`);

    if (message.robot_id && message.state) {
      this.updateSingleRobot(message.state);
    }
  }

  handlePlaybackEvent(message) {
    console.log(
      `[Nova] Playback event: ${message.event_type} for robot ${message.robot_id}`
    );

    // After any playback event, refresh robot list to get updated state
    this.sendCommand({ type: "get_robots" });
  }

  handleCommandResponse(message) {
    if (message.success) {
      console.log(`[Nova] Command successful`);

      // No need to handle command_id since we don't use confirmations
      // State updates come via robot_state_update messages
    } else {
      console.error(`[Nova] Command failed:`, message.error);
      vscode.window.showErrorMessage(`Nova: ${message.error}`);
    }
  }

  updateRobotList(robots) {
    console.log(`[Nova] Updating robot list with ${robots.length} robots`);

    // Clear existing robots
    this.robots.clear();

    // Add all robots
    robots.forEach((robotData) => {
      const robot = new RobotState(robotData);
      this.robots.set(robot.id, robot);
    });

    this.updateStatusBar();
    this.updateSidebar();
  }

  updateSingleRobot(robotData) {
    const robot = this.robots.get(robotData.id);
    if (robot) {
      robot.update(robotData);
    } else {
      this.robots.set(robotData.id, new RobotState(robotData));
    }

    this.updateStatusBar();
    this.updateSidebar();
  }

  updateSidebar() {
    if (this.sidebarProvider) {
      this.sidebarProvider.updateView();
    }
  }

  createStatusBar() {
    this.statusBarItem = vscode.window.createStatusBarItem(
      vscode.StatusBarAlignment.Left,
      100
    );
    this.statusBarItem.command = "nova.showPanel";
    this.statusBarItem.show();
    this.updateStatusBar();
  }

  updateStatusBar() {
    if (!this.statusBarItem) return;

    const robotCount = this.robots.size;
    const connectedIcon = this.isConnected ? "$(check)" : "$(x)";

    if (robotCount === 0) {
      this.statusBarItem.text = `${connectedIcon} Nova: No robots`;
      this.statusBarItem.tooltip = this.isConnected
        ? "Connected - No robots active"
        : "Disconnected";
    } else {
      this.statusBarItem.text = `${connectedIcon} Nova: ${robotCount} robot${
        robotCount > 1 ? "s" : ""
      }`;
      this.statusBarItem.tooltip = this.isConnected
        ? `Connected - ${robotCount} robot${robotCount > 1 ? "s" : ""} active`
        : "Disconnected";
    }
  }

  async showPanel() {
    const robots = Array.from(this.robots.values());

    if (robots.length === 0) {
      vscode.window.showInformationMessage(
        "No robots are currently registered."
      );
      return;
    }

    const robotItems = robots.map((robot) => ({
      label: `$(radio-tower) ${robot.name}`,
      description: `${robot.state} | ${robot.speed}% | ${robot.direction}`,
      robot: robot,
    }));

    const selected = await vscode.window.showQuickPick(robotItems, {
      placeHolder: "Select a robot to control",
    });

    if (selected) {
      await this.showRobotControls(selected.robot);
    }
  }

  async showRobotControls(robot) {
    const actions = [];

    // Speed control
    actions.push({
      label: "$(gauge) Set Speed",
      description: `Current: ${robot.speed}%`,
      action: "set_speed",
    });

    // Pause/Resume
    if (robot.can_pause) {
      actions.push({
        label: "$(debug-pause) Pause",
        description: "Pause robot execution",
        action: "pause",
      });
    }

    if (robot.can_resume) {
      actions.push({
        label: "$(debug-start) Resume",
        description: "Resume robot execution",
        action: "resume",
      });
    }

    // Direction controls
    actions.push({
      label: "$(arrow-right) Step Forward",
      description: "Execute next step forward",
      action: "step_forward",
    });

    actions.push({
      label: "$(arrow-left) Step Backward",
      description: "Execute next step backward",
      action: "step_backward",
    });

    const selected = await vscode.window.showQuickPick(actions, {
      placeHolder: `Control ${robot.name}`,
    });

    if (selected) {
      await this.executeAction(robot.id, selected.action);
    }
  }

  async executeAction(robotId, action, value = null) {
    const robot = this.robots.get(robotId);
    if (!robot) {
      vscode.window.showErrorMessage("Robot not found");
      return;
    }

    try {
      let command;

      switch (action) {
        case "set_speed":
          let speed;
          if (value !== null && value !== undefined) {
            speed = parseInt(value);
          } else {
            const speedStr = await vscode.window.showInputBox({
              prompt: "Enter speed percentage (0-100)",
              value: robot.speed.toString(),
              validateInput: (input) => {
                const num = parseInt(input);
                if (isNaN(num) || num < 0 || num > 100) {
                  return "Please enter a number between 0 and 100";
                }
                return null;
              },
            });

            if (speedStr === undefined) return;
            speed = parseInt(speedStr);
          }

          command = {
            type: "set_speed",
            robot_id: robotId,
            speed: speed,
          };
          break;

        case "pause":
          command = {
            type: "pause",
            robot_id: robotId,
          };
          break;

        case "resume":
          command = {
            type: "resume",
            robot_id: robotId,
          };
          break;

        case "step_forward":
          command = {
            type: "step_forward",
            robot_id: robotId,
          };
          break;

        case "step_backward":
          command = {
            type: "step_backward",
            robot_id: robotId,
          };
          break;

        default:
          vscode.window.showErrorMessage(`Unknown action: ${action}`);
          return;
      }

      console.log(`[Nova] Sending command:`, command);
      const sent = this.sendCommand(command);

      if (sent) {
        // Show brief success message since we don't get confirmations
        vscode.window.showInformationMessage(
          `Nova: ${action.replace("_", " ")} command sent to ${robot.name}`
        );
      } else {
        vscode.window.showErrorMessage(
          "Failed to send command - not connected"
        );
      }
    } catch (error) {
      console.error(`[Nova] Error executing action ${action}:`, error);
      vscode.window.showErrorMessage(
        `Failed to execute ${action}: ${error.message}`
      );
    }
  }

  dispose() {
    this.clearReconnectTimer();
    if (this.ws) {
      this.ws.close();
    }
    if (this.statusBarItem) {
      this.statusBarItem.dispose();
    }
  }
}

class NovaSidebarProvider {
  constructor(controller) {
    this.controller = controller;
    this.view = null;
  }

  resolveWebviewView(webviewView) {
    this.view = webviewView;
    webviewView.webview.options = {
      enableScripts: true,
      localResourceRoots: [],
    };

    this.updateView();

    // Handle messages from the webview
    webviewView.webview.onDidReceiveMessage((message) => {
      switch (message.command) {
        case "executeAction":
          this.controller.executeAction(
            message.robotId,
            message.action,
            message.value
          );
          break;
        case "refresh":
          this.controller.sendCommand({ type: "get_robots" });
          break;
      }
    });
  }

  updateView() {
    if (!this.view) return;
    this.updateContent();
  }

  updateContent() {
    if (!this.view) return;

    const robots = Array.from(this.controller.robots.values());
    const isConnected = this.controller.isConnected;

    this.view.webview.html = `
      <!DOCTYPE html>
      <html lang="en">
      <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Nova Robot Control</title>
        <style>
          body {
            font-family: var(--vscode-font-family);
            font-size: var(--vscode-font-size);
            color: var(--vscode-foreground);
            background-color: var(--vscode-editor-background);
            margin: 0;
            padding: 16px;
          }
          
          .status {
            margin-bottom: 16px;
            padding: 8px;
            border-radius: 4px;
            background-color: var(--vscode-textBlockQuote-background);
            border-left: 4px solid ${
              isConnected
                ? "var(--vscode-charts-green)"
                : "var(--vscode-charts-red)"
            };
          }
          
          .robot-card {
            margin-bottom: 16px;
            padding: 12px;
            border: 1px solid var(--vscode-panel-border);
            border-radius: 4px;
            background-color: var(--vscode-editor-background);
          }
          
          .robot-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 8px;
          }
          
          .robot-name {
            font-weight: bold;
            color: var(--vscode-textLink-foreground);
          }
          
          .robot-state {
            display: inline-block;
            padding: 2px 8px;
            border-radius: 12px;
            font-size: 0.85em;
            font-weight: bold;
            text-transform: uppercase;
            color: white;
            background-color: ${
              robots.length > 0 ? this.getStateColor(robots[0].state) : "#666"
            };
          }
          
          .robot-info {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 8px;
            margin-bottom: 8px;
            font-size: 0.9em;
          }
          
          .robot-controls {
            display: flex;
            flex-wrap: wrap;
            gap: 8px;
          }
          
          .control-btn {
            flex: 1;
            min-width: 80px;
            padding: 6px 12px;
            border: 1px solid var(--vscode-button-border);
            border-radius: 4px;
            background-color: var(--vscode-button-background);
            color: var(--vscode-button-foreground);
            cursor: pointer;
            font-size: 0.85em;
            text-align: center;
            transition: background-color 0.2s;
          }
          
          .control-btn:hover {
            background-color: var(--vscode-button-hoverBackground);
          }
          
          .control-btn:disabled {
            background-color: var(--vscode-button-secondaryBackground);
            color: var(--vscode-button-secondaryForeground);
            cursor: not-allowed;
            opacity: 0.6;
          }
          
          .refresh-btn {
            width: 100%;
            padding: 8px;
            margin-top: 16px;
            border: 1px solid var(--vscode-button-border);
            border-radius: 4px;
            background-color: var(--vscode-button-secondaryBackground);
            color: var(--vscode-button-secondaryForeground);
            cursor: pointer;
            font-size: 0.9em;
          }
          
          .refresh-btn:hover {
            background-color: var(--vscode-button-secondaryHoverBackground);
          }
          
          .no-robots {
            text-align: center;
            padding: 32px;
            color: var(--vscode-descriptionForeground);
          }
        </style>
      </head>
      <body>
        <div class="status">
          <strong>Status:</strong> ${
            isConnected ? "✅ Connected" : "❌ Disconnected"
          }
        </div>
        
        ${
          robots.length === 0
            ? `
          <div class="no-robots">
            <p>No robots are currently registered.</p>
            <p>Start a Nova program with WebSocket control to see robots here.</p>
          </div>
        `
            : ""
        }
        
        ${robots
          .map(
            (robot) => `
          <div class="robot-card">
            <div class="robot-header">
              <span class="robot-name">${robot.name}</span>
              <span class="robot-state" style="background-color: ${this.getStateColor(
                robot.state
              )}">${robot.state}</span>
            </div>
            
            <div class="robot-info">
              <div><strong>Speed:</strong> ${robot.speed}%</div>
              <div><strong>Direction:</strong> ${robot.direction}</div>
            </div>
            
            <div class="robot-controls">
              <button class="control-btn" onclick="setSpeed('${robot.id}', ${
              robot.speed
            })">
                Set Speed
              </button>
              <button class="control-btn" ${robot.can_pause ? "" : "disabled"} 
                onclick="executeAction('${robot.id}', 'pause')">
                Pause
              </button>
              <button class="control-btn" ${robot.can_resume ? "" : "disabled"} 
                onclick="executeAction('${robot.id}', 'resume')">
                Resume
              </button>
              <button class="control-btn" onclick="executeAction('${
                robot.id
              }', 'step_forward')">
                Step →
              </button>
              <button class="control-btn" onclick="executeAction('${
                robot.id
              }', 'step_backward')">
                ← Step
              </button>
            </div>
          </div>
        `
          )
          .join("")}
        
        <button class="refresh-btn" onclick="refresh()">
          🔄 Refresh
        </button>
        
        <script>
          const vscode = acquireVsCodeApi();
          
          function executeAction(robotId, action, value = null) {
            vscode.postMessage({
              command: 'executeAction',
              robotId: robotId,
              action: action,
              value: value
            });
          }
          
          function setSpeed(robotId, currentSpeed) {
            const speed = prompt('Enter speed percentage (0-100):', currentSpeed);
            if (speed !== null) {
              const speedNum = parseInt(speed);
              if (!isNaN(speedNum) && speedNum >= 0 && speedNum <= 100) {
                executeAction(robotId, 'set_speed', speedNum);
              } else {
                alert('Please enter a number between 0 and 100');
              }
            }
          }
          
          function refresh() {
            vscode.postMessage({
              command: 'refresh'
            });
          }
        </script>
      </body>
      </html>
    `;
  }

  getStateColor(state) {
    switch (state) {
      case "executing":
      case "playing":
        return "#4CAF50";
      case "paused":
        return "#FF9800";
      case "idle":
        return "#2196F3";
      default:
        return "#666";
    }
  }
}

function activate(context) {
  console.log("[Nova] Extension activating...");

  const controller = new NovaController();

  // Create and register sidebar provider
  const sidebarProvider = new NovaSidebarProvider(controller);
  controller.sidebarProvider = sidebarProvider;

  // Register webview provider
  const sidebarRegistration = vscode.window.registerWebviewViewProvider(
    "nova.robotControlView",
    sidebarProvider
  );

  // Register commands
  const showPanelCommand = vscode.commands.registerCommand(
    "nova.showPanel",
    () => controller.showPanel()
  );

  const showControlPanelCommand = vscode.commands.registerCommand(
    "nova.showControlPanel",
    () => controller.showPanel()
  );

  const refreshRobotsCommand = vscode.commands.registerCommand(
    "nova.refreshRobots",
    () => {
      controller.sendCommand({ type: "get_robots" });
      vscode.window.showInformationMessage("Refreshed robot list");
    }
  );

  // Listen for configuration changes
  const configurationChangeListener = vscode.workspace.onDidChangeConfiguration(
    (event) => {
      if (event.affectsConfiguration("nova.websocket")) {
        controller.loadConfiguration();
        controller.connect();
      }
    }
  );

  // Add all subscriptions
  context.subscriptions.push(
    showPanelCommand,
    showControlPanelCommand,
    refreshRobotsCommand,
    sidebarRegistration,
    configurationChangeListener,
    controller
  );

  console.log("[Nova] Extension activated successfully");
}

function deactivate() {}

module.exports = { activate, deactivate };
