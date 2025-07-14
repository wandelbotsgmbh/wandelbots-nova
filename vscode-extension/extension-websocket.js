/**
 * Nova Robot Control VS Code Extension - WebSocket Version (Enhanced)
 *
 * Enhanced extension with comprehensive robot control functionality:
 * - Real-time event broadcasting from Nova WebSocket server
 * - Robot registration and discovery events
 * - Support for parallel robot execution
 * - Enhanced UI with better state tracking
 * - Program lifecycle events
 */

const vscode = require("vscode");
const WebSocket = require("ws");

class NovaController {
  constructor() {
    this.ws = null;
    this.isConnected = false;
    this.robots = new Map();
    this.statusBarItem = null;
    this.config = { host: "localhost", port: 8765 };
    this.reconnectTimer = null;
    this.lastConnectionAttempt = 0;
    this.connectionId = Date.now(); // Track connection sessions

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
    // Prevent rapid connection attempts
    const now = Date.now();
    if (now - this.lastConnectionAttempt < 1000) {
      return;
    }
    this.lastConnectionAttempt = now;

    try {
      const wsUrl = `ws://${this.config.host}:${this.config.port}`;
      if (this.ws) {
        this.ws.close();
      }

      this.ws = new WebSocket(wsUrl);

      this.ws.on("open", () => {
        console.log("[Nova] WebSocket connected successfully");
        vscode.window.showInformationMessage("Nova: Connected successfully!");
        this.isConnected = true;

        // Subscribe to enhanced event system
        this.sendCommand({ type: "subscribe_events" });

        this.updateStatusBar();
        this.clearReconnectTimer();
      });

      this.ws.on("message", (data) => {
        try {
          const message = JSON.parse(data.toString());
          console.log("[Nova] Received message:", message.type || "unknown");
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

        if (this.autoReconnect) {
          this.scheduleReconnect();
        }
      });

      this.ws.on("error", (error) => {
        this.isConnected = false;

        if (this.autoReconnect) {
          this.scheduleReconnect();
        }
      });
    } catch (error) {
      console.log("[Nova] Connection error:", error.message);
      vscode.window.showErrorMessage(`Nova connection error: ${error.message}`);

      if (this.autoReconnect) {
        this.scheduleReconnect();
      }
    }
  }

  scheduleReconnect() {
    if (!this.autoReconnect) {
      return;
    }

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
    // Handle different message types from enhanced WebSocket interface
    switch (message.type) {
      case "playback_event":
        this.handlePlaybackEvent(message);
        break;
      case "state_change":
        this.handleStateChange(message);
        break;
      case "speed_change":
        this.handleSpeedChange(message);
        break;
      default:
        // Handle legacy robot list updates
        if (message.robots) {
          this.updateRobotList(message.robots);
        } else if (message.robot) {
          this.updateSingleRobot(message.robot);
        }
        break;
    }
  }

  handlePlaybackEvent(message) {
    console.log(
      `[Nova] Playback event: ${message.event_type} for robot ${message.robot_id}`
    );

    switch (message.event_type) {
      case "robot_registered":
        this.onRobotRegistered(message);
        break;
      case "robot_unregistered":
        this.onRobotUnregistered(message);
        break;
      case "speed_change":
        this.onSpeedChanged(message);
        break;
      case "state_change":
        this.onStateChanged(message);
        break;
      case "execution_started":
        this.onExecutionStarted(message);
        break;
      case "execution_stopped":
        this.onExecutionStopped(message);
        break;
      case "program_started":
        this.onProgramStarted(message);
        break;
      case "program_stopped":
        this.onProgramStopped(message);
        break;
    }

    // Request updated robot list to stay in sync
    this.sendCommand({ type: "get_robots" });
  }

  handleStateChange(message) {
    // Legacy state change handler
    if (message.robot_id) {
      this.updateRobotState(message.robot_id, message.state);
    }
  }

  handleSpeedChange(message) {
    // Legacy speed change handler
    if (message.robot_id) {
      this.updateRobotSpeed(message.robot_id, message.speed);
    }
  }

  onRobotRegistered(message) {
    console.log(`[Nova] Robot registered: ${message.robot_id}`);
    vscode.window.showInformationMessage(
      `Nova: Robot ${message.robot_name || message.robot_id} registered`
    );
  }

  onRobotUnregistered(message) {
    console.log(`[Nova] Robot unregistered: ${message.robot_id}`);
    this.robots.delete(message.robot_id);
    this.updateStatusBar();
    this.updateSidebar();
  }

  onSpeedChanged(message) {
    console.log(
      `[Nova] Speed changed for ${message.robot_id}: ${message.old_speed}% → ${message.new_speed}%`
    );
    if (this.robots.has(message.robot_id)) {
      const robot = this.robots.get(message.robot_id);
      robot.speed = message.new_speed;
      this.robots.set(message.robot_id, robot);
      this.updateStatusBar();
      this.updateSidebar();
    }
  }

  onStateChanged(message) {
    console.log(
      `[Nova] State changed for ${message.robot_id}: ${message.old_state} → ${message.new_state}`
    );
    if (this.robots.has(message.robot_id)) {
      const robot = this.robots.get(message.robot_id);
      robot.state = message.new_state;
      robot.speed = message.speed;
      this.robots.set(message.robot_id, robot);
      this.updateStatusBar();
      this.updateSidebar();
    }
  }

  onExecutionStarted(message) {
    console.log(
      `[Nova] Execution started for ${message.robot_id} at ${message.speed}%`
    );
    this.updateRobotState(message.robot_id, "executing");
    this.updateRobotSpeed(message.robot_id, message.speed);
  }

  onExecutionStopped(message) {
    console.log(`[Nova] Execution stopped for ${message.robot_id}`);
    this.updateRobotState(message.robot_id, "idle");
  }

  onProgramStarted(message) {
    console.log(
      `[Nova] Program started: ${message.program_name} with ${message.total_robots} robots`
    );
    vscode.window.showInformationMessage(
      `Nova: Program "${message.program_name}" started with ${message.total_robots} robot(s)`
    );
  }

  onProgramStopped(message) {
    console.log(`[Nova] Program stopped: ${message.program_name}`);
    vscode.window.showInformationMessage(
      `Nova: Program "${message.program_name}" stopped`
    );
  }

  updateRobotState(robotId, state) {
    if (this.robots.has(robotId)) {
      const robot = this.robots.get(robotId);
      robot.state = state;
      this.robots.set(robotId, robot);
      this.updateStatusBar();
      this.updateSidebar();
    }
  }

  updateRobotSpeed(robotId, speed) {
    if (this.robots.has(robotId)) {
      const robot = this.robots.get(robotId);
      robot.speed = speed;
      this.robots.set(robotId, robot);
      this.updateStatusBar();
      this.updateSidebar();
    }
  }

  updateRobotList(robots) {
    this.robots.clear();
    robots.forEach((robot) => {
      // Enhance robot data with additional metadata
      const enhancedRobot = {
        ...robot,
        displayName: robot.name || robot.id,
        lastUpdated: new Date(),
        isOnline: true,
      };
      this.robots.set(robot.id, enhancedRobot);
    });
    this.updateStatusBar();
    this.updateSidebar();
  }

  updateSingleRobot(robot) {
    if (robot?.id) {
      const existingRobot = this.robots.get(robot.id);
      const enhancedRobot = {
        ...existingRobot,
        ...robot,
        displayName: robot.name || robot.id,
        lastUpdated: new Date(),
        isOnline: true,
      };
      this.robots.set(robot.id, enhancedRobot);
      this.updateStatusBar();
      this.updateSidebar();
    }
  }

  updateSidebar() {
    // Update sidebar if available
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
  }

  updateStatusBar() {
    const robotCount = this.robots.size;
    const activeRobots = Array.from(this.robots.values()).filter(
      (robot) => robot.state === "executing"
    ).length;
    const pausedRobots = Array.from(this.robots.values()).filter(
      (robot) => robot.state === "paused"
    ).length;

    if (!this.isConnected) {
      this.statusBarItem.text = "$(robot) Nova: Not Running";
      this.statusBarItem.tooltip = "Nova not running - Click to retry";
      this.statusBarItem.backgroundColor = new vscode.ThemeColor(
        "statusBarItem.warningBackground"
      );
    } else if (robotCount === 0) {
      this.statusBarItem.text = "$(robot) Nova: Ready";
      this.statusBarItem.tooltip = "Nova connected - No robots registered yet";
      this.statusBarItem.backgroundColor = undefined;
    } else {
      // Enhanced status with more detailed info
      let statusText = `$(robot) Nova: ${activeRobots}`;
      if (pausedRobots > 0) {
        statusText += ` (${pausedRobots} paused)`;
      }
      statusText += `/${robotCount}`;

      this.statusBarItem.text = statusText;
      this.statusBarItem.tooltip = `${activeRobots} executing, ${pausedRobots} paused, ${robotCount} total robots - Click to control`;

      if (activeRobots > 0) {
        this.statusBarItem.backgroundColor = new vscode.ThemeColor(
          "statusBarItem.prominentBackground"
        );
      } else {
        this.statusBarItem.backgroundColor = undefined;
      }
    }
  }

  async showPanel() {
    if (!this.isConnected) {
      const retry = await vscode.window.showInformationMessage(
        "Nova not connected. Retry connection?",
        "Retry",
        "Settings",
        "Cancel"
      );
      if (retry === "Retry") {
        this.connect();
      } else if (retry === "Settings") {
        vscode.commands.executeCommand(
          "workbench.action.openSettings",
          "nova.websocket"
        );
      }
      return;
    }

    const robots = Array.from(this.robots.values());
    if (robots.length === 0) {
      const action = await vscode.window.showInformationMessage(
        "No robots registered yet. Make sure a Nova program with external control is running.",
        "Refresh",
        "Help"
      );
      if (action === "Refresh") {
        this.sendCommand({ type: "get_robots" });
      } else if (action === "Help") {
        vscode.commands.executeCommand("nova.showHelp");
      }
      return;
    }

    // Sort robots by state (executing first, then paused, then idle)
    const sortedRobots = robots.sort((a, b) => {
      const stateOrder = { executing: 0, paused: 1, idle: 2 };
      return (stateOrder[a.state] || 3) - (stateOrder[b.state] || 3);
    });

    const robotItems = sortedRobots.map((robot) => {
      let icon = "robot";
      if (robot.state === "executing") icon = "play";
      else if (robot.state === "paused") icon = "debug-pause";

      return {
        label: `$(${icon}) ${robot.displayName}`,
        description: `${robot.speed}% - ${robot.state}${
          robot.registered_at
            ? ` (registered ${new Date(
                robot.registered_at
              ).toLocaleTimeString()})`
            : ""
        }`,
        detail: robot.id !== robot.displayName ? `ID: ${robot.id}` : undefined,
        robot,
      };
    });

    const selected = await vscode.window.showQuickPick(robotItems, {
      placeHolder: "Select robot to control",
      matchOnDescription: true,
      matchOnDetail: true,
    });

    if (selected) {
      await this.showRobotControls(selected.robot);
    }
  }

  async showRobotControls(robot) {
    const actions = [];

    // Speed control with current speed display
    actions.push({
      label: "$(gauge) Change Speed",
      description: `Current: ${robot.speed}%`,
      action: "speed",
    });

    // Quick speed presets
    const speedPresets = [10, 25, 50, 75, 100];
    speedPresets.forEach((speed) => {
      if (speed !== robot.speed) {
        actions.push({
          label: `$(zap) Set Speed ${speed}%`,
          description: `Quick set to ${speed}%`,
          action: "quick_speed",
          value: speed,
        });
      }
    });

    // Pause/Resume based on current state
    if (robot.can_pause && robot.state !== "paused") {
      actions.push({
        label: "$(debug-pause) Pause",
        description: "Pause robot execution",
        action: "pause",
      });
    }

    if (robot.can_resume && robot.state === "paused") {
      actions.push({
        label: "$(debug-start) Resume",
        description: "Resume robot execution",
        action: "resume",
      });
    }

    // Movement controls
    actions.push(
      {
        label: "$(arrow-right) Step Forward",
        description: "Move execution forward",
        action: "forward",
      },
      {
        label: "$(arrow-left) Step Backward",
        description: "Move execution backward",
        action: "backward",
      }
    );

    // Refresh robot status
    actions.push({
      label: "$(refresh) Refresh Status",
      description: "Update robot information",
      action: "refresh",
    });

    const selected = await vscode.window.showQuickPick(actions, {
      placeHolder: `Control ${robot.displayName} (${robot.state}, ${robot.speed}%)`,
      matchOnDescription: true,
    });

    if (selected) {
      if (selected.action === "quick_speed") {
        await this.executeAction(robot.id, "speed", selected.value);
      } else {
        await this.executeAction(robot.id, selected.action);
      }
    }
  }

  async executeAction(robotId, action, value = null) {
    let success = false;
    let message = "";

    try {
      switch (action) {
        case "speed":
          let speedInput;
          if (value !== null) {
            speedInput = value.toString();
          } else {
            speedInput = await vscode.window.showInputBox({
              prompt: "Enter speed (0-100)",
              placeHolder: "e.g., 50",
              validateInput: (value) => {
                const num = parseInt(value);
                return isNaN(num) || num < 0 || num > 100
                  ? "Enter a number between 0 and 100"
                  : null;
              },
            });
          }

          if (speedInput) {
            const speed = parseInt(speedInput);
            success = this.sendCommand({
              type: "set_speed",
              robot_id: robotId,
              speed: speed,
            });
            message = `Speed set to ${speed}%`;
          }
          break;

        case "pause":
          success = this.sendCommand({ type: "pause", robot_id: robotId });
          message = "Robot paused";
          break;

        case "resume":
          success = this.sendCommand({ type: "resume", robot_id: robotId });
          message = "Robot resumed";
          break;

        case "forward":
          success = this.sendCommand({
            type: "step_forward",
            robot_id: robotId,
          });
          message = "Moving forward";
          break;

        case "backward":
          success = this.sendCommand({
            type: "step_backward",
            robot_id: robotId,
          });
          message = "Moving backward";
          break;

        case "refresh":
          success = this.sendCommand({ type: "get_robots" });
          message = "Robot status refreshed";
          break;

        default:
          throw new Error(`Unknown action: ${action}`);
      }

      if (success) {
        vscode.window.showInformationMessage(`✓ ${message}`);
      } else {
        vscode.window.showErrorMessage(
          `✗ Failed to ${action} robot - Not connected`
        );
      }
    } catch (error) {
      console.error(`[Nova] Error executing action ${action}:`, error);
      vscode.window.showErrorMessage(`✗ Error: ${error.message}`);
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
    this._view = null;
  }

  resolveWebviewView(webviewView) {
    this._view = webviewView;
    webviewView.webview.options = {
      enableScripts: true,
      localResourceRoots: [],
    };

    // Handle messages from webview
    webviewView.webview.onDidReceiveMessage(async (message) => {
      const { command, robotId, value } = message;

      try {
        switch (command) {
          case "pause":
            await this.controller.executeAction(robotId, "pause");
            break;
          case "resume":
            await this.controller.executeAction(robotId, "resume");
            break;
          case "setSpeed":
            await this.controller.executeAction(robotId, "speed", value);
            break;
          case "forward":
            await this.controller.executeAction(robotId, "forward");
            break;
          case "backward":
            await this.controller.executeAction(robotId, "backward");
            break;
          case "refresh":
            this.controller.sendCommand({ type: "get_robots" });
            break;
          case "showDetails":
            await this.controller.showRobotControls(
              this.controller.robots.get(robotId)
            );
            break;
          default:
            console.warn(`[Nova] Unknown webview command: ${command}`);
        }
      } catch (error) {
        console.error(`[Nova] Error handling webview command:`, error);
        vscode.window.showErrorMessage(`Error: ${error.message}`);
      }
    });

    this.updateContent();
  }

  updateView() {
    this.updateContent();
  }

  updateContent() {
    if (!this._view) return;

    const robots = Array.from(this.controller.robots.values());
    const isConnected = this.controller.isConnected;
    const totalRobots = robots.length;
    const executingRobots = robots.filter(
      (r) => r.state === "executing"
    ).length;
    const pausedRobots = robots.filter((r) => r.state === "paused").length;

    // Keep robots in their original order to avoid confusing shuffling
    const sortedRobots = robots;

    this._view.webview.html = `<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body {
            font-family: var(--vscode-font-family);
            padding: 12px;
            color: var(--vscode-foreground);
            font-size: 13px;
        }
        .status {
            margin-bottom: 16px;
            padding: 12px;
            border-radius: 6px;
            background: var(--vscode-textBlockQuote-background);
            border-left: 3px solid var(--vscode-textBlockQuote-border);
        }
        .status-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 8px;
            margin-top: 8px;
        }
        .status-item {
            display: flex;
            justify-content: space-between;
            font-size: 12px;
        }
        .robot {
            margin-bottom: 12px;
            padding: 12px;
            border: 1px solid var(--vscode-panel-border);
            border-radius: 6px;
            transition: background-color 0.2s;
        }
        .robot:hover {
            background: var(--vscode-list-hoverBackground);
        }
        .robot.executing {
            border-left: 3px solid var(--vscode-charts-green);
        }
        .robot.paused {
            border-left: 3px solid var(--vscode-charts-orange);
        }
        .robot.idle {
            border-left: 3px solid var(--vscode-charts-gray);
        }
        .robot-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 10px;
        }
        .robot-name {
            font-weight: 600;
            color: var(--vscode-foreground);
        }
        .robot-id {
            font-size: 11px;
            color: var(--vscode-descriptionForeground);
            margin-top: 2px;
        }
        .robot-state {
            font-size: 11px;
            padding: 3px 8px;
            border-radius: 4px;
            font-weight: 500;
            text-transform: uppercase;
        }
        .robot-state.executing {
            background: var(--vscode-charts-green);
            color: var(--vscode-charts-foreground);
        }
        .robot-state.paused {
            background: var(--vscode-charts-orange);
            color: var(--vscode-charts-foreground);
        }
        .robot-state.idle {
            background: var(--vscode-charts-gray);
            color: var(--vscode-charts-foreground);
        }
        .controls {
            display: flex;
            gap: 6px;
            margin-bottom: 10px;
            flex-wrap: wrap;
        }
        button {
            background: var(--vscode-button-background);
            color: var(--vscode-button-foreground);
            border: none;
            padding: 6px 12px;
            border-radius: 4px;
            cursor: pointer;
            font-size: 11px;
            font-weight: 500;
            transition: background-color 0.2s;
        }
        button:hover:not(:disabled) {
            background: var(--vscode-button-hoverBackground);
        }
        button:disabled {
            opacity: 0.5;
            cursor: not-allowed;
        }
        button.secondary {
            background: var(--vscode-button-secondaryBackground);
            color: var(--vscode-button-secondaryForeground);
        }
        .speed-control {
            display: flex;
            align-items: center;
            gap: 10px;
            margin-bottom: 8px;
        }
        input[type="range"] {
            flex: 1;
            height: 4px;
        }
        .speed-value {
            min-width: 40px;
            font-size: 12px;
            font-weight: 600;
            color: var(--vscode-charts-blue);
        }
        .robot-meta {
            font-size: 11px;
            color: var(--vscode-descriptionForeground);
            margin-top: 8px;
        }
        .no-robots {
            text-align: center;
            color: var(--vscode-descriptionForeground);
            margin: 30px 0;
            padding: 20px;
        }
        .no-robots-icon {
            font-size: 24px;
            margin-bottom: 8px;
        }
        .refresh-btn {
            background: var(--vscode-button-secondaryBackground);
            color: var(--vscode-button-secondaryForeground);
            width: 100%;
            margin-top: 12px;
            padding: 10px;
        }
        .connection-status {
            display: flex;
            align-items: center;
            gap: 8px;
            font-weight: 600;
        }
        .status-indicator {
            width: 8px;
            height: 8px;
            border-radius: 50%;
        }
        .status-indicator.connected {
            background: var(--vscode-charts-green);
        }
        .status-indicator.disconnected {
            background: var(--vscode-charts-red);
        }
    </style>
</head>
<body>
    <div class="status">
        <div class="connection-status">
            <div class="status-indicator ${
              isConnected ? "connected" : "disconnected"
            }"></div>
            <span>${isConnected ? "Connected" : "Disconnected"}</span>
        </div>
        
        ${
          totalRobots > 0
            ? `
        <div class="status-grid">
            <div class="status-item">
                <span>Total:</span>
                <span>${totalRobots}</span>
            </div>
            <div class="status-item">
                <span>Executing:</span>
                <span>${executingRobots}</span>
            </div>
            <div class="status-item">
                <span>Paused:</span>
                <span>${pausedRobots}</span>
            </div>
            <div class="status-item">
                <span>Idle:</span>
                <span>${totalRobots - executingRobots - pausedRobots}</span>
            </div>
        </div>
        `
            : ""
        }
    </div>

    ${
      totalRobots > 0
        ? sortedRobots
            .map(
              (robot) => `
        <div class="robot ${robot.state}">
            <div class="robot-header">
                <div>
                    <div class="robot-name">${robot.displayName}</div>
                </div>
                <span class="robot-state ${robot.state}">${robot.state}</span>
            </div>
            
            <div class="controls">
                <button onclick="stepBackward('${robot.id}')" ${
                robot.state !== "paused" ? "disabled" : ""
              }>
                    ⏮ Backward
                </button>
                <button onclick="pauseRobot('${robot.id}')" ${
                robot.state === "paused" || robot.state === "idle"
                  ? "disabled"
                  : ""
              }>
                    ⏸ Pause
                </button>
                <button onclick="stepForward('${robot.id}')" ${
                robot.state !== "paused" ? "disabled" : ""
              }>
                    ▶ Forward
                </button>
            </div>
            
            <div class="speed-control">
                <span>Speed:</span>
                <input type="range" min="1" max="100" value="${robot.speed}" 
                       onchange="setSpeed('${robot.id}', this.value)"
                       oninput="updateSpeedDisplay('${robot.id}', this.value)">
                <span class="speed-value" id="speed-${robot.id}">${
                robot.speed
              }%</span>
            </div>
        </div>
    `
            )
            .join("")
        : `
        <div class="no-robots">
            <div class="no-robots-icon">🤖</div>
            <div>No robots registered</div>
            <small>Start a Nova program with external control to see robots here</small>
        </div>
    `
    }

    <script>
        const vscode = acquireVsCodeApi();
        
        function pauseRobot(robotId) {
            vscode.postMessage({ command: 'pause', robotId });
        }
        
        function resumeRobot(robotId) {
            vscode.postMessage({ command: 'resume', robotId });
        }
        
        function stepForward(robotId) {
            vscode.postMessage({ command: 'forward', robotId });
        }
        
        function stepBackward(robotId) {
            vscode.postMessage({ command: 'backward', robotId });
        }
        
        function setSpeed(robotId, speed) {
            vscode.postMessage({ command: 'setSpeed', robotId, value: parseInt(speed) });
        }
        
        function updateSpeedDisplay(robotId, speed) {
            const display = document.getElementById('speed-' + robotId);
            if (display) {
                display.textContent = speed + '%';
            }
        }
        
        function showDetails(robotId) {
            vscode.postMessage({ command: 'showDetails', robotId });
        }
        
        function refresh() {
            vscode.postMessage({ command: 'refresh' });
        }
    </script>
</body>
</html>`;
  }
}

function activate(context) {
  console.log("[Nova] Extension activating...");
  vscode.window.showInformationMessage("Nova extension is activating...");

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
      vscode.window.showInformationMessage("Refreshing robot list...");
    }
  );

  const showHelpCommand = vscode.commands.registerCommand(
    "nova.showHelp",
    () => {
      showNovaHelp();
    }
  );

  const focusCommand = vscode.commands.registerCommand("nova.focus", () => {
    vscode.commands.executeCommand("nova.robotControlView.focus");
  });

  // Listen for configuration changes
  const configurationChangeListener = vscode.workspace.onDidChangeConfiguration(
    (event) => {
      if (event.affectsConfiguration("nova.websocket")) {
        console.log("[Nova] Configuration changed, reloading...");
        controller.loadConfiguration();
        if (controller.isConnected) {
          vscode.window.showInformationMessage(
            "Nova: Configuration updated. Reconnecting..."
          );
          controller.connect();
        }
      }
    }
  );

  // Add all subscriptions
  context.subscriptions.push(
    showPanelCommand,
    showControlPanelCommand,
    refreshRobotsCommand,
    showHelpCommand,
    focusCommand,
    sidebarRegistration,
    configurationChangeListener,
    controller
  );

  console.log("[Nova] Extension activated successfully");
  vscode.window.showInformationMessage(
    "Nova extension activated! Check status bar for connection status."
  );
}

async function showNovaHelp() {
  const helpContent = `
# Nova Robot Control Extension

This extension provides real-time control of Nova robots through WebSocket connection.

## Features
- **Real-time robot discovery**: Automatically detects registered robots
- **Speed control**: Adjust execution speed from 0-100%
- **Pause/Resume**: Control execution flow
- **Direction control**: Step forward/backward through execution
- **Multiple robot support**: Control multiple robots simultaneously
- **Live status updates**: Real-time state and speed monitoring

## Getting Started
1. Start a Nova program with \`external_control=nova.external_control.WebSocketControl()\`
2. The extension will automatically connect to the WebSocket server
3. View robots in the sidebar or use Command Palette commands

## Commands
- **Nova: Show Robot Controls** - Quick pick interface for robot control
- **Nova: Show Robot Control Panel** - Show sidebar panel
- **Nova: Refresh Robot List** - Manually refresh robot status
- **Nova: Show Help** - Show this help information

## Configuration
- **nova.websocket.host**: WebSocket server host (default: localhost)
- **nova.websocket.port**: WebSocket server port (default: 8765)
- **nova.websocket.autoReconnect**: Auto-reconnect on connection loss
- **nova.websocket.reconnectInterval**: Reconnection interval in milliseconds

## Troubleshooting
- Ensure Nova program is running with WebSocket control enabled
- Check that no firewall is blocking port 8765
- Verify host and port settings in VS Code settings
- Use "Refresh Robot List" if robots don't appear

## Example Nova Program
\`\`\`python
@nova.program(
    name="My Robot Program",
    external_control=nova.external_control.WebSocketControl()
)
async def my_program():
    # Your robot code here
    pass
\`\`\`
  `;

  const panel = vscode.window.createWebviewPanel(
    "novaHelp",
    "Nova Robot Control Help",
    vscode.ViewColumn.One,
    { enableScripts: false }
  );

  panel.webview.html = `
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <style>
            body {
                font-family: var(--vscode-font-family);
                line-height: 1.6;
                color: var(--vscode-foreground);
                max-width: 800px;
                margin: 0 auto;
                padding: 20px;
            }
            h1, h2, h3 { color: var(--vscode-textLink-foreground); }
            code {
                background: var(--vscode-textCodeBlock-background);
                padding: 2px 4px;
                border-radius: 3px;
                font-family: var(--vscode-editor-font-family);
            }
            pre {
                background: var(--vscode-textCodeBlock-background);
                padding: 16px;
                border-radius: 6px;
                overflow-x: auto;
                border-left: 3px solid var(--vscode-textBlockQuote-border);
            }
            ul { padding-left: 20px; }
            li { margin-bottom: 4px; }
        </style>
    </head>
    <body>
        ${helpContent
          .split("\n")
          .map((line) => {
            if (line.startsWith("# ")) return `<h1>${line.slice(2)}</h1>`;
            if (line.startsWith("## ")) return `<h2>${line.slice(3)}</h2>`;
            if (line.startsWith("- ")) return `<li>${line.slice(2)}</li>`;
            if (line.startsWith("```"))
              return line.includes("python") ? "<pre><code>" : "</code></pre>";
            if (line.trim() === "") return "<br>";
            return `<p>${line}</p>`;
          })
          .join("")}
    </body>
    </html>
  `;
}

function deactivate() {}

module.exports = { activate, deactivate };
