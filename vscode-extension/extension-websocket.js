/**
 * Nova Robot Control VS Code Extension - WebSocket Version (Lean)
 *
 * Simplified extension focused on core robot control functionality.
 * Removed redundant popup and sidebar implementations.
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

    this.connect();
    this.createStatusBar();
  }

  connect() {
    try {
      const wsUrl = `ws://${this.config.host}:${this.config.port}`;
      console.log(`[Nova] Attempting to connect to ${wsUrl}`);
      vscode.window.showInformationMessage(`Nova: Connecting to ${wsUrl}...`);
      this.ws = new WebSocket(wsUrl);

      this.ws.on("open", () => {
        console.log("[Nova] WebSocket connected successfully");
        vscode.window.showInformationMessage("Nova: Connected successfully!");
        this.isConnected = true;
        this.sendCommand({ type: "subscribe_events" });
        this.updateStatusBar();
        this.clearReconnectTimer();
      });

      this.ws.on("message", (data) => {
        const message = JSON.parse(data.toString());
        console.log("[Nova] Received message:", message.type);
        this.handleMessage(message);
      });

      this.ws.on("close", () => {
        console.log("[Nova] WebSocket connection closed");
        this.isConnected = false;
        this.robots.clear();
        this.updateStatusBar();
        this.scheduleReconnect();
      });

      this.ws.on("error", (error) => {
        console.log("[Nova] WebSocket error:", error.message);
        vscode.window.showErrorMessage(
          `Nova WebSocket error: ${error.message}`
        );
        this.isConnected = false;
        this.scheduleReconnect();
      });
    } catch (error) {
      console.log("[Nova] Connection error:", error.message);
      vscode.window.showErrorMessage(`Nova connection error: ${error.message}`);
      this.scheduleReconnect();
    }
  }

  scheduleReconnect() {
    this.clearReconnectTimer();
    this.reconnectTimer = setTimeout(() => this.connect(), 3000);
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
    if (message.robots) {
      this.updateRobotList(message.robots);
    } else if (message.robot) {
      this.updateSingleRobot(message.robot);
    }
  }

  updateRobotList(robots) {
    this.robots.clear();
    robots.forEach((robot) => this.robots.set(robot.id, robot));
    this.updateStatusBar();
    // Update sidebar if available
    if (this.sidebarProvider) {
      this.sidebarProvider.updateView();
    }
  }

  updateSingleRobot(robot) {
    if (robot?.id) {
      this.robots.set(robot.id, robot);
      this.updateStatusBar();
      // Update sidebar if available
      if (this.sidebarProvider) {
        this.sidebarProvider.updateView();
      }
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

    if (!this.isConnected) {
      this.statusBarItem.text = "$(robot) Nova: Disconnected";
      this.statusBarItem.tooltip = "Nova not connected - Click to retry";
    } else if (robotCount === 0) {
      this.statusBarItem.text = "$(robot) Nova: No robots";
      this.statusBarItem.tooltip = "No robots running";
    } else {
      this.statusBarItem.text = `$(robot) Nova: ${activeRobots}/${robotCount}`;
      this.statusBarItem.tooltip = `${activeRobots} active robots - Click to control`;
      this.statusBarItem.backgroundColor = new vscode.ThemeColor(
        "statusBarItem.prominentBackground"
      );
    }
  }

  async showPanel() {
    if (!this.isConnected) {
      const retry = await vscode.window.showInformationMessage(
        "Nova not connected. Retry connection?",
        "Retry",
        "Cancel"
      );
      if (retry === "Retry") this.connect();
      return;
    }

    const robots = Array.from(this.robots.values());
    if (robots.length === 0) {
      vscode.window.showInformationMessage(
        "No robots found. Start a Nova program first."
      );
      return;
    }

    const robotItems = robots.map((robot) => ({
      label: `$(${
        robot.state === "executing"
          ? "play"
          : robot.state === "paused"
          ? "debug-pause"
          : "robot"
      }) ${robot.id}`,
      description: `${robot.speed}% - ${robot.state}`,
      robot,
    }));

    const selected = await vscode.window.showQuickPick(robotItems, {
      placeHolder: "Select robot to control",
    });

    if (selected) {
      await this.showRobotControls(selected.robot);
    }
  }

  async showRobotControls(robot) {
    const actions = [];

    // Speed control
    actions.push({
      label: "$(gauge) Change Speed",
      description: `Current: ${robot.speed}%`,
      action: "speed",
    });

    // Pause/Resume
    if (robot.can_pause) {
      actions.push({
        label: "$(debug-pause) Pause",
        action: "pause",
      });
    }

    if (robot.can_resume) {
      actions.push({
        label: "$(debug-start) Resume",
        action: "resume",
      });
    }

    // Movement
    actions.push(
      {
        label: "$(arrow-right) Step Forward",
        action: "forward",
      },
      {
        label: "$(arrow-left) Step Backward",
        action: "backward",
      }
    );

    const selected = await vscode.window.showQuickPick(actions, {
      placeHolder: `Control ${robot.id}`,
    });

    if (selected) {
      await this.executeAction(robot.id, selected.action);
    }
  }

  async executeAction(robotId, action) {
    let success = false;
    let message = "";

    switch (action) {
      case "speed":
        const speedInput = await vscode.window.showInputBox({
          prompt: "Enter speed (0-100)",
          validateInput: (value) => {
            const num = parseInt(value);
            return isNaN(num) || num < 0 || num > 100
              ? "Enter a number 0-100"
              : null;
          },
        });
        if (speedInput) {
          success = this.sendCommand({
            type: "set_speed",
            robot_id: robotId,
            speed: parseInt(speedInput),
          });
          message = `Speed set to ${speedInput}%`;
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
        success = this.sendCommand({ type: "step_forward", robot_id: robotId });
        message = "Moving forward";
        break;

      case "backward":
        success = this.sendCommand({
          type: "step_backward",
          robot_id: robotId,
        });
        message = "Moving backward";
        break;
    }

    if (success) {
      vscode.window.showInformationMessage(`✓ ${message}`);
    } else {
      vscode.window.showErrorMessage(`✗ Failed to ${action} robot`);
    }
  }

  dispose() {
    this.clearReconnectTimer();
    if (this.ws) this.ws.close();
    if (this.statusBarItem) this.statusBarItem.dispose();
  }
}

class NovaSidebarProvider {
  constructor(controller) {
    this.controller = controller;
    this._view = null;
  }

  resolveWebviewView(webviewView) {
    this._view = webviewView;
    webviewView.webview.options = { enableScripts: true };

    // Handle messages from webview
    webviewView.webview.onDidReceiveMessage(async (message) => {
      const { command, robotId, value } = message;

      switch (command) {
        case "pause":
          await this.controller.executeAction(robotId, "pause");
          break;
        case "resume":
          await this.controller.executeAction(robotId, "resume");
          break;
        case "setSpeed":
          this.controller.sendCommand({
            type: "set_speed",
            robot_id: robotId,
            speed: value,
          });
          break;
        case "refresh":
          this.controller.sendCommand({ type: "get_robots" });
          break;
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

    this._view.webview.html = `<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body {
            font-family: var(--vscode-font-family);
            padding: 10px;
            color: var(--vscode-foreground);
        }
        .status {
            margin-bottom: 15px;
            padding: 10px;
            border-radius: 4px;
            background: var(--vscode-textBlockQuote-background);
        }
        .robot {
            margin-bottom: 10px;
            padding: 10px;
            border: 1px solid var(--vscode-panel-border);
            border-radius: 4px;
        }
        .robot-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 8px;
        }
        .robot-name {
            font-weight: bold;
        }
        .robot-state {
            font-size: 12px;
            padding: 2px 6px;
            border-radius: 3px;
            background: var(--vscode-badge-background);
            color: var(--vscode-badge-foreground);
        }
        .controls {
            display: flex;
            gap: 5px;
            margin-bottom: 8px;
        }
        button {
            background: var(--vscode-button-background);
            color: var(--vscode-button-foreground);
            border: none;
            padding: 6px 12px;
            border-radius: 3px;
            cursor: pointer;
            font-size: 12px;
        }
        button:hover {
            background: var(--vscode-button-hoverBackground);
        }
        button:disabled {
            opacity: 0.5;
            cursor: not-allowed;
        }
        .speed-control {
            display: flex;
            align-items: center;
            gap: 8px;
        }
        input[type="range"] {
            flex: 1;
        }
        .speed-value {
            min-width: 35px;
            font-size: 12px;
        }
        .no-robots {
            text-align: center;
            color: var(--vscode-descriptionForeground);
            margin: 20px 0;
        }
        .refresh-btn {
            background: var(--vscode-button-secondaryBackground);
            color: var(--vscode-button-secondaryForeground);
            width: 100%;
            margin-top: 10px;
        }
    </style>
</head>
<body>
    <div class="status">
        <strong>Connection:</strong> ${
          isConnected ? "🟢 Connected" : "🔴 Disconnected"
        }
        <br>
        <strong>Robots:</strong> ${robots.length}
    </div>

    ${
      robots.length > 0
        ? robots
            .map(
              (robot) => `
        <div class="robot">
            <div class="robot-header">
                <span class="robot-name">${robot.id}</span>
                <span class="robot-state">${robot.state}</span>
            </div>
            
            <div class="controls">
                <button onclick="pauseRobot('${robot.id}')" ${
                !robot.can_pause ? "disabled" : ""
              }>
                    ⏸ Pause
                </button>
                <button onclick="resumeRobot('${robot.id}')" ${
                !robot.can_resume ? "disabled" : ""
              }>
                    ▶ Resume
                </button>
            </div>
            
            <div class="speed-control">
                <span>Speed:</span>
                <input type="range" min="0" max="100" value="${robot.speed}" 
                       onchange="setSpeed('${robot.id}', this.value)">
                <span class="speed-value">${robot.speed}%</span>
            </div>
        </div>
    `
            )
            .join("")
        : `
        <div class="no-robots">
            No robots detected<br>
            <small>Start a Nova program to see robots</small>
        </div>
    `
    }

    <button class="refresh-btn" onclick="refresh()">🔄 Refresh</button>

    <script>
        const vscode = acquireVsCodeApi();
        
        function pauseRobot(robotId) {
            vscode.postMessage({ command: 'pause', robotId });
        }
        
        function resumeRobot(robotId) {
            vscode.postMessage({ command: 'resume', robotId });
        }
        
        function setSpeed(robotId, speed) {
            vscode.postMessage({ command: 'setSpeed', robotId, value: parseInt(speed) });
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
  context.subscriptions.push(
    vscode.commands.registerCommand("nova.showPanel", () =>
      controller.showPanel()
    ),
    vscode.commands.registerCommand("nova.refreshRobots", () =>
      controller.sendCommand({ type: "get_robots" })
    ),
    sidebarRegistration,
    controller
  );

  console.log("[Nova] Extension activated successfully");
  vscode.window.showInformationMessage(
    "Nova extension activated! Check status bar for connection."
  );
}

function deactivate() {}

module.exports = { activate, deactivate };
