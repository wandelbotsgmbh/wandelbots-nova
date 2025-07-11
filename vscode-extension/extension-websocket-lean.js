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
      this.ws = new WebSocket(wsUrl);

      this.ws.on("open", () => {
        this.isConnected = true;
        this.sendCommand({ type: "subscribe_events" });
        this.updateStatusBar();
        this.clearReconnectTimer();
      });

      this.ws.on("message", (data) => {
        const message = JSON.parse(data.toString());
        this.handleMessage(message);
      });

      this.ws.on("close", () => {
        this.isConnected = false;
        this.robots.clear();
        this.updateStatusBar();
        this.scheduleReconnect();
      });

      this.ws.on("error", () => {
        this.isConnected = false;
        this.scheduleReconnect();
      });
    } catch (error) {
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
  }

  updateSingleRobot(robot) {
    if (robot?.id) {
      this.robots.set(robot.id, robot);
      this.updateStatusBar();
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

function activate(context) {
  const controller = new NovaController();

  // Register commands
  context.subscriptions.push(
    vscode.commands.registerCommand("nova.showPanel", () =>
      controller.showPanel()
    ),
    vscode.commands.registerCommand("nova.refreshRobots", () =>
      controller.sendCommand({ type: "get_robots" })
    ),
    controller
  );
}

function deactivate() {}

module.exports = { activate, deactivate };
