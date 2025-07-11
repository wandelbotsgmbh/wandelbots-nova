/**
 * Nova Robot Control VS Code Extension - WebSocket Version
 *
 * This extension uses WebSocket communication to control Nova robots in real-time.
 * Benefits over subprocess approach:
 * - Real-time bidirectional communication
 * - Persistent connection with live updates
 * - Shared state across Nova program and extension
 * - No process separation issues
 */

const vscode = require("vscode");
const WebSocket = require("ws");
const path = require("path");

// Get extension version
const packagePath = path.join(__dirname, "package.json");
const packageInfo = require(packagePath);
const EXTENSION_VERSION = packageInfo.version;
const BUILD_TIMESTAMP = packageInfo.buildTimestamp || new Date().toISOString();

class NovaWebSocketController {
  constructor() {
    this.config = {
      host: "localhost",
      port: 8765,
      autoReconnect: true,
      reconnectInterval: 5000,
    };
    this.ws = null;
    this.isConnected = false;
    this.isNovaAvailable = false;
    this.robots = new Map();
    this.statusBarItem = null;
    this.sidebarProvider = null;
    this.popupPanel = null;
    this.selectedRobotId = null;
    this.reconnectInterval = null;

    console.log("Nova WebSocket Controller initialized");
    this.connect();
  }

  /**
   * Get current VS Code configuration
   */
  getConfiguration() {
    const config = vscode.workspace.getConfiguration("nova.websocket");
    return {
      host: config.get("host", "localhost"),
      port: config.get("port", 8765),
      autoReconnect: config.get("autoReconnect", true),
      reconnectInterval: config.get("reconnectInterval", 3000),
    };
  }

  /**
   * Update configuration (call when settings change)
   */
  updateConfiguration() {
    const oldConfig = this.config;
    this.config = this.getConfiguration();

    // If connection settings changed, reconnect
    if (
      oldConfig.host !== this.config.host ||
      oldConfig.port !== this.config.port
    ) {
      console.log(
        `WebSocket config changed: ${oldConfig.host}:${oldConfig.port} -> ${this.config.host}:${this.config.port}`
      );
      this.reconnect();
    }
  }

  /**
   * Connect to Nova WebSocket server
   */
  connect() {
    try {
      const wsUrl = `ws://${this.config.host}:${this.config.port}`;
      console.log(`Connecting to Nova WebSocket server at ${wsUrl}`);
      this.ws = new WebSocket(wsUrl);

      this.ws.on("open", () => {
        console.log(`Connected to Nova WebSocket server at ${wsUrl}`);
        this.isConnected = true;
        this.isNovaAvailable = true;
        this.clearReconnectInterval();

        // Subscribe to robot events for real-time updates
        // This will also return the current robot list in the response
        this.sendCommand({ type: "subscribe_events" });

        this.updateStatusBar();
      });

      this.ws.on("message", (data) => {
        try {
          const response = JSON.parse(data.toString());
          this.handleMessage(response);
        } catch (error) {
          console.error("Error parsing WebSocket message:", error);
        }
      });

      this.ws.on("close", () => {
        console.log("Nova WebSocket connection closed");
        this.isConnected = false;
        this.isNovaAvailable = false;
        this.robots.clear();
        this.updateStatusBar();
        this.scheduleReconnect();
      });

      this.ws.on("error", (error) => {
        console.log("Nova WebSocket error:", error.message);
        this.isConnected = false;
        this.isNovaAvailable = false;
        this.robots.clear();
        this.updateStatusBar();
        this.scheduleReconnect();
      });
    } catch (error) {
      console.error("Error creating WebSocket connection:", error);
      this.scheduleReconnect();
    }
  }

  /**
   * Schedule reconnection attempt
   */
  scheduleReconnect() {
    if (!this.config.autoReconnect) {
      console.log("Auto-reconnect disabled, not scheduling reconnection");
      return;
    }

    this.clearReconnectInterval();
    this.reconnectInterval = setInterval(() => {
      console.log("Attempting to reconnect to Nova WebSocket...");
      this.connect();
    }, this.config.reconnectInterval);
  }

  /**
   * Manual reconnect method
   */
  reconnect() {
    this.clearReconnectInterval();
    if (this.ws) {
      this.ws.close();
    }
    this.isConnected = false;
    this.isNovaAvailable = false;
    this.robots.clear();
    this.updateStatusBar();

    // Connect with new settings
    this.connect();
  }

  /**
   * Clear reconnection interval
   */
  clearReconnectInterval() {
    if (this.reconnectInterval) {
      clearInterval(this.reconnectInterval);
      this.reconnectInterval = null;
    }
  }

  /**
   * Send command to Nova WebSocket server
   */
  sendCommand(command) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(command));
      return true;
    }
    return false;
  }
  /**
   * Handle incoming WebSocket message
   */
  handleMessage(message) {
    console.log("Received WebSocket message:", message.type, message);

    switch (message.type) {
      case "robot_update":
      case "robot_list":
        // Full robot list update
        this.updateRobotList(message.robots || []);
        break;

      case "robot_state_changed":
        // Single robot state change - prioritize full robot data for consistency
        if (message.robot) {
          // Full robot data provided - use this for complete update
          this.updateSingleRobot(message.robot);
        } else if (message.robot_id && message.state) {
          // Minimal state update - fallback
          this.updateRobotState(message.robot_id, message.state);
        }
        break;

      case "robot_added":
        // New robot detected - add it to the robot list
        if (message.robot) {
          this.addRobot(message.robot);
        } else {
          // Fallback: refresh the robot list to ensure consistency
          console.log("Robot added without full data, requesting refresh");
          this.refreshRobots();
        }
        break;

      case "robot_removed":
        // Robot disconnected
        this.removeRobot(message.robot_id);
        break;

      case "robot_speed_changed":
        // Robot speed updated - prioritize full robot data for consistency
        if (message.robot) {
          this.updateSingleRobot(message.robot);
        } else if (message.robot_id && message.speed !== undefined) {
          this.updateRobotSpeed(message.robot_id, message.speed);
        }
        break;

      default:
        // Legacy support for old message format
        if (message.success && message.robots) {
          this.updateRobotList(message.robots);
        } else if (message.success === false) {
          console.error("WebSocket command failed:", message.error);
        }
    }
  }

  /**
   * Update the complete robot list - ensure proper state handling
   */
  updateRobotList(robots) {
    console.log("Updating robot list with", robots.length, "robots");
    this.robots.clear();
    robots.forEach((robot) => {
      // Trust the server-provided pause/resume capabilities
      // They should be computed based on actual robot state
      this.robots.set(robot.id, robot);
    });
    this.updateUI();
  }

  /**
   * Update a single robot's data - ensure proper state handling
   */
  updateSingleRobot(robot) {
    if (robot && robot.id) {
      console.log(
        `Updating robot ${robot.id}: state=${robot.state}, speed=${robot.speed}, can_pause=${robot.can_pause}, can_resume=${robot.can_resume}`
      );

      // Trust the server-provided data completely
      this.robots.set(robot.id, robot);
      this.updateUI();
    }
  }

  /**
   * Add a new robot
   */
  addRobot(robot) {
    if (robot && robot.id) {
      console.log(
        `Adding new robot: ${robot.id} (state: ${robot.state}, can_pause: ${robot.can_pause}, can_resume: ${robot.can_resume})`
      );
      this.robots.set(robot.id, robot);
      this.updateUI();
      vscode.window.showInformationMessage(`🤖 Robot ${robot.id} connected`);
    }
  }

  /**
   * Remove a robot
   */
  removeRobot(robotId) {
    if (robotId && this.robots.has(robotId)) {
      console.log(`Removing robot: ${robotId}`);
      this.robots.delete(robotId);
      this.updateUI();
      vscode.window.showInformationMessage(`🤖 Robot ${robotId} disconnected`);
    }
  }

  /**
   * Refresh robots by requesting updated list - should rarely be needed with real-time updates
   */
  refreshRobots() {
    console.log(
      "Manual refresh requested - this should be rare with real-time updates"
    );
    this.sendCommand({ type: "get_robots" });
  }

  /**
   * Update robot state - only used as fallback when full robot data isn't available
   */
  updateRobotState(robotId, state) {
    const robot = this.robots.get(robotId);
    if (robot) {
      console.log(`Updating robot ${robotId} state: ${robot.state} → ${state}`);
      robot.state = state;

      // Request full robot data from server to ensure consistency
      // This prevents discrepancies between client-side assumptions and server state
      console.log(
        `Requesting full robot data for ${robotId} to ensure consistency`
      );
      this.sendCommand({ type: "get_status", robot_id: robotId });

      this.robots.set(robotId, robot);
      this.updateUI();
    }
  }

  /**
   * Update robot speed
   */
  updateRobotSpeed(robotId, speed) {
    const robot = this.robots.get(robotId);
    if (robot) {
      console.log(`Updating robot ${robotId} speed: ${speed}%`);
      robot.speed = speed;
      this.robots.set(robotId, robot);
      this.updateUI();
    }
  }

  /**
   * Get robot state icon
   */
  getRobotStateIcon(state) {
    switch (state) {
      case "executing":
        return "$(play)";
      case "paused":
        return "$(debug-pause)";
      case "idle":
        return "$(robot)";
      default:
        return "$(question)";
    }
  }

  /**
   * Get robot state color
   */
  getRobotStateColor(state) {
    switch (state) {
      case "executing":
        return "#22c55e"; // Green
      case "paused":
        return "#f59e0b"; // Orange
      case "idle":
        return "#6b7280"; // Gray
      default:
        return "#dc2626"; // Red
    }
  }

  /**
   * Update all UI components
   */
  updateUI() {
    this.updateStatusBar();
    this.updateSidebar();

    // Update popup if it's open
    if (this.popupPanel) {
      this.updatePopupContent();
    }
  }

  /**
   * Update sidebar content
   */
  updateSidebar() {
    if (this.sidebarProvider) {
      this.sidebarProvider.updateContent();
    }
  }

  /**
   * Get robots via WebSocket
   */
  async getRobots() {
    return new Promise((resolve) => {
      if (!this.isConnected) {
        resolve([]);
        return;
      }

      // Return current robot list (updated via WebSocket messages)
      resolve(Array.from(this.robots.values()));
    });
  }

  /**
   * Set robot speed via WebSocket
   */
  async setRobotSpeed(robotId, speed) {
    if (!this.isConnected) {
      return false;
    }

    return this.sendCommand({
      type: "set_speed",
      robot_id: robotId,
      speed: speed,
    });
  }

  /**
   * Pause robot via WebSocket
   */
  async pauseRobot(robotId) {
    if (!this.isConnected) {
      return false;
    }

    // Send command without optimistic updates - let WebSocket events handle UI updates
    return this.sendCommand({
      type: "pause",
      robot_id: robotId,
    });
  }

  /**
   * Resume robot via WebSocket
   */
  async resumeRobot(robotId) {
    if (!this.isConnected) {
      return false;
    }

    // Send command without optimistic updates - let WebSocket events handle UI updates
    return this.sendCommand({
      type: "resume",
      robot_id: robotId,
    });
  }

  /**
   * Move forward via WebSocket
   */
  async stepForward(robotId) {
    if (!this.isConnected) {
      return false;
    }

    return this.sendCommand({
      type: "step_forward",
      robot_id: robotId,
    });
  }

  /**
   * Move backward via WebSocket
   */
  async stepBackward(robotId) {
    if (!this.isConnected) {
      return false;
    }

    return this.sendCommand({
      type: "step_backward",
      robot_id: robotId,
    });
  }

  /**
   * Update status bar
   */
  updateStatusBar() {
    if (!this.statusBarItem) {
      this.statusBarItem = vscode.window.createStatusBarItem(
        vscode.StatusBarAlignment.Left,
        100
      );
    }

    const robotCount = this.robots.size;
    const activeRobots = Array.from(this.robots.values()).filter(
      (robot) => robot.state === "executing"
    ).length;

    if (!this.isNovaAvailable) {
      this.statusBarItem.text = "$(robot) Nova: Not connected";
      this.statusBarItem.tooltip = "Nova WebSocket server not available";
      this.statusBarItem.command = "nova.showHelp";
      this.statusBarItem.backgroundColor = new vscode.ThemeColor(
        "statusBarItem.warningBackground"
      );
    } else if (robotCount === 0) {
      this.statusBarItem.text = "$(robot) Nova: No robots";
      this.statusBarItem.tooltip = "Nova connected but no robots running";
      this.statusBarItem.command = "nova.focus";
      this.statusBarItem.backgroundColor = undefined;
    } else {
      this.statusBarItem.text = `$(robot) Nova: ${activeRobots}/${robotCount}`;
      this.statusBarItem.tooltip = `Nova robots: ${activeRobots} active, ${robotCount} total. Click to control.`;
      this.statusBarItem.command = "nova.focus";
      this.statusBarItem.backgroundColor = new vscode.ThemeColor(
        "statusBarItem.prominentBackground"
      );
    }

    this.statusBarItem.show();
  }

  /**
   * Show main control panel using Quick Pick
   */
  async showPanel() {
    const robots = Array.from(this.robots.values());
    const items = [];

    // Header information
    items.push({
      label: "$(info) Nova Robot Control (WebSocket)",
      description: this.isNovaAvailable
        ? `${robots.length} robots detected`
        : "Nova not connected",
      detail: "Select an action below",
    });

    // Separator
    items.push({
      label: "",
      kind: vscode.QuickPickItemKind.Separator,
    });

    if (!this.isNovaAvailable) {
      items.push({
        label: "$(warning) Nova Not Connected",
        description: "WebSocket connection failed",
        action: "show_help",
      });
      items.push({
        label: "$(refresh) Retry Connection",
        description: "Attempt to reconnect to Nova",
        action: "reconnect",
      });
    } else if (robots.length === 0) {
      items.push({
        label: "$(search) No Robots Found",
        description: "Start a Nova program to see robots",
        action: "refresh",
      });
      items.push({
        label: "$(refresh) Refresh",
        description: "Request robot list update",
        action: "refresh",
      });
    } else {
      // Show robot controls
      robots.forEach((robot) => {
        const icon = this.getRobotStateIcon(robot.state);
        const stateColor = this.getRobotStateColor(robot.state);

        items.push({
          label: `${icon} ${robot.id}`,
          description: `${robot.speed}% speed`,
          detail: `State: ${robot.state}`,
          action: "control_robot",
          robot: robot,
        });
      });

      // Separator
      items.push({
        label: "",
        kind: vscode.QuickPickItemKind.Separator,
      });

      // Global actions
      items.push({
        label: "$(refresh) Refresh Robot List",
        description: "Update robot status",
        action: "refresh",
      });
    }

    const selected = await vscode.window.showQuickPick(items, {
      placeHolder: "Nova Robot Control - Select an action",
      matchOnDescription: true,
      matchOnDetail: true,
    });

    if (selected) {
      await this.handlePanelAction(selected);
    }
  }

  /**
   * Handle panel action selection
   */
  async handlePanelAction(selected) {
    switch (selected.action) {
      case "refresh":
        this.sendCommand({ type: "get_robots" });
        vscode.window.showInformationMessage("Robot list refreshed");
        break;

      case "reconnect":
        this.connect();
        vscode.window.showInformationMessage("Reconnecting to Nova...");
        break;

      case "control_robot":
        if (selected.robot) {
          await this.showRobotControls(selected.robot);
        }
        break;

      case "show_help":
        await this.showHelp();
        break;
    }
  }

  /**
   * Show controls for specific robot
   */
  async showRobotControls(robot) {
    const items = [];

    // Robot info header
    items.push({
      label: `$(robot) ${robot.id}`,
      description: `${robot.speed}% speed - ${robot.state}`,
      detail: "Select a control action",
    });

    // Separator
    items.push({
      label: "",
      kind: vscode.QuickPickItemKind.Separator,
    });

    // Speed control
    items.push({
      label: "$(gauge) Change Speed",
      description: `Current: ${robot.speed}%`,
      detail: "Set new speed percentage (0-100)",
      action: "change_speed",
    });

    // Pause/Resume
    if (robot.can_pause) {
      items.push({
        label: "$(debug-pause) Pause Robot",
        description: "Pause robot execution",
        action: "pause",
      });
    }

    if (robot.can_resume) {
      items.push({
        label: "$(debug-start) Resume Robot",
        description: "Resume robot execution",
        action: "resume",
      });
    }

    // Move Forward/Backward
    items.push({
      label: "$(arrow-right) Move Forward",
      description: "Continue moving robot forward",
      action: "step_forward",
    });

    items.push({
      label: "$(arrow-left) Move Backward",
      description: "Continue moving robot backward",
      action: "step_backward",
    });

    // Separator
    items.push({
      label: "",
      kind: vscode.QuickPickItemKind.Separator,
    });

    items.push({
      label: "$(arrow-left) Back to Robot List",
      description: "Return to main panel",
      action: "back",
    });

    const selected = await vscode.window.showQuickPick(items, {
      placeHolder: `Control ${robot.id}`,
      matchOnDescription: true,
    });

    if (selected) {
      await this.handleRobotAction(robot, selected);
    }
  }

  /**
   * Handle robot control action
   */
  async handleRobotAction(robot, selected) {
    switch (selected.action) {
      case "change_speed":
        await this.changeRobotSpeed(robot);
        break;

      case "pause":
        const pauseSuccess = await this.pauseRobot(robot.id);
        if (pauseSuccess) {
          vscode.window.showInformationMessage(`✓ Paused ${robot.id}`);
        } else {
          vscode.window.showErrorMessage(`✗ Failed to pause ${robot.id}`);
        }
        break;

      case "resume":
        const resumeSuccess = await this.resumeRobot(robot.id);
        if (resumeSuccess) {
          vscode.window.showInformationMessage(`✓ Resumed ${robot.id}`);
        } else {
          vscode.window.showErrorMessage(`✗ Failed to resume ${robot.id}`);
        }
        break;

      case "step_forward":
        const stepForwardSuccess = await this.stepForward(robot.id);
        if (stepForwardSuccess) {
          vscode.window.showInformationMessage(`⏩ Moving forward ${robot.id}`);
        } else {
          vscode.window.showErrorMessage(
            `✗ Failed to move forward ${robot.id}`
          );
        }
        break;

      case "step_backward":
        const stepBackwardSuccess = await this.stepBackward(robot.id);
        if (stepBackwardSuccess) {
          vscode.window.showInformationMessage(
            `⏪ Moving backward ${robot.id}`
          );
        } else {
          vscode.window.showErrorMessage(
            `✗ Failed to move backward ${robot.id}`
          );
        }
        break;

      case "back":
        await this.showPanel();
        break;
    }
  }

  /**
   * Change robot speed using input box
   */
  async changeRobotSpeed(robot) {
    const speedInput = await vscode.window.showInputBox({
      prompt: `Enter new speed for ${robot.id} (0-100)`,
      placeHolder: robot.speed.toString(),
      validateInput: (value) => {
        const num = parseInt(value);
        if (isNaN(num)) {
          return "Please enter a valid number";
        }
        if (num < 0 || num > 100) {
          return "Speed must be between 0 and 100";
        }
        return null;
      },
    });

    if (speedInput !== undefined) {
      const newSpeed = parseInt(speedInput);
      const success = await this.setRobotSpeed(robot.id, newSpeed);

      if (success) {
        vscode.window.showInformationMessage(
          `✓ Set ${robot.id} speed to ${newSpeed}%`
        );
        // Robot data will be updated via WebSocket message
      } else {
        vscode.window.showErrorMessage(`✗ Failed to set speed for ${robot.id}`);
      }
    }
  }

  /**
   * Show help information
   */
  async showHelp() {
    const helpMessage = `Nova Robot Control Extension (WebSocket)

✅ Real-time robot control via WebSocket connection
✅ Automatic state synchronization - no manual refresh needed
✅ Reliable pause/resume functionality
✅ Configurable connection settings

Connection Status: ${this.isNovaAvailable ? "Connected" : "Disconnected"}
WebSocket URL: ws://${this.config.host}:${this.config.port}
Auto-reconnect: ${this.config.autoReconnect ? "Enabled" : "Disabled"}
Reconnect interval: ${this.config.reconnectInterval}ms

Key Features:
• Real-time state updates via WebSocket events
• Single source of truth for robot state
• Automatic UI updates when robot state changes
• No need for manual refresh in normal operation

Configuration:
• Go to VS Code Settings → Extensions → Nova Robot Control
• Change host/port to connect to remote Nova instances
• Configure auto-reconnect behavior

Requirements:
• Nova Python library running with WebSocket server
• Nova program actively executing robots

Troubleshooting:
• Make sure Nova program is running on configured host
• Check that WebSocket server started on configured port
• Verify firewall settings for remote connections
• Try "Retry Connection" if disconnected
• Use manual refresh only if real-time updates aren't working

Current Status:
• Connected: ${this.isConnected}
• Robots: ${this.robots.size}`;

    vscode.window.showInformationMessage(helpMessage, { modal: true });
  }

  /**
   * Dispose resources
   */
  dispose() {
    this.clearReconnectInterval();
    if (this.ws) {
      // Unsubscribe from events before closing
      this.sendCommand({ type: "unsubscribe_events" });
      this.ws.close();
    }
    if (this.statusBarItem) {
      this.statusBarItem.dispose();
    }
    if (this.popupPanel) {
      this.popupPanel.dispose();
    }
  }

  /**
   * Show robot control popup (like Copilot/notifications)
   */
  async showControlPanel() {
    const robots = Array.from(this.robots.values());

    if (!this.isNovaAvailable) {
      await this.showHelp();
      return;
    }

    if (robots.length === 0) {
      vscode.window.showInformationMessage(
        "No robots found. Start a Nova program to see robots."
      );
      return;
    }

    // Create popup webview panel
    if (this.popupPanel) {
      this.popupPanel.dispose();
    }

    this.popupPanel = vscode.window.createWebviewPanel(
      "novaRobotPopup",
      "Nova Robot Control",
      { viewColumn: vscode.ViewColumn.One, preserveFocus: true },
      {
        enableScripts: true,
        retainContextWhenHidden: false,
        localResourceRoots: [],
      }
    );

    // Set initial robot if none selected
    if (!this.selectedRobotId && robots.length > 0) {
      this.selectedRobotId = robots[0].id;
    }

    // Handle popup disposal
    this.popupPanel.onDidDispose(() => {
      this.popupPanel = null;
    });

    // Handle messages from popup
    this.popupPanel.webview.onDidReceiveMessage(async (message) => {
      await this.handlePopupMessage(message);
    });

    // Update popup content
    this.updatePopupContent();

    // Show as small popup
    this.popupPanel.reveal(vscode.ViewColumn.One, true);
  }

  /**
   * Handle messages from popup webview
   */
  async handlePopupMessage(message) {
    const { command, robotId, value } = message;

    switch (command) {
      case "selectRobot":
        this.selectedRobotId = robotId;
        this.updatePopupContent();
        break;

      case "pause":
        const pauseSuccess = await this.pauseRobot(robotId);
        if (pauseSuccess) {
          vscode.window.showInformationMessage(`⏸️ Paused ${robotId}`);
          // Update UI immediately
          this.updateUI();
        } else {
          vscode.window.showErrorMessage(`Failed to pause ${robotId}`);
        }
        break;

      case "resume":
        const resumeSuccess = await this.resumeRobot(robotId);
        if (resumeSuccess) {
          vscode.window.showInformationMessage(`▶️ Resumed ${robotId}`);
          // Update UI immediately
          this.updateUI();
        } else {
          vscode.window.showErrorMessage(`Failed to resume ${robotId}`);
        }
        break;

      case "setSpeed":
        const speedSuccess = await this.setRobotSpeed(robotId, value);
        if (speedSuccess) {
          vscode.window.showInformationMessage(
            `⚡ Set ${robotId} speed to ${value}%`
          );
        } else {
          vscode.window.showErrorMessage(`Failed to set speed for ${robotId}`);
        }
        break;

      case "stepForward":
        const stepForwardSuccess = await this.stepForward(robotId);
        if (stepForwardSuccess) {
          vscode.window.showInformationMessage(`⏭️ Step forward ${robotId}`);
        } else {
          vscode.window.showErrorMessage(`Failed to step forward ${robotId}`);
        }
        break;

      case "stepBackward":
        const stepBackwardSuccess = await this.stepBackward(robotId);
        if (stepBackwardSuccess) {
          vscode.window.showInformationMessage(`⏮️ Step backward ${robotId}`);
        } else {
          vscode.window.showErrorMessage(`Failed to step backward ${robotId}`);
        }
        break;

      case "refresh":
        this.sendCommand({ type: "get_robots" });
        break;

      case "close":
        if (this.popupPanel) {
          this.popupPanel.dispose();
        }
        break;
    }
  }

  /**
   * Update popup content
   */
  updatePopupContent() {
    if (!this.popupPanel) return;

    const robots = Array.from(this.robots.values());
    const selectedRobot =
      robots.find((r) => r.id === this.selectedRobotId) || robots[0];

    if (selectedRobot) {
      this.selectedRobotId = selectedRobot.id;
    }

    const html = this.getPopupHTML(robots, selectedRobot);
    this.popupPanel.webview.html = html;
  }

  /**
   * Generate compact popup HTML
   */
  getPopupHTML(robots, selectedRobot) {
    const robotOptions = robots
      .map(
        (robot) =>
          `<option value="${robot.id}" ${
            robot.id === selectedRobot?.id ? "selected" : ""
          }>
        ${robot.id}
      </option>`
      )
      .join("");

    const isExecuting = selectedRobot?.state === "executing";
    const isPaused = selectedRobot?.state === "paused";
    const isIdle = selectedRobot?.state === "idle";
    const canPause = selectedRobot?.can_pause === true;
    const canResume = selectedRobot?.can_resume === true;

    return `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Nova Robot Control</title>
    <style>
        body {
            font-family: var(--vscode-font-family);
            font-size: 13px;
            background: var(--vscode-dropdown-background);
            color: var(--vscode-dropdown-foreground);
            margin: 0;
            padding: 12px;
            border: 1px solid var(--vscode-dropdown-border);
            border-radius: 6px;
            box-shadow: 0 4px 12px rgba(0,0,0,0.15);
            min-width: 280px;
            max-width: 320px;
        }
        
        .header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 12px;
            padding-bottom: 8px;
            border-bottom: 1px solid var(--vscode-dropdown-border);
        }
        
        .title {
            font-weight: 600;
            color: var(--vscode-foreground);
        }
        
        .version-info {
            font-size: 10px;
            color: var(--vscode-descriptionForeground);
            font-weight: 400;
        }
        
        .close-btn {
            background: none;
            border: none;
            color: var(--vscode-foreground);
            cursor: pointer;
            padding: 2px 4px;
            border-radius: 3px;
            font-size: 16px;
        }
        
        .close-btn:hover {
            background: var(--vscode-toolbar-hoverBackground);
        }
        
        .robot-section {
            margin-bottom: 12px;
        }
        
        .robot-select {
            width: 100%;
            background: var(--vscode-input-background);
            color: var(--vscode-input-foreground);
            border: 1px solid var(--vscode-input-border);
            border-radius: 3px;
            padding: 6px 8px;
            font-size: 13px;
            margin-bottom: 8px;
        }
        
        .robot-status {
            display: flex;
            align-items: center;
            gap: 8px;
            font-size: 12px;
            color: var(--vscode-descriptionForeground);
        }
        
        .status-dot {
            width: 8px;
            height: 8px;
            border-radius: 50%;
        }
        
        .status-executing { background: #22c55e; }
        .status-paused { background: #f59e0b; }
        .status-other { background: #6b7280; }
        
        .controls-section {
            margin-bottom: 12px;
        }
        
        .section-label {
            font-size: 11px;
            font-weight: 600;
            color: var(--vscode-descriptionForeground);
            margin-bottom: 6px;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        
        .play-pause-row {
            display: flex;
            gap: 8px;
            margin-bottom: 12px;
        }
        
        .control-btn {
            flex: 1;
            background: var(--vscode-button-background);
            color: var(--vscode-button-foreground);
            border: none;
            border-radius: 4px;
            padding: 8px 12px;
            cursor: pointer;
            font-size: 13px;
            font-weight: 500;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 6px;
            transition: background 0.15s;
        }
        
        .control-btn:hover:not(:disabled) {
            background: var(--vscode-button-hoverBackground);
        }
        
        .control-btn:disabled {
            background: var(--vscode-button-secondaryBackground);
            color: var(--vscode-button-secondaryForeground);
            cursor: not-allowed;
            opacity: 0.6;
        }
        
        .resume-btn {
            background: #059669;
        }
        
        .resume-btn:hover:not(:disabled) {
            background: #047857;
        }
        
        .pause-btn {
            background: #dc2626;
        }
        
        .pause-btn:hover:not(:disabled) {
            background: #b91c1c;
        }
        
        .step-controls {
            display: flex;
            gap: 8px;
            margin-top: 8px;
        }
        
        .step-btn {
            background: var(--vscode-button-secondaryBackground);
            color: var(--vscode-button-secondaryForeground);
        }
        
        .step-btn:hover:not(:disabled) {
            background: var(--vscode-button-hoverBackground);
        }
        
        .speed-section {
            margin-bottom: 12px;
        }
        
        .speed-row {
            display: flex;
            align-items: center;
            gap: 8px;
            margin-bottom: 6px;
        }
        
        .speed-slider {
            flex: 1;
            height: 4px;
            background: var(--vscode-progressBar-background);
            border-radius: 2px;
            outline: none;
            -webkit-appearance: none;
        }
        
        .speed-slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 16px;
            height: 16px;
            background: var(--vscode-progressBar-background);
            border-radius: 50%;
            cursor: pointer;
            border: 2px solid var(--vscode-button-background);
        }
        
        .speed-slider::-moz-range-thumb {
            width: 16px;
            height: 16px;
            background: var(--vscode-button-background);
            border-radius: 50%;
            cursor: pointer;
            border: none;
        }
        
        .speed-value {
            font-weight: 600;
            color: var(--vscode-textLink-foreground);
            min-width: 35px;
            text-align: right;
        }
        
        .speed-labels {
            display: flex;
            justify-content: space-between;
            font-size: 10px;
            color: var(--vscode-descriptionForeground);
            margin-top: 2px;
        }
        
        .footer {
            display: flex;
            justify-content: space-between;
            margin-top: 12px;
            padding-top: 8px;
            border-top: 1px solid var(--vscode-dropdown-border);
        }
        
        .refresh-btn {
            background: none;
            border: 1px solid var(--vscode-button-border);
            color: var(--vscode-button-foreground);
            border-radius: 3px;
            padding: 4px 8px;
            cursor: pointer;
            font-size: 11px;
        }
        
        .refresh-btn:hover {
            background: var(--vscode-button-hoverBackground);
        }
        
        .connection-status {
            font-size: 10px;
            color: var(--vscode-descriptionForeground);
            display: flex;
            align-items: center;
            gap: 4px;
        }
    </style>
</head>
<body>
    <div class="header">
        <div class="title">🤖 Nova Robot Control</div>
        <div class="version-info">v${EXTENSION_VERSION}</div>
        <button class="close-btn" onclick="closePopup()">×</button>
    </div>

    ${
      robots.length > 1
        ? `
    <div class="robot-section">
        <select class="robot-select" id="robotSelect">
            ${robotOptions}
        </select>
    </div>
    `
        : ""
    }

    ${
      selectedRobot
        ? `
    <div class="robot-section">
        <div class="robot-status">
            <span class="status-dot status-${selectedRobot.state.toLowerCase()}"></span>
            <strong>${selectedRobot.id}</strong>
            <span>•</span>
            <span>${selectedRobot.state}</span>
        </div>
    </div>

    <div class="controls-section">
        <div class="section-label">Control</div>
        <div class="play-pause-row">
            ${
              isPaused || !isExecuting
                ? `<button class="control-btn resume-btn" onclick="resumeRobot()" ${
                    !canResume ? "disabled" : ""
                  }>
                    <span>▶</span> Resume
                </button>`
                : `<button class="control-btn pause-btn" onclick="pauseRobot()" ${
                    !canPause ? "disabled" : ""
                  }>
                    <span>⏸</span> Pause
                </button>`
            }
        </div>
        ${
          isPaused
            ? `<div class="step-controls">
                <button class="control-btn step-btn" onclick="stepBackward()" ${
                  !canResume ? "disabled" : ""
                }>
                    <span>⏮</span> Step Back
                </button>
                <button class="control-btn step-btn" onclick="stepForward()" ${
                  !canResume ? "disabled" : ""
                }>
                    <span>⏭</span> Step Forward
                </button>
            </div>`
            : ""
        }
    </div>

    <div class="speed-section">
        <div class="section-label">Speed</div>
        <div class="speed-row">
            <input type="range" class="speed-slider" id="speedSlider" 
                   min="0" max="100" value="${selectedRobot.speed}"
                   oninput="updateSpeedDisplay(this.value)" 
                   onchange="setSpeed(this.value)">
            <span class="speed-value" id="speedValue">${
              selectedRobot.speed
            }%</span>
        </div>
        <div class="speed-labels">
            <span>0%</span>
            <span>100%</span>
        </div>
    </div>
    `
        : `
    <div class="robot-section">
        <div style="text-align: center; color: var(--vscode-descriptionForeground); padding: 20px;">
            No robot selected
        </div>
    </div>
    `
    }

    <div class="footer">
        <button class="refresh-btn" onclick="refreshRobots()">🔄 Refresh</button>
        <div class="connection-status">
            <span style="color: ${
              this.isConnected ? "#22c55e" : "#dc2626"
            };">●</span>
            ${this.config.host}:${this.config.port}
        </div>
    </div>

    <script>
        const vscode = acquireVsCodeApi();
        
        function closePopup() {
            vscode.postMessage({ command: 'close' });
        }
        
        function refreshRobots() {
            vscode.postMessage({ command: 'refresh' });
        }
        
        function pauseRobot() {
            const robotId = getRobotId();
            vscode.postMessage({
                command: 'pause',
                robotId: robotId
            });
        }
        
        function resumeRobot() {
            const robotId = getRobotId();
            vscode.postMessage({
                command: 'resume',
                robotId: robotId
            });
        }
        
        function stepForward() {
            const robotId = getRobotId();
            vscode.postMessage({
                command: 'stepForward',
                robotId: robotId
            });
        }
        
        function stepBackward() {
            const robotId = getRobotId();
            vscode.postMessage({
                command: 'stepBackward',
                robotId: robotId
            });
        }
        
        function setSpeed(speed) {
            const robotId = getRobotId();
            vscode.postMessage({
                command: 'setSpeed',
                robotId: robotId,
                value: parseInt(speed)
            });
        }
        
        function updateSpeedDisplay(speed) {
            document.getElementById('speedValue').textContent = speed + '%';
        }
        
        function getRobotId() {
            const select = document.getElementById('robotSelect');
            return select ? select.value : '${selectedRobot?.id || ""}';
        }
        
        // Handle robot selection change
        const robotSelect = document.getElementById('robotSelect');
        if (robotSelect) {
            robotSelect.addEventListener('change', function(e) {
                vscode.postMessage({
                    command: 'selectRobot',
                    robotId: e.target.value
                });
            });
        }
    </script>
</body>
</html>`;
  }
}

/**
 * Nova Robot Control Sidebar Provider
 * Provides a webview in the sidebar for robot control
 */
class NovaRobotControlProvider {
  constructor(controller) {
    this.controller = controller;
    this._view = null;
  }

  resolveWebviewView(webviewView, context, token) {
    this._view = webviewView;

    webviewView.webview.options = {
      enableScripts: true,
      localResourceRoots: [],
    };

    // Handle messages from the webview
    webviewView.webview.onDidReceiveMessage(async (message) => {
      await this.handleSidebarMessage(message);
    });

    // Update content when robots change
    this.updateContent();

    // Refresh content periodically (optional)
    setInterval(() => {
      if (this._view?.visible) {
        this.updateContent();
      }
    }, 5000);
  }

  /**
   * Handle messages from sidebar webview
   */
  async handleSidebarMessage(message) {
    const { command, robotId, value } = message;

    switch (command) {
      case "selectRobot":
        this.controller.selectedRobotId = robotId;
        this.updateContent();
        break;

      case "pause":
        const pauseSuccess = await this.controller.pauseRobot(robotId);
        if (pauseSuccess) {
          vscode.window.showInformationMessage(`⏸️ Paused ${robotId}`);
          // Update UI immediately
          this.updateContent();
        } else {
          vscode.window.showErrorMessage(`Failed to pause ${robotId}`);
        }
        break;

      case "resume":
        const resumeSuccess = await this.controller.resumeRobot(robotId);
        if (resumeSuccess) {
          vscode.window.showInformationMessage(`▶️ Resumed ${robotId}`);
          // Update UI immediately
          this.updateContent();
        } else {
          vscode.window.showErrorMessage(`Failed to resume ${robotId}`);
        }
        break;

      case "setSpeed":
        const speedSuccess = await this.controller.setRobotSpeed(
          robotId,
          value
        );
        if (speedSuccess) {
          vscode.window.showInformationMessage(
            `⚡ Set ${robotId} speed to ${value}%`
          );
        } else {
          vscode.window.showErrorMessage(`Failed to set speed for ${robotId}`);
        }
        break;

      case "refresh":
        this.controller.sendCommand({ type: "get_robots" });
        break;
    }
  }

  /**
   * Update sidebar content
   */
  updateContent() {
    if (!this._view) return;

    const robots = Array.from(this.controller.robots.values());
    const selectedRobot =
      robots.find((r) => r.id === this.controller.selectedRobotId) || robots[0];

    if (selectedRobot) {
      this.controller.selectedRobotId = selectedRobot.id;
    }

    this._view.webview.html = this.getSidebarHTML(robots, selectedRobot);
  }

  /**
   * Generate sidebar HTML
   */
  getSidebarHTML(robots, selectedRobot) {
    const robotOptions = robots
      .map(
        (robot) => `<option value="${robot.id}" ${
          robot.id === selectedRobot?.id ? "selected" : ""
        }>
        ${robot.id}
      </option>`
      )
      .join("");

    const isExecuting = selectedRobot?.state === "executing";
    const isPaused = selectedRobot?.state === "paused";
    const isIdle = selectedRobot?.state === "idle";
    const canPause = selectedRobot?.can_pause === true;
    const canResume = selectedRobot?.can_resume === true;

    return `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Nova Robot Control</title>
    <style>
        body {
            font-family: var(--vscode-font-family);
            font-size: var(--vscode-font-size);
            background: var(--vscode-editor-background);
            color: var(--vscode-editor-foreground);
            margin: 0;
            padding: 16px;
            line-height: 1.4;
        }
        
        .header {
            margin-bottom: 20px;
            padding-bottom: 12px;
            border-bottom: 1px solid var(--vscode-panel-border);
        }
        
        .title {
            font-size: 16px;
            font-weight: 600;
            color: var(--vscode-editor-foreground);
            margin: 0 0 8px 0;
        }
        
        .subtitle {
            font-size: 12px;
            color: var(--vscode-descriptionForeground);
            margin: 0;
        }
        
        .version-info {
            font-size: 10px;
            color: var(--vscode-descriptionForeground);
            margin: 4px 0;
            font-style: italic;
        }
        
        .connection-status {
            display: flex;
            align-items: center;
            gap: 8px;
            margin-top: 8px;
            font-size: 12px;
        }
        
        .status-dot {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            flex-shrink: 0;
        }
        
        .status-connected {
            background-color: var(--vscode-testing-iconPassed);
        }
        
        .status-disconnected {
            background-color: var(--vscode-testing-iconFailed);
        }
        
        .robot-section {
            margin-bottom: 24px;
        }
        
        .section-label {
            font-size: 13px;
            font-weight: 500;
            color: var(--vscode-editor-foreground);
            margin-bottom: 8px;
        }
        
        .robot-select {
            width: 100%;
            background: var(--vscode-input-background);
            color: var(--vscode-input-foreground);
            border: 1px solid var(--vscode-input-border);
            border-radius: 2px;
            padding: 8px 12px;
            font-size: 13px;
            margin-bottom: 12px;
        }
        
        .robot-select:focus {
            outline: 1px solid var(--vscode-focusBorder);
            outline-offset: -1px;
        }
        
        .robot-info {
            background: var(--vscode-textBlockQuote-background);
            border-left: 4px solid var(--vscode-textBlockQuote-border);
            padding: 12px;
            margin-bottom: 16px;
            border-radius: 0 3px 3px 0;
        }
        
        .robot-info-item {
            display: flex;
            justify-content: space-between;
            margin-bottom: 4px;
            font-size: 12px;
        }
        
        .robot-info-item:last-child {
            margin-bottom: 0;
        }
        
        .robot-info-label {
            color: var(--vscode-descriptionForeground);
        }
        
        .robot-info-value {
            color: var(--vscode-editor-foreground);
            font-weight: 500;
        }
        
        .controls-section {
            margin-bottom: 24px;
        }
        
        .play-pause-row {
            display: flex;
            gap: 8px;
            margin-bottom: 16px;
        }
        
        .control-btn {
            flex: 1;
            background: var(--vscode-button-background);
            color: var(--vscode-button-foreground);
            border: none;
            border-radius: 2px;
            padding: 10px 16px;
            font-size: 13px;
            font-weight: 500;
            cursor: pointer;
            display: flex;
            align-items: center;
            justify-content: center;
            gap: 6px;
            transition: background-color 0.2s;
        }
        
        .control-btn:hover:not(:disabled) {
            background: var(--vscode-button-hoverBackground);
        }
        
        .control-btn:disabled {
            background: var(--vscode-button-secondaryBackground);
            color: var(--vscode-button-secondaryForeground);
            cursor: not-allowed;
            opacity: 0.6;
        }
        

        
        .pause-btn {
            background: var(--vscode-testing-iconFailed);
        }
        
        .pause-btn:hover:not(:disabled) {
            background: var(--vscode-testing-iconFailed);
            filter: brightness(1.1);
        }
        
        .step-controls {
            display: flex;
            gap: 8px;
            margin-top: 12px;
        }
        
        .step-btn {
            background: var(--vscode-button-secondaryBackground);
            color: var(--vscode-button-secondaryForeground);
        }
        
        .step-btn:hover:not(:disabled) {
            background: var(--vscode-button-secondaryHoverBackground);
        }
        
        .speed-section {
            margin-bottom: 24px;
        }
        
        .speed-row {
            display: flex;
            align-items: center;
            gap: 12px;
            margin-bottom: 8px;
        }
        
        .speed-slider {
            flex: 1;
            appearance: none;
            height: 4px;
            background: var(--vscode-scrollbarSlider-background);
            border-radius: 2px;
            outline: none;
        }
        
        .speed-slider::-webkit-slider-thumb {
            appearance: none;
            width: 16px;
            height: 16px;
            background: var(--vscode-button-background);
            border-radius: 50%;
            cursor: pointer;
            border: 2px solid var(--vscode-editor-background);
            box-shadow: 0 1px 3px rgba(0,0,0,0.3);
        }
        
        .speed-slider::-webkit-slider-thumb:hover {
            background: var(--vscode-button-hoverBackground);
        }
        
        .speed-value {
            font-size: 13px;
            font-weight: 500;
            color: var(--vscode-editor-foreground);
            min-width: 40px;
            text-align: right;
        }
        
        .speed-labels {
            display: flex;
            justify-content: space-between;
            font-size: 11px;
            color: var(--vscode-descriptionForeground);
        }
        
        .refresh-section {
            text-align: center;
        }
        
        .refresh-btn {
            background: var(--vscode-button-secondaryBackground);
            color: var(--vscode-button-secondaryForeground);
            border: 1px solid var(--vscode-button-border);
            border-radius: 2px;
            padding: 8px 16px;
            font-size: 12px;
            cursor: pointer;
            display: inline-flex;
            align-items: center;
            gap: 6px;
        }
        
        .refresh-btn:hover {
            background: var(--vscode-button-secondaryHoverBackground);
        }
        
        .no-robots {
            text-align: center;
            padding: 40px 20px;
            color: var(--vscode-descriptionForeground);
        }
        
        .no-robots-icon {
            font-size: 48px;
            margin-bottom: 16px;
        }
        
        .no-robots-text {
            font-size: 14px;
            margin-bottom: 8px;
        }
        
        .no-robots-subtitle {
            font-size: 12px;
            color: var(--vscode-descriptionForeground);
        }
    </style>
</head>
<body>
    <div class="header">
        <h1 class="title">Nova Robot Control</h1>
        <p class="subtitle">Real-time robot control via WebSocket</p>
        <div class="version-info">v${EXTENSION_VERSION} | ${new Date(
      BUILD_TIMESTAMP
    ).toLocaleString()}</div>
        <div class="connection-status">
            <div class="status-dot ${
              this.controller.isConnected
                ? "status-connected"
                : "status-disconnected"
            }"></div>
            <span>${
              this.controller.isConnected ? "Connected" : "Disconnected"
            } (${this.controller.config.host}:${
      this.controller.config.port
    })</span>
        </div>
    </div>
    
    ${
      robots.length > 0
        ? `
    <div class="robot-section">
        <div class="section-label">Robot Selection</div>
        <select class="robot-select" id="robotSelect">
            ${robotOptions}
        </select>
    </div>

    ${
      selectedRobot
        ? `
    <div class="controls-section">
        <div class="section-label">Control</div>
        <div class="play-pause-row">
            ${
              isPaused || !isExecuting
                ? `
            <button class="control-btn resume-btn" onclick="resumeRobot()" ${
              !canResume ? "disabled" : ""
            }>
                <span>▶</span> Resume
            </button>`
                : `
            <button class="control-btn pause-btn" onclick="pauseRobot()" ${
              !canPause ? "disabled" : ""
            }>
                <span>⏸</span> Pause
            </button>`
            }
        </div>
        ${
          isPaused
            ? `<div class="step-controls">
                <button class="control-btn step-btn" onclick="stepBackward()" ${
                  !canResume ? "disabled" : ""
                }>
                    <span>⏮</span> Step Back
                </button>
                <button class="control-btn step-btn" onclick="stepForward()" ${
                  !canResume ? "disabled" : ""
                }>
                    <span>⏭</span> Step Forward
                </button>
            </div>`
            : ""
        }
    </div>

    <div class="speed-section">
        <div class="section-label">Speed Control</div>
        <div class="speed-row">
            <input type="range" class="speed-slider" id="speedSlider" 
                   min="0" max="100" value="${selectedRobot.speed}"
                   oninput="updateSpeedDisplay(this.value)" 
                   onchange="setSpeed(this.value)">
            <span class="speed-value" id="speedValue">${
              selectedRobot.speed
            }%</span>
        </div>
        <div class="speed-labels">
            <span>0%</span>
            <span>100%</span>
        </div>
    </div>`
        : ""
    }
    `
        : `
    <div class="no-robots">
        <div class="no-robots-icon">🤖</div>
        <div class="no-robots-text">No robots detected</div>
        <div class="no-robots-subtitle">Start a Nova program to see available robots</div>
    </div>
    `
    }
    
    <div class="refresh-section">
        <button class="refresh-btn" onclick="refreshRobots()">
            🔄 Refresh Robot List
        </button>
    </div>

    <script>
        const vscode = acquireVsCodeApi();
        
        function refreshRobots() {
            vscode.postMessage({ command: 'refresh' });
        }
        
        function pauseRobot() {
            const robotId = getRobotId();
            vscode.postMessage({
                command: 'pause',
                robotId: robotId
            });
        }
        
        function resumeRobot() {
            const robotId = getRobotId();
            vscode.postMessage({
                command: 'resume',
                robotId: robotId
            });
        }
        
        function stepForward() {
            const robotId = getRobotId();
            vscode.postMessage({
                command: 'stepForward',
                robotId: robotId
            });
        }
        
        function stepBackward() {
            const robotId = getRobotId();
            vscode.postMessage({
                command: 'stepBackward',
                robotId: robotId
            });
        }
        
        function setSpeed(speed) {
            const robotId = getRobotId();
            vscode.postMessage({
                command: 'setSpeed',
                robotId: robotId,
                value: parseInt(speed)
            });
        }
        
        function updateSpeedDisplay(speed) {
            const speedValue = document.getElementById('speedValue');
            if (speedValue) {
                speedValue.textContent = speed + '%';
            }
        }
        
        function getRobotId() {
            const select = document.getElementById('robotSelect');
            return select ? select.value : '${selectedRobot?.id || ""}';
        }
        
        // Handle robot selection change
        const robotSelect = document.getElementById('robotSelect');
        if (robotSelect) {
            robotSelect.addEventListener('change', function(e) {
                vscode.postMessage({
                    command: 'selectRobot',
                    robotId: e.target.value
                });
            });
        }
    </script>
</body>
</html>`;
  }
}

/**
 * Extension activation
 */
function activate(context) {
  console.log("Nova Robot Control extension (WebSocket) is activating...");

  const controller = new NovaWebSocketController();

  // Create sidebar provider
  const sidebarProvider = new NovaRobotControlProvider(controller);
  controller.sidebarProvider = sidebarProvider;

  // Register sidebar webview provider
  const sidebarProviderRegistration = vscode.window.registerWebviewViewProvider(
    "nova.robotControlView",
    sidebarProvider
  );

  // Register commands
  const showPanelCommand = vscode.commands.registerCommand(
    "nova.showPanel",
    () => {
      controller.showPanel();
    }
  );

  const showControlPanelCommand = vscode.commands.registerCommand(
    "nova.showControlPanel",
    () => {
      controller.showControlPanel();
    }
  );

  const showHelpCommand = vscode.commands.registerCommand(
    "nova.showHelp",
    () => {
      controller.showHelp();
    }
  );

  const refreshRobotsCommand = vscode.commands.registerCommand(
    "nova.refreshRobots",
    async () => {
      controller.sendCommand({ type: "get_robots" });
      vscode.window.showInformationMessage("✓ Robot list refreshed");
    }
  );

  // Add focus command for status bar
  const focusCommand = vscode.commands.registerCommand("nova.focus", () => {
    vscode.commands.executeCommand("nova-robot-control.focus");
  });

  // Listen for configuration changes
  const configChangeListener = vscode.workspace.onDidChangeConfiguration(
    (event) => {
      if (event.affectsConfiguration("nova.websocket")) {
        console.log("Nova WebSocket configuration changed");
        controller.updateConfiguration();
      }
    }
  );

  // Add to subscriptions
  context.subscriptions.push(sidebarProviderRegistration);
  context.subscriptions.push(showPanelCommand);
  context.subscriptions.push(showControlPanelCommand);
  context.subscriptions.push(showHelpCommand);
  context.subscriptions.push(refreshRobotsCommand);
  context.subscriptions.push(focusCommand);
  context.subscriptions.push(configChangeListener);
  context.subscriptions.push(controller);

  console.log(
    "✓ Nova Robot Control extension (WebSocket) activated successfully!"
  );
}

/**
 * Extension deactivation
 */
function deactivate() {
  console.log("Nova Robot Control extension (WebSocket) deactivated");
}

module.exports = {
  activate,
  deactivate,
};
