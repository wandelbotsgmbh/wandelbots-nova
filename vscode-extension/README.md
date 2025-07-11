# Nova Robot Control VS Code Extension

A VS Code extension that provides real-time detection and control of Nova robots using WebSocket communication.

## Features

- **Real-time Detection** - Automatically detects Nova robots via WebSocket
- **Live Updates** - Real-time robot status and feedback
- **Status Bar Integration** - Shows robot count and status
- **Speed Control** - Adjust robot speed with input dialogs
- **Pause/Resume** - Control robot execution
- **Native GUI** - Uses only built-in VS Code components
- **Zero Setup** - Works automatically when Nova is imported

## How It Works

When you import Nova in your Python program, a WebSocket server starts automatically on `ws://localhost:8765`. The VS Code extension connects to this server for real-time robot control and monitoring.

**Enhanced WebSocket Features:**

- **Automatic Reconnection** - Reconnects if connection is lost
- **Heartbeat Monitoring** - Detects disconnections quickly
- **Real-time Updates** - Live robot status changes
- **Error Handling** - Clear user feedback for connection issues
- **Smart Refresh** - Refreshes robot list or reconnects as needed

## Installation

### Option 1: Install from VSIX (Recommended)

1. Package the extension:

```bash
cd vscode-extension
npm install -g vsce
vsce package
```

2. Install in VS Code:
   - Open VS Code
   - Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
   - Type "Extensions: Install from VSIX"
   - Select the generated `nova-robot-control-2.0.0.vsix` file

### Option 2: Development Mode

1. Open the `vscode-extension` folder in VS Code
2. Press `F5` to launch Extension Development Host
3. The extension will be active in the new VS Code window

## Usage

### Automatic Operation

The extension automatically:

1. Detects your Python environment
2. Monitors for Nova robots every 3 seconds
3. Updates the status bar with robot information
4. Shows controls when robots are detected

### Manual Commands

Access via Command Palette (`Ctrl+Shift+P`):

- **Nova: Show Robot Controls** - Open main control panel
- **Nova: Detect Robots** - Manually check for robots
- **Nova: Refresh Robot List** - Update robot status

### Status Bar

Click the robot icon in the status bar to open controls:

- 🤖 **Nova: Not detected** - Nova not available
- 🤖 **Nova: No robots** - Nova detected, no robots running
- 🤖 **Nova: 1/2** - 1 active robot out of 2 total

### Robot Controls

**Speed Control:**

1. Select robot from list
2. Choose "Change Speed"
3. Enter speed percentage (0-100)

**Pause/Resume:**

1. Select robot from list
2. Choose "Pause Robot" or "Resume Robot"
3. Confirm action

## Requirements

- **Nova Python Library** - Must be installed in your project
- **Active Nova Program** - Running Nova code with robots
- **WebSocket Connection** - Nova automatically starts WebSocket server on port 8765

## Testing the Extension

### Test WebSocket Connection

```bash
cd vscode-extension
node test-websocket.js
```

This will test the WebSocket connection to Nova (make sure a Nova program is running first).

### Manual Testing Steps

1. **Start a Nova program:**

   ```bash
   python examples/websocket_control_test.py
   ```

2. **Check VS Code status bar** - Should show "Nova: X robots"

3. **Use Command Palette:**

   - Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
   - Type "Nova: Show Robot Controls"

4. **Test robot control** through the Quick Pick interface

## Troubleshooting

### Extension Not Detecting Nova

- ✅ Make sure a Nova program is running
- ✅ Check that WebSocket server started (port 8765 should be open)
- ✅ Try "Nova: Refresh Robot List" command
- ✅ Check VS Code Developer Console for errors

### WebSocket Connection Failed

- ✅ Verify Nova program imports `nova` library
- ✅ Check if port 8765 is available (not blocked by firewall)
- ✅ Look for "WebSocket server started" message in Nova output

## Python Environment Detection (Legacy)

The extension now uses WebSocket and doesn't need Python environment detection. Legacy info:

1. `.venv/bin/python` (Linux/Mac) or `.venv/Scripts/python.exe` (Windows)
2. `venv/bin/python` (Linux/Mac) or `venv/Scripts/python.exe` (Windows)
3. VS Code Python extension settings
4. System `python` command

## Native GUI Components Used

- **Status Bar Item** - Shows robot status
- **Quick Pick Menus** - Main navigation and robot selection
- **Input Box** - Speed adjustment
- **Information Messages** - Success notifications
- **Error Messages** - Failure notifications
- **Command Palette** - Manual commands

## Screenshots

### Status Bar

```
🤖 Nova: 2/3    [Click to open controls]
```

### Main Control Panel

```
┌─────────────────────────────────────┐
│ ℹ️  Nova Robot Control              │
│    3 robots detected                │
│                                     │
│ ▶️  robot1        75% speed         │
│    State: executing                 │
│                                     │
│ ⏸️  robot2        50% speed         │
│    State: paused                    │
│                                     │
│ 🔄 Refresh Robot List              │
└─────────────────────────────────────┘
```

### Robot Controls

```
┌─────────────────────────────────────┐
│ 🤖 robot1                          │
│    75% speed - executing            │
│                                     │
│ 📊 Change Speed     Current: 75%    │
│    Set new speed percentage (0-100) │
│                                     │
│ ⏸️  Pause Robot                     │
│    Pause robot execution            │
│                                     │
│ ← Back to Robot List               │
└─────────────────────────────────────┘
```

## Development

### File Structure

```
vscode-extension/
├── package.json      # Extension manifest
├── extension.js      # Main extension code
└── README.md        # This file
```

### Key Classes

- **NovaController** - Main controller class
- **executeCommand()** - Subprocess communication with Nova
- **showPanel()** - Main Quick Pick interface
- **showRobotControls()** - Individual robot controls

### Testing

1. Run a Nova program:

```bash
python examples/vscode_auto_detection_demo.py
```

2. Open VS Code in the same workspace
3. Install the extension
4. Check status bar for robot detection
5. Click robot icon to open controls

## Troubleshooting

### No Robots Detected

1. **Check Nova Installation:**

```bash
python -c "import nova; print('Nova available')"
```

2. **Verify Robot Program:**

   - Make sure your Nova program is actively running
   - Check that robots are executing movements

3. **Python Environment:**
   - Verify extension is using correct Python
   - Check Command Palette > "Python: Select Interpreter"

### Connection Issues

1. **Refresh Robot List:**

   - Use Command Palette > "Nova: Refresh Robot List"
   - Or click status bar and select "Refresh"

2. **Check Logs:**
   - Open Developer Console: Help > Toggle Developer Tools
   - Look for Nova extension messages

### Performance

- Extension checks for robots every 3 seconds
- Minimal CPU usage when no robots detected
- Subprocess calls timeout after 10 seconds

## License

Same as wandelbots-nova library.
