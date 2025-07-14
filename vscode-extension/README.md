# Nova Robot Control VS Code Extension

A VS Code extension for real-time control of Nova robots through WebSocket connection. This extension provides comprehensive robot control capabilities with support for multiple robots, real-time event broadcasting, and enhanced state management.

## Features

### 🤖 Robot Discovery & Management

- **Automatic Robot Registration**: Robots are automatically discovered when motion groups are created
- **Multi-Robot Support**: Control multiple robots simultaneously with parallel execution support
- **Real-time Status Updates**: Live monitoring of robot states, speeds, and execution status
- **Enhanced Robot Metadata**: Display robot names, registration times, and detailed status information

### 🎮 Comprehensive Control

- **Speed Control**: Adjust execution speed from 0-100% with quick presets
- **Pause/Resume**: Fine-grained control over robot execution
- **Direction Control**: Step forward/backward through robot trajectories
- **Smart State Management**: Intelligent pause/resume behavior with speed preservation

### 📡 Enhanced WebSocket Integration

- **Event-Based Architecture**: Real-time event broadcasting from Nova WebSocket server
- **Program Lifecycle Events**: Notifications for program start/stop events
- **Connection Management**: Auto-reconnection with configurable intervals
- **Comprehensive Error Handling**: Robust error handling and user feedback

### 🎨 Modern UI

- **Sidebar Panel**: Comprehensive robot control panel with live updates
- **Quick Pick Interface**: Fast robot selection and control via Command Palette
- **Visual State Indicators**: Color-coded robot states and execution status
- **Responsive Design**: Optimized for different VS Code themes and layouts

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

## Installation

1. Open VS Code
2. Go to Extensions (Ctrl+Shift+X)
3. Search for "Nova Robot Control"
4. Install the extension

### From Source (Development)

```bash
# Clone repository
git clone https://github.com/wandelbotsgmbh/wandelbots-nova
cd wandelbots-nova/vscode-extension

# Install dependencies
npm install

# Package extension
npm run package
```

## Usage

### 1. Setup Nova Program

Create a Nova program with WebSocket control enabled:

```python
import nova
from nova.external_control import WebSocketControl

@nova.program(
    name="My Robot Program",
    external_control=WebSocketControl()  # Enable WebSocket control
)
async def my_program():
    async with nova.Nova() as nova:
        cell = nova.cell()
        controller = await cell.controller("my_controller")

        async with controller[0] as motion_group:
            # Your robot movements here
            await motion_group.plan_and_execute(actions, tcp)
```

### 2. Connect Extension

The extension automatically connects to the WebSocket server when:

- A Nova program with WebSocket control is running
- The server is available on `localhost:8765` (configurable)

### 3. Control Robots

#### Via Sidebar Panel

1. Open the Nova Robot Control sidebar panel
2. View all registered robots with their current status
3. Use controls directly in the panel:
   - Adjust speed with slider
   - Pause/Resume execution
   - View detailed robot information

#### Via Command Palette

1. Open Command Palette (Ctrl+Shift+P)
2. Type "Nova" to see available commands:
   - `Nova: Show Robot Controls` - Quick robot selection and control
   - `Nova: Show Robot Control Panel` - Open sidebar panel
   - `Nova: Refresh Robot List` - Manual refresh
   - `Nova: Show Help` - Display help information

#### Status Bar Integration

- Click the robot icon in the status bar for quick access
- Shows live count of executing/paused/total robots
- Visual indicators for connection status

## Configuration

Configure the extension via VS Code settings:

```json
{
  "nova.websocket.host": "localhost",
  "nova.websocket.port": 8765,
  "nova.websocket.autoReconnect": true,
  "nova.websocket.reconnectInterval": 3000
}
```

### Settings Details

| Setting                            | Description                       | Default       |
| ---------------------------------- | --------------------------------- | ------------- |
| `nova.websocket.host`              | WebSocket server host             | `"localhost"` |
| `nova.websocket.port`              | WebSocket server port             | `8765`        |
| `nova.websocket.autoReconnect`     | Auto-reconnect on connection loss | `true`        |
| `nova.websocket.reconnectInterval` | Reconnection interval (ms)        | `3000`        |

## Multiple Robot Support

The extension fully supports controlling multiple robots simultaneously:

```python
@nova.program(
    name="Multi-Robot Demo",
    external_control=WebSocketControl(),
    preconditions=ProgramPreconditions(
        controllers=[
            virtual_controller(name="robot1", ...),
            virtual_controller(name="robot2", ...),
        ]
    )
)
async def multi_robot_program():
    async with Nova() as nova:
        cell = nova.cell()
        robot1 = await cell.controller("robot1")
        robot2 = await cell.controller("robot2")

        # Both robots will be registered and controllable
        async with robot1[0] as mg1, robot2[0] as mg2:
            # Parallel execution - both robots controllable independently
            await asyncio.gather(
                mg1.plan_and_execute(actions1, tcp1),
                mg2.plan_and_execute(actions2, tcp2)
            )
```

## Event System

The extension listens to comprehensive events from the Nova WebSocket server:

- **Robot Registration/Unregistration**: When robots become available/unavailable
- **State Changes**: Execution start/stop, pause/resume events
- **Speed Changes**: Real-time speed adjustments with source tracking
- **Program Lifecycle**: Program start/stop notifications
- **Execution Events**: Detailed execution state tracking

## Troubleshooting

### Connection Issues

1. **Extension shows "Disconnected"**

   - Ensure Nova program with `WebSocketControl()` is running
   - Check host/port configuration in VS Code settings
   - Verify no firewall is blocking the connection

2. **Robots not appearing**

   - Confirm motion groups are created in Nova program
   - Use "Refresh Robot List" command
   - Check Nova program logs for registration events

3. **Controls not working**
   - Verify WebSocket connection is active
   - Check robot state (some controls only work in specific states)
   - Look for error messages in VS Code output panel

### Performance Tips

- Use auto-reconnect for reliable connections
- Adjust reconnection interval based on network stability
- Monitor VS Code output panel for detailed logging

## Development

### Building from Source

```bash
# Clone repository
git clone https://github.com/wandelbotsgmbh/wandelbots-nova
cd wandelbots-nova/vscode-extension

# Install dependencies
npm install

# Package extension
npm run package
```

### Debugging

1. Open extension folder in VS Code
2. Press F5 to launch Extension Development Host
3. Test with Nova programs in the development instance

### File Structure

```
vscode-extension/
├── package.json               # Extension manifest
├── extension-websocket.js     # Main extension code
├── README.md                  # Documentation
└── resources/                 # Icons and assets
    └── robot-icon.svg
```

### Key Classes

- **NovaController** - Main WebSocket controller
- **NovaSidebarProvider** - Sidebar webview provider
- **Event Handlers** - Comprehensive event processing

## Changelog

### Version 3.0.0

- **Enhanced WebSocket Integration**: Full support for new event-based architecture
- **Robot Registration Events**: Automatic discovery when motion groups are created
- **Multiple Robot Support**: Comprehensive parallel execution control
- **Improved UI**: Modern sidebar with enhanced robot information
- **Better Error Handling**: Robust connection management and user feedback
- **Configuration Management**: Live configuration updates without restart

### Previous Versions

- Version 2.x: Smart speed control and functional stepping
- Version 1.x: Basic WebSocket robot control

## License

This extension is part of the Wandelbots Nova project. See LICENSE file for details.

## Support

For issues, questions, or contributions:

- GitHub Issues: [wandelbots-nova repository](https://github.com/wandelbotsgmbh/wandelbots-nova)
- Documentation: [Nova Documentation](https://docs.wandelbots.io)

````

2. Open VS Code in the same workspace
3. Install the extension
4. Check status bar for robot detection
5. Click robot icon to open controls

## Troubleshooting

### No Robots Detected

1. **Check Nova Installation:**

```bash
python -c "import nova; print('Nova available')"
````

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
