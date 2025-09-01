import * as fs from 'fs'
import * as os from 'os'
import * as path from 'path'
import * as vscode from 'vscode'

interface PythonExecutionDetails {
  execCommand?: string[]
}
interface PythonSettingsAPI {
  getExecutionDetails(resource: vscode.Uri): Promise<PythonExecutionDetails>
}
interface PythonExtensionAPI {
  settings?: PythonSettingsAPI
}

/**
 * Run or debug a Nova program
 * @param uri - File URI
 * @param functionName - Name of the function to run
 * @param debug - Whether to run in debug mode
 * @param fineTune - Whether to run with trajectory tuning enabled
 */
export async function runNovaProgram(
  uri: vscode.Uri,
  functionName: string,
  debug: boolean,
  fineTune: boolean = false,
): Promise<void> {
  try {
    const filePath = uri.fsPath
    const workspaceFolder = vscode.workspace.getWorkspaceFolder(uri)

    if (!workspaceFolder) {
      vscode.window.showErrorMessage('No workspace folder found for this file')
      return
    }

    // Create or reuse terminal for running the program
    const terminalName = `Python: ${functionName}`
    let terminal = vscode.window.terminals.find((t) => t.name === terminalName)

    if (!terminal) {
      console.log(`Creating new terminal: ${terminalName}`)
      terminal = vscode.window.createTerminal({
        name: terminalName,
        cwd: workspaceFolder.uri.fsPath,
      })
    } else {
      console.log(`Reusing existing terminal: ${terminalName}`)
    }

    // Show the terminal
    terminal.show()

    // Try to get the Python interpreter path from VS Code's Python extension
    let pythonPath = 'python'
    try {
      const pythonExtension =
        vscode.extensions.getExtension<PythonExtensionAPI>('ms-python.python')
      if (pythonExtension && pythonExtension.isActive) {
        const pythonApi = pythonExtension.exports
        const activeInterpreter =
          await pythonApi?.settings?.getExecutionDetails(uri)
        if (activeInterpreter?.execCommand?.length) {
          pythonPath = activeInterpreter.execCommand[0]
        }
      }
    } catch (error) {
      console.log(
        'Could not get Python interpreter from Python extension, using default:',
        error,
      )
    }

    if (debug) {
      // For debugging, we'll use VS Code's built-in Python debugger
      vscode.window.showInformationMessage(
        `Starting debug session for Nova program: ${functionName}`,
      )

      const debugConfig: vscode.DebugConfiguration = {
        name: `Debug Nova Program: ${functionName}`,
        type: 'python',
        request: 'launch',
        program: filePath,
        console: 'integratedTerminal',
        cwd: workspaceFolder.uri.fsPath,
        env: {
          ENABLE_TRAJECTORY_TUNER: '1',
        },
      }

      await vscode.debug.startDebugging(workspaceFolder, debugConfig)
      return // Debug session handles execution
    }

    // Create a temporary Python file that imports and runs the specific function
    const moduleName = path.basename(filePath, '.py')
    const moduleDir = path.dirname(filePath)
    const moduleDirEscaped = moduleDir.replace(/\\/g, '\\\\')

    // TODO: solve this via the runner
    const tempFileContent = `
import sys
import os
import asyncio
import inspect

# Add the module directory to Python path
sys.path.insert(0, r'${moduleDirEscaped}')

# Import the module containing the function
import ${moduleName}

# Get the function and run it
# Nova functions return coroutines when called, even if they don't appear as coroutine functions
func = getattr(${moduleName}, '${functionName}')
result = func()

# Check if the result is a coroutine and run it with asyncio.run()
if inspect.iscoroutine(result):
    asyncio.run(result)
# If it's not a coroutine, the function executed synchronously
`.trim()

    const tempFilePath = path.join(
      os.tmpdir(),
      `nova_run_${functionName}_${Date.now()}.py`,
    )
    fs.writeFileSync(tempFilePath, tempFileContent, { encoding: 'utf8' })

    let command = `"${pythonPath}" "${tempFilePath}"`

    if (fineTune) {
      // Set environment variable for trajectory tuning
      if (process.platform === 'win32') {
        // Windows: use set command
        command = `set ENABLE_TRAJECTORY_TUNER=1 && ${command}`
      } else {
        // Unix-like systems (macOS, Linux): use export
        command = `ENABLE_TRAJECTORY_TUNER=1 ${command}`
      }
      vscode.window.showInformationMessage(
        `Running Nova program with trajectory tuning: ${functionName}`,
      )
    } else {
      vscode.window.showInformationMessage(
        `Running Nova program: ${functionName}`,
      )
    }

    // Clean up temp file after a delay
    setTimeout((): void => {
      try {
        if (fs.existsSync(tempFilePath)) {
          fs.unlinkSync(tempFilePath)
          console.log(`Cleaned up temp file: ${tempFilePath}`)
        }
      } catch (error: any) {
        console.log(`Could not clean up temp file: ${error.message}`)
      }
    }, 30_000) // Clean up after 30 seconds

    // Send the command to terminal (nova.rrd file watcher will handle completion)
    console.log(`🚀 Sending command to terminal "${terminalName}": ${command}`)
    terminal.sendText(command)
    console.log(
      `✅ Nova program function started: ${functionName} in terminal: ${terminalName}`,
    )
    console.log(
      '⏳ Waiting for nova.rrd file to be created/updated to detect completion...',
    )
    console.log(
      '📝 Make sure your Nova program writes to nova.rrd when it completes!',
    )
  } catch (error: any) {
    vscode.window.showErrorMessage(
      `Failed to run Nova program: ${error.message}`,
    )
    console.error('Error running Nova program:', error)
  }
}
