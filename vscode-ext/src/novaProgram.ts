import * as fs from 'fs'
import * as os from 'os'
import * as path from 'path'
import * as vscode from 'vscode'

import { COMMAND_SELECT_VIEWER_TAB } from './consts'
import { getNovaApiAddress } from './urlResolver'

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
    // Ensure the file is saved before running/debugging
    let document = await vscode.workspace.openTextDocument(uri)
    if (document.isUntitled || document.isDirty) {
      const choice = await vscode.window.showWarningMessage(
        'This file has unsaved changes. Save before running?',
        { modal: true },
        'Save',
        "Don't Save",
        'Cancel',
      )

      if (choice === 'Cancel' || choice === undefined) {
        return
      }

      if (choice === 'Save') {
        const saved = await document.save()
        if (!saved) {
          vscode.window.showErrorMessage('File was not saved. Aborting run.')
          return
        }
        // Document URI may change after save (e.g., from untitled)
        uri = document.uri
      } else if (document.isUntitled) {
        // Cannot run an untitled (unsaved) document without saving
        vscode.window.showErrorMessage('Please save the file before running.')
        return
      }
    }

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

    // Resolve NOVA_API from settings/environment
    const novaApiAddress = getNovaApiAddress()

    if (debug) {
      // For debugging, we'll use VS Code's built-in Python debugger
      vscode.window.showInformationMessage(
        `Starting debug session for NOVA program: ${functionName}`,
      )

      const debugConfig: vscode.DebugConfiguration = {
        name: `Debug NOVA program: ${functionName}`,
        type: 'python',
        request: 'launch',
        program: filePath,
        console: 'integratedTerminal',
        cwd: workspaceFolder.uri.fsPath,
        env: {
          ENABLE_TRAJECTORY_TUNING: '1',
          NOVA_API: novaApiAddress,
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
# NOVA functions return coroutines when called, even if they don't appear as coroutine functions
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

    // Inject NOVA_API and optional tuning flag into environment for the run
    if (process.platform === 'win32') {
      const envParts: string[] = [`set NOVA_API=${novaApiAddress}`]
      if (fineTune) envParts.push('set ENABLE_TRAJECTORY_TUNING=1')
      command = `${envParts.join(' && ')} && ${command}`
    } else {
      const envParts: string[] = [`NOVA_API="${novaApiAddress}"`]
      if (fineTune) envParts.push('ENABLE_TRAJECTORY_TUNING=1')
      command = `${envParts.join(' ')} ${command}`
    }

    vscode.window.showInformationMessage(
      fineTune
        ? `Running NOVA program with trajectory tuning: ${functionName}`
        : `Running NOVA program: ${functionName}`,
    )

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

    // If fine-tune requested, pre-select the Fine-Tuning tab (index 1 due to hidden tab2)
    if (fineTune) {
      try {
        await vscode.commands.executeCommand(COMMAND_SELECT_VIEWER_TAB, 1)
      } catch (e) {
        console.log('Could not select fine-tuning tab:', e)
      }
    }

    // Send the command to terminal (nova.rrd file watcher will handle completion)
    console.log(`🚀 Sending command to terminal "${terminalName}": ${command}`)
    terminal.sendText(command)
    console.log(
      `✅ NOVA program function started: ${functionName} in terminal: ${terminalName}`,
    )
    console.log(
      '⏳ Waiting for nova.rrd file to be created/updated to detect completion...',
    )
    console.log(
      '📝 Make sure your NOVA program writes to nova.rrd when it completes!',
    )
  } catch (error: any) {
    vscode.window.showErrorMessage(
      `Failed to run NOVA program: ${error.message}`,
    )
    console.error('Error running NOVA program:', error)
  }
}
