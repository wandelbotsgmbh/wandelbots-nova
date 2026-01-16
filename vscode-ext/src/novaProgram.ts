import * as fs from 'fs'
import * as os from 'os'
import * as path from 'path'
import * as vscode from 'vscode'

import { COMMAND_SELECT_VIEWER_TAB } from './consts'
import { logger } from './logging'
import { getNovaConfig } from './urlResolver'

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

      if (choice === 'Cancel' || choice === undefined) return

      if (choice === 'Save') {
        const saved = await document.save()
        if (!saved) {
          vscode.window.showErrorMessage('File was not saved. Aborting run.')
          return
        }
        // URI may change after save (e.g., from untitled)
        uri = document.uri
      } else if (document.isUntitled) {
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

    // Determine Python interpreter (via Python extension if available)
    let pythonPath = 'python'
    try {
      const pythonExtension =
        vscode.extensions.getExtension<PythonExtensionAPI>('ms-python.python')
      if (pythonExtension) {
        if (!pythonExtension.isActive) {
          await pythonExtension.activate() // ensure exports populated
        }
        const pythonApi = pythonExtension.exports
        const execDetails = await pythonApi?.settings?.getExecutionDetails(uri)
        if (execDetails?.execCommand?.length) {
          pythonPath = execDetails.execCommand[0]
        }
      }
    } catch (err) {
      console.log('Falling back to default Python interpreter:', err)
    }

    // Resolve NOVA env
    const { novaApi, novaAccessToken, cellId, natsBroker } = getNovaConfig()

    const runEnv: Record<string, string> = {
      NOVA_API: novaApi,
      ...(novaAccessToken && novaAccessToken.trim()
        ? { NOVA_ACCESS_TOKEN: novaAccessToken }
        : {}),
      ...(cellId && cellId.trim() ? { CELL_NAME: cellId } : {}),
      ...(natsBroker && natsBroker.trim() ? { NATS_BROKER: natsBroker } : {}),
      ...(fineTune ? { ENABLE_TRAJECTORY_TUNING: '1' } : {}),
    }

    // Log the resolved environment used to run the program
    const log = logger.child('runNovaProgram')
    log.info('Resolved NOVA environment for program run:', runEnv)

    if (debug) {
      // Use VS Code's Python debugger
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
        env: runEnv,
      }

      await vscode.debug.startDebugging(workspaceFolder, debugConfig)
      return
    }

    // Create a temporary Python shim that imports and runs the target function
    const moduleName = path.basename(filePath, '.py')
    const moduleDir = path.dirname(filePath)
    const moduleDirEscaped = moduleDir.replace(/\\/g, '\\\\')

    const tempFileContent = `
import sys
import asyncio
import inspect

# Ensure module directory is on sys.path
sys.path.insert(0, r'${moduleDirEscaped}')

import ${moduleName}
from nova import run_program


func = getattr(${moduleName}, '${functionName}')
run_program(func)
`.trim()

    const tempFilePath = path.join(
      os.tmpdir(),
      `nova_run_${functionName}_${Date.now()}.py`,
    )
    fs.writeFileSync(tempFilePath, tempFileContent, { encoding: 'utf8' })

    // Recreate terminal to ensure fresh env is applied
    const terminalName = `Python: ${functionName}`
    vscode.window.terminals
      .filter((t) => t.name === terminalName)
      .forEach((t) => t.dispose())

    console.log(`Creating terminal "${terminalName}" with NOVA env`)
    const terminal = vscode.window.createTerminal({
      name: terminalName,
      cwd: workspaceFolder.uri.fsPath,
      env: runEnv,
    })
    terminal.show()

    // Quote paths conservatively for common shells
    const quote = (s: string) => `"${s.replace(/"/g, '\\"')}"`
    const command = `${quote(pythonPath)} ${quote(tempFilePath)}`

    // Fire it off (no env prefix)
    terminal.sendText(command, true)

    vscode.window.showInformationMessage(
      fineTune
        ? `Running NOVA program with trajectory tuning: ${functionName}`
        : `Running NOVA program: ${functionName}`,
    )

    // Optionally switch to the Fine-Tuning tab
    if (fineTune) {
      try {
        await vscode.commands.executeCommand(COMMAND_SELECT_VIEWER_TAB, 1)
      } catch (e) {
        console.warn('Could not select fine-tuning tab:', e)
      }
    }

    // Clean up temp file after a short delay
    setTimeout((): void => {
      try {
        if (fs.existsSync(tempFilePath)) {
          fs.unlinkSync(tempFilePath)
          console.log(`Cleaned up temp file: ${tempFilePath}`)
        }
      } catch (error: any) {
        console.log(`Could not clean up temp file: ${error.message}`)
      }
    }, 30_000)
  } catch (error: any) {
    vscode.window.showErrorMessage(
      `Failed to run NOVA program: ${error.message}`,
    )
    console.error('Error running NOVA program:', error)
  }
}
