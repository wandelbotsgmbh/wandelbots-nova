import * as fs from 'fs'
import * as os from 'os'
import * as path from 'path'
import * as vscode from 'vscode'

import { NovaCodeLensProvider } from './codeLens'
import {
  COMMAND_DEBUG_NOVA_PROGRAM,
  COMMAND_OPEN_NOVA_VIEWER,
  COMMAND_READ_ROBOT_POSE,
  COMMAND_REFRESH_CODE_LENS,
  COMMAND_REFRESH_NOVA_VIEWER,
  COMMAND_RUN_NOVA_PROGRAM,
  COMMAND_SHOW_APP,
  VIEWER_ID,
} from './consts'
import { NovaApi } from './novaApi'
import { runNovaProgram } from './novaProgram'
import { getNovaApiAddress } from './urlResolver'
import {
  WandelbotsNovaViewerProvider,
  setupPythonScriptMonitoring,
} from './viewer'

let decorationType: vscode.TextEditorDecorationType | undefined
let disposables: vscode.Disposable[] = []

export function activate(context: vscode.ExtensionContext) {
  console.log('Wandelbots NOVA extension activating...')

  // ------------------------------
  // Wandelbots NOVA Viewer
  // ------------------------------

  const provider = new WandelbotsNovaViewerProvider(context.extensionUri)

  // Register the custom view provider
  context.subscriptions.push(
    vscode.window.registerWebviewViewProvider(VIEWER_ID, provider),
  )

  setupPythonScriptMonitoring(context, provider)

  // Register command to open the webview
  context.subscriptions.push(
    vscode.commands.registerCommand(COMMAND_OPEN_NOVA_VIEWER, () => {
      provider.forceReveal()
    }),
  )

  // Register command to refresh the webview
  context.subscriptions.push(
    vscode.commands.registerCommand(COMMAND_REFRESH_NOVA_VIEWER, async () => {
      // Use the new hard refresh method for manual refreshes
      await provider.hardRefresh()
      vscode.window.showInformationMessage('Wandelbots NOVA Viewer refreshed')
    }),
  )

  // Listen for configuration changes
  context.subscriptions.push(
    vscode.workspace.onDidChangeConfiguration((e) => {
      if (e.affectsConfiguration(VIEWER_ID)) {
        vscode.commands.executeCommand(COMMAND_REFRESH_NOVA_VIEWER)
      }
    }),
  )

  // ------------------------------
  // Wandelbots NOVA CodeLens
  // ------------------------------

  const novaCodeLensProvider = new NovaCodeLensProvider()

  // Register Nova CodeLens provider
  context.subscriptions.push(
    vscode.languages.registerCodeLensProvider(
      { language: 'python' },
      novaCodeLensProvider,
    ),
  )

  // Register command to run Nova program
  context.subscriptions.push(
    vscode.commands.registerCommand(
      COMMAND_RUN_NOVA_PROGRAM,
      async (uri, functionName, line) => {
        await runNovaProgram(uri, functionName, false)
      },
    ),
  )

  // Register command to debug Nova program
  context.subscriptions.push(
    vscode.commands.registerCommand(
      COMMAND_DEBUG_NOVA_PROGRAM,
      async (uri, functionName, line) => {
        await runNovaProgram(uri, functionName, true)
      },
    ),
  )

  // Register command to refresh CodeLens
  context.subscriptions.push(
    vscode.commands.registerCommand(COMMAND_REFRESH_CODE_LENS, () => {
      console.log('Refreshing Nova CodeLens')
      novaCodeLensProvider.refresh()
      vscode.window.showInformationMessage('Nova CodeLens refreshed')
    }),
  )

  // ------------------------------
  // Wandelbots NOVA Robot Pose
  // ------------------------------

  context.subscriptions.push(
    vscode.commands.registerCommand(COMMAND_READ_ROBOT_POSE, async () => {
      try {
        // Get configuration from VSCode settings
        const novaApiUrl = getNovaApiAddress()

        // Note: novaApiUrl is still required for validation, even though getNovaApiAddress() is used for the actual connection
        if (!novaApiUrl) {
          vscode.window.showErrorMessage(
            'Nova API URL not configured. Please set "wandelbots-nova-viewer.novaApi" in your VSCode settings.',
          )
          return
        }

        /*if (!accessToken) {
          vscode.window.showErrorMessage(
            'Nova access token not configured. Please set "wandelbots-nova-viewer.accessToken" in your VSCode settings.',
          )
          return
        }*/

        // Prompt for cell ID
        const cellId = await vscode.window.showInputBox({
          prompt: 'Enter the cell ID',
          placeHolder: 'e.g., cell',
        })

        if (!cellId) {
          return
        }

        const novaApi = new NovaApi()

        try {
          await novaApi.connect({
            apiUrl: novaApiUrl,
            accessToken: 'test-token',
            cellId,
          })

          // Get controllers
          const controllers = await novaApi.getControllers()

          if (controllers.length === 0) {
            vscode.window.showErrorMessage('No controllers found in the cell')
            return
          }

          let selectedController: any

          if (controllers.length === 1) {
            selectedController = controllers[0]
          } else {
            // Show selection list for multiple controllers
            const controllerNames = controllers.map((c) => c.controller)
            const selectedControllerName = await vscode.window.showQuickPick(
              controllerNames,
              {
                placeHolder: 'Select a controller',
              },
            )

            if (!selectedControllerName) {
              return
            }

            selectedController = controllers.find(
              (c) => c.controller === selectedControllerName,
            )
          }

          // Get motion groups for the selected controller
          const motionGroups = await novaApi.getMotionGroups(
            selectedController.controller,
          )

          if (motionGroups.length === 0) {
            vscode.window.showErrorMessage(
              'No motion groups found for the selected controller',
            )
            return
          }

          let selectedMotionGroup: any

          if (motionGroups.length === 1) {
            selectedMotionGroup = motionGroups[0]
          } else {
            // Show selection list for multiple motion groups
            const motionGroupNames = motionGroups.map((mg) => mg.id)
            const selectedMotionGroupName = await vscode.window.showQuickPick(
              motionGroupNames,
              {
                placeHolder: 'Select a motion group',
              },
            )

            if (!selectedMotionGroupName) {
              return
            }

            selectedMotionGroup = motionGroups.find(
              (mg) => mg.id === selectedMotionGroupName,
            )
          }

          // Get TCPs for the selected motion group
          const tcps = await novaApi.getTcps(selectedMotionGroup.id)

          if (tcps.length === 0) {
            vscode.window.showErrorMessage(
              'No TCPs found for the selected motion group',
            )
            return
          }

          let selectedTcp: any

          if (tcps.length === 1) {
            selectedTcp = tcps[0]
          } else {
            // Show selection list for multiple TCPs
            const tcpNames = tcps.map((tcp) => tcp.id)
            const selectedTcpName = await vscode.window.showQuickPick(tcpNames, {
              placeHolder: 'Select a TCP',
            })

            if (!selectedTcpName) {
              return
            }

            selectedTcp = tcps.find((tcp) => tcp.id === selectedTcpName)
          }

          // Get the robot pose
          const pose = await novaApi.getRobotPose(
            selectedMotionGroup.id,
            selectedTcp.id,
          )

          // Format the pose string
          const poseString = `Pose((${pose.x.toFixed(3)}, ${pose.y.toFixed(3)}, ${pose.z.toFixed(3)}, ${pose.rx.toFixed(3)}, ${pose.ry.toFixed(3)}, ${pose.rz.toFixed(3)}))`

          // Get the active text editor
          const editor = vscode.window.activeTextEditor
          if (editor) {
            // Insert the pose at the current cursor position
            await editor.edit((editBuilder) => {
              editBuilder.insert(editor.selection.active, poseString)
            })

            vscode.window.showInformationMessage(
              `Robot pose inserted: ${poseString}`,
            )
          } else {
            // If no active editor, show the pose in a message
            vscode.window.showInformationMessage(`Robot pose: ${poseString}`)
          }
        } finally {
          novaApi.dispose()
        }
      } catch (error) {
        vscode.window.showErrorMessage(`Failed to read robot pose: ${error}`)
        console.error('Error reading robot pose:', error)
      }
    }),
  )

  // ------------------------------
  // Wandelbots NOVA App
  // ------------------------------

  context.subscriptions.push(
    vscode.commands.registerCommand(COMMAND_SHOW_APP, () => {
      const panel = vscode.window.createWebviewPanel(
        'webview',
        'React',
        vscode.ViewColumn.One,
        {
          enableScripts: true,
        },
      )

      // Get path to index.html in build directory
      const indexPath = vscode.Uri.joinPath(
        context.extensionUri,
        'app',
        'build',
        'index.html',
      )

      // Read the HTML file content
      const htmlContent = fs.readFileSync(indexPath.fsPath, 'utf8')

      // Convert any resource URIs to webview URIs
      const buildUri = panel.webview.asWebviewUri(
        vscode.Uri.joinPath(context.extensionUri, 'app', 'build'),
      )

      // Replace paths to be webview-friendly
      const updatedHtml = htmlContent.replace(
        /(href|src)="([^"]*)"/g,
        (match, attr, path) => {
          if (path.startsWith('/')) {
            path = path.slice(1)
          }
          return `${attr}="${buildUri}/${path}"`
        },
      )

      panel.webview.html = updatedHtml
    }),
  )

  // Refresh CodeLens when documents change
  context.subscriptions.push(
    vscode.workspace.onDidChangeTextDocument((e) => {
      if (e.document.languageId === 'python') {
        novaCodeLensProvider.refresh()
      }
    }),
  )

  context.subscriptions.push(...disposables)
}

export function deactivate() {
  decorationType?.dispose()
  disposables.forEach((d) => d.dispose())

  // Clean up temp files if needed
  const tempDir = path.join(__dirname, os.tmpdir())
  if (fs.existsSync(tempDir)) {
    try {
      fs.rmdirSync(tempDir, { recursive: true })
    } catch (error) {
      console.error('Failed to clean up temp directory:', error)
    }
  }
}
