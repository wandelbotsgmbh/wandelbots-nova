import * as fs from 'fs'
import * as os from 'os'
import * as path from 'path'
import * as vscode from 'vscode'

import { NovaCodeLensProvider } from './codeLens'
import {
  COMMAND_DEBUG_NOVA_PROGRAM,
  COMMAND_FINE_TUNE_NOVA_PROGRAM,
  COMMAND_OPEN_NOVA_VIEWER,
  COMMAND_READ_ROBOT_POSE,
  COMMAND_REFRESH_CODE_LENS,
  COMMAND_REFRESH_NOVA_VIEWER,
  COMMAND_RUN_NOVA_PROGRAM,
  COMMAND_SELECT_VIEWER_TAB,
  COMMAND_SHOW_APP,
  VIEWER_ID,
} from './consts'
import {
  startNatsLineSubscriber,
  stopNatsLineSubscriber,
} from './lineHighlighter'
import { logger } from './logging'
import { readRobotPose } from './nova/readRobotPose'
import { NovaApi } from './novaApi'
import { runNovaProgram } from './novaProgram'
import { getAccessToken, getCellId, getNovaApiAddress } from './urlResolver'
import {
  WandelbotsNovaViewerProvider,
  setupPythonScriptMonitoring,
} from './viewer'

let decorationType: vscode.TextEditorDecorationType | undefined
let disposables: vscode.Disposable[] = []

export async function activate(context: vscode.ExtensionContext) {
  logger.info('Wandelbots NOVA extension activating...')

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
    vscode.commands.registerCommand(COMMAND_OPEN_NOVA_VIEWER, async () => {
      // Set desired tab before revealing so initialTab is injected on first load
      provider.selectTab(1)
      await provider.forceReveal()
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

  // Register command to open extension settings (used by view title gear)
  context.subscriptions.push(
    vscode.commands.registerCommand('wandelbots-nova.openSettings', async () => {
      await vscode.commands.executeCommand(
        'workbench.action.openSettings',
        '@ext:wandelbots.wandelbots-nova',
      )
    }),
  )

  // Register command to select a specific tab in the viewer
  context.subscriptions.push(
    vscode.commands.registerCommand(
      COMMAND_SELECT_VIEWER_TAB,
      async (tabIndex: number) => {
        // Select the tab BEFORE reveal so first-time open uses initialTab
        provider.selectTab(typeof tabIndex === 'number' ? tabIndex : 0)
        await provider.forceReveal()
      },
    ),
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

  // Register command to fine-tune Nova program
  context.subscriptions.push(
    vscode.commands.registerCommand(
      COMMAND_FINE_TUNE_NOVA_PROGRAM,
      async (uri, functionName, line) => {
        await runNovaProgram(uri, functionName, false, true)
      },
    ),
  )

  // Register command to refresh CodeLens
  context.subscriptions.push(
    vscode.commands.registerCommand(COMMAND_REFRESH_CODE_LENS, () => {
      logger.info('Refreshing Nova CodeLens')
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
        const novaApiUrl = getNovaApiAddress()
        if (!novaApiUrl) {
          vscode.window.showErrorMessage(
            'Nova API URL not configured. Please set "wandelbots-nova-viewer.novaApi" in your VSCode settings.',
          )
          return
        }

        const accessToken = getAccessToken()

        const cellId = getCellId()
        logger.info('cellId', cellId)
        if (!cellId) return

        const novaApi = new NovaApi()
        logger.info('novaApi', novaApi)

        await novaApi.connect({
          apiUrl: novaApiUrl,
          accessToken,
          cellId,
        })

        await readRobotPose(novaApi)
      } catch (error) {
        logger.error('Error reading robot pose:', error)
        vscode.window.showErrorMessage(`Failed to read robot pose: ${error}`)
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

  await startNatsLineSubscriber(context, {
    servers: 'nats://localhost:4222',
    subject: 'editor.motion-event',
    name: 'vscode-line-highlighter',
  })

  context.subscriptions.push(...disposables)
}

export async function deactivate() {
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

  await stopNatsLineSubscriber()
}
