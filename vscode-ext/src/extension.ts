import * as fs from 'fs'
import * as os from 'os'
import * as path from 'path'
import * as vscode from 'vscode'

import { NovaCodeLensProvider } from './codeLens'
import {
  COMMAND_DEBUG_NOVA_PROGRAM,
  COMMAND_OPEN_NOVA_VIEWER,
  COMMAND_REFRESH_CODE_LENS,
  COMMAND_REFRESH_NOVA_VIEWER,
  COMMAND_RUN_NOVA_PROGRAM,
  COMMAND_SHOW_APP,
  VIEWER_ID,
} from './consts'
import { runNovaProgram } from './novaProgram'
import {
  WandelbotsNovaViewerProvider,
  setupPythonScriptMonitoring,
} from './viewer'
import { getConfiguredUrl } from './urlResolver'

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
  // Wandelbots NOVA App
  // ------------------------------

  context.subscriptions.push(vscode.commands.registerCommand(COMMAND_SHOW_APP, () => {
    const panel = vscode.window.createWebviewPanel("webview", "React", vscode.ViewColumn.One, {
        enableScripts: true
    })

    const scriptSrc = panel.webview.asWebviewUri(vscode.Uri.joinPath(context.extensionUri, "app", "build", "index.js"))
    const cssSrc = panel.webview.asWebviewUri(vscode.Uri.joinPath(context.extensionUri, "app", "build", "index.css"))

    panel.webview.html = `<!DOCTYPE html>
    <html lang="en">
      <head>
        <link rel="stylesheet" href="${cssSrc}" />
      </head>
      <body>
        <noscript>You need to enable JavaScript to run this app.</noscript>
        <div id="root"></div>
        <script src="${scriptSrc}"></script>
      </body>
    </html>
    `
  }))

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
