import * as vscode from 'vscode'

import { EXPLORER_ID, VIEWER_ID } from './consts.js'
import { getConfiguredUrl } from './urlResolver.js'

// Track if this is the initial activation
let isInitialActivation = true

export class WandelbotsNovaViewerProvider
  implements vscode.WebviewViewProvider
{
  private _view?: vscode.WebviewView
  private _url: string = 'https://wandelbots.com' // Default URL, will be updated
  // Track running Python processes (kept from original; attach usage if needed)
  private _runningPythonProcesses: Set<number> = new Set()

  constructor(private readonly _extensionUri: vscode.Uri) {}

  /**
   * Refresh the webview content using postMessage (most efficient)
   */
  async refreshView(): Promise<void> {
    // Check if auto-refresh is enabled
    const config = vscode.workspace.getConfiguration('wandelbots-viewer')
    const autoRefresh = config.get<boolean>(
      'autoRefreshOnPythonExecution',
      true,
    )

    if (!autoRefresh) {
      console.log('Auto-refresh disabled, skipping view refresh')
      return
    }

    if (this._view) {
      console.log('Refreshing Wandelbots view due to Python script completion')

      try {
        // Method 1: Use postMessage to refresh iframe (fastest)
        this._view.webview.postMessage({
          command: 'refresh',
          timestamp: Date.now(),
        })

        console.log('Sent refresh message to webview')
      } catch (error) {
        console.error(
          'Error sending refresh message, falling back to URL refresh:',
          error,
        )
        // Fallback: Update URL and reload
        await this.hardRefresh()
      }
    }
  }

  /**
   * Hard refresh by updating the HTML content (fallback method)
   */
  async hardRefresh(): Promise<void> {
    if (this._view) {
      console.log('Performing hard refresh of Wandelbots view')

      try {
        this._url = await getConfiguredUrl()
        console.log(`Refreshed URL: ${this._url}`)

        this._view.webview.html = this._getHtmlForWebview(this._view.webview)
      } catch (error) {
        console.error('Error during hard refresh:', error)
        this._view.webview.html = this._getErrorHtml(error)
      }
    }
  }

  /**
   * Check if nova.rrd file exists in the workspace
   */
  async hasNovaRrdFile(): Promise<boolean> {
    try {
      if (
        !vscode.workspace.workspaceFolders ||
        vscode.workspace.workspaceFolders.length === 0
      ) {
        console.log('No workspace folder found')
        return false
      }

      // Search for nova.rrd files in all workspace folders
      const rrdFiles = await vscode.workspace.findFiles(
        '**/nova.rrd',
        undefined,
        1,
      )
      const hasRrdFile = rrdFiles.length > 0

      if (hasRrdFile) {
        console.log('Found nova.rrd file in workspace:', rrdFiles[0].fsPath)
      } else {
        console.log('No nova.rrd file found in workspace')
      }

      return hasRrdFile
    } catch (error) {
      console.error('Error checking for nova.rrd file:', error)
      return false
    }
  }

  /**
   * Reveal and focus the webview (only if auto-open is enabled)
   */
  async reveal(): Promise<void> {
    // Don't auto-open on initial activation (VS Code startup)
    if (isInitialActivation) {
      console.log('Skipping auto-open during initial activation')
      return
    }

    // Check if auto-open is enabled
    const config = vscode.workspace.getConfiguration(VIEWER_ID)
    const autoOpen = config.get<boolean>('autoOpenOnPythonExecution', true)

    if (!autoOpen) {
      console.log('Auto-open disabled, skipping view reveal')
      return
    }

    // Check if nova.rrd file requirement is enabled and if the file exists
    const requireNovaRrd = config.get<boolean>('requireNovaRrdFile', true)
    if (requireNovaRrd) {
      const hasNovaRrd = await this.hasNovaRrdFile()
      if (!hasNovaRrd) {
        console.log(
          'Nova.rrd file requirement enabled but no nova.rrd file found in workspace, skipping auto-open',
        )
        return
      }
    } else {
      console.log(
        'Nova.rrd file requirement disabled, proceeding with auto-open',
      )
    }

    this.forceReveal()
  }

  /**
   * Force reveal the webview (used for manual commands)
   */
  async forceReveal(): Promise<void> {
    try {
      await vscode.commands.executeCommand(
        `workbench.view.extension.${EXPLORER_ID}`,
      )

      if (this._view) {
        this._view.show?.(true)
      }

      setTimeout(() => {
        void vscode.commands.executeCommand('workbench.action.focusSideBar')
      }, 300)
    } catch (error) {
      console.error('Failed to reveal Wandelbots Viewer:', error)

      // Fallback: try to open the view again
      setTimeout(() => {
        void vscode.commands.executeCommand(
          `workbench.view.extension.${EXPLORER_ID}`,
        )
      }, 1000)
    }
  }

  /**
   * Called when the view is first opened or becomes visible again
   */
  async resolveWebviewView(
    webviewView: vscode.WebviewView,
    context: vscode.WebviewViewResolveContext,
    token: vscode.CancellationToken,
  ): Promise<void> {
    this._view = webviewView

    // Reset initial activation flag after first view resolution
    if (isInitialActivation) {
      isInitialActivation = false
    }

    webviewView.webview.options = {
      // Enable JavaScript in the webview
      enableScripts: true,
      // Restrict the webview to only loading content from our extension's directory
      localResourceRoots: [this._extensionUri],
    }

    // Handle messages from the webview
    webviewView.webview.onDidReceiveMessage(
      async (message: { command: string; url?: string }) => {
        switch (message.command) {
          case 'refresh':
            console.log('Webview requested refresh')
            await this.hardRefresh()
            break
          case 'ready':
            console.log('Webview is ready')
            break
        }
      },
      undefined,
      [],
    )

    // Show loading screen
    webviewView.webview.html = this._getLoadingHtml()

    try {
      this._url = await getConfiguredUrl()
      console.log(`Using URL: ${this._url}`)

      webviewView.webview.html = this._getHtmlForWebview(webviewView.webview)
    } catch (error) {
      console.error('Error resolving URL:', error)
      webviewView.webview.html = this._getErrorHtml(error)
    }
  }

  /**
   * Returns the HTML content for the webview with debug info
   */
  private _getHtmlForWebview(webview: vscode.Webview): string {
    // Use the dynamically resolved URL
    const url = this._url

    // Get custom URL from settings if specified
    const config = vscode.workspace.getConfiguration('wandelbots-viewer')
    const customUrl = config.get('customUrl')
    const showDebugInfo = config.get('showDebugInfo', false)

    // If custom URL is provided, use it instead
    const finalUrl = customUrl || url
    console.log(`Final URL: ${finalUrl}`)

    // If URL is missing or invalid, show an error
    if (!finalUrl || finalUrl === 'localhost' || finalUrl === 'undefined') {
      return this._getErrorHtml(new Error('Failed to resolve a valid URL'))
    }

    // Return HTML with an iframe that loads the website
    return `
      <!DOCTYPE html>
      <html lang="en">
      <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Wandelbots Viewer</title>
        <style>
          body, html {
            margin: 0;
            padding: 0;
            height: 100%;
            overflow: hidden;
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, 'Open Sans', 'Helvetica Neue', sans-serif;
          }
          iframe {
            width: 100%;
            height: ${showDebugInfo ? 'calc(100% - 80px)' : '100%'};
            border: none;
          }
          .loader {
            position: absolute;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            text-align: center;
            background: rgba(255, 255, 255, 0.8);
            padding: 20px;
            border-radius: 4px;
          }
          .loading {
            display: block;
            margin-bottom: 10px;
          }
          .spinner {
            border: 4px solid #f3f3f3;
            border-top: 4px solid #3498db;
            border-radius: 50%;
            width: 30px;
            height: 30px;
            animation: spin 2s linear infinite;
            margin: 0 auto;
          }
          @keyframes spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
          }
          .debug-info {
            background: #f3f3f3;
            padding: 10px;
            font-size: 12px;
            font-family: monospace;
            overflow: auto;
            max-height: 80px;
          }
        </style>
      </head>
      <body>
        <div class="loader" id="loader">
          <span class="loading">Loading Wandelbots...</span>
          <div class="spinner"></div>
        </div>
        ${
          showDebugInfo
            ? `<div class="debug-info">
          <strong>URL:</strong> ${finalUrl}<br>
          <strong>ENV:</strong> ${
            process.env.VSCODE_PROXY_URI || 'Not available'
          }<br>
          <button onclick="refreshPage()">Refresh</button>
        </div>`
            : ''
        }
        <iframe src="${finalUrl}" id="mainFrame" onload="handleFrameLoad()" onerror="handleFrameError()"></iframe>
        <script>
          const vscode = acquireVsCodeApi();
          const loader = document.getElementById('loader');
          const mainFrame = document.getElementById('mainFrame');
          let loadTimeoutId;

          function handleFrameLoad() {
            // Clear any timeout
            if (loadTimeoutId) {
              clearTimeout(loadTimeoutId);
            }

            // Hide the loader
            loader.style.display = 'none';
            console.log('Frame loaded successfully');
          }

          function handleFrameError() {
            console.error('Frame failed to load');
            loader.innerHTML = '<span class="loading">Error loading content. <button onclick="refreshPage()">Try Again</button></span>';
          }

          // Set a timeout to detect if the page doesn't load
          loadTimeoutId = setTimeout(() => {
            if (loader.style.display !== 'none') {
              console.log('Frame load timeout');
              loader.innerHTML = '<span class="loading">Loading timed out. <button onclick="refreshPage()">Try Again</button></span>';
            }
          }, 15000);

          function refreshPage() {
            // Show loading state
            if (loader) {
              loader.style.display = 'block';
              loader.innerHTML = '<span class="loading">Refreshing...</span><div class="spinner"></div>';
            }

            // Reload the iframe content
            if (mainFrame) {
              mainFrame.src = mainFrame.src;
            }
          }

          function refreshIframe() {
            console.log('Refreshing iframe content');
            if (mainFrame) {
              const currentSrc = mainFrame.src;
              // Add timestamp to force refresh
              const separator = currentSrc.includes('?') ? '&' : '?';
              mainFrame.src = currentSrc + separator + '_refresh=' + Date.now();
            }
          }

          // Listen for messages from the extension
          window.addEventListener('message', event => {
            const message = event.data;
            switch (message.command) {
              case 'refresh':
                console.log('Received refresh command from extension');
                refreshIframe();
                break;
              case 'hardRefresh':
                console.log('Received hard refresh command from extension');
                window.location.reload();
                break;
              case 'updateUrl':
                console.log('Received URL update command:', message.url);
                if (message.url && mainFrame) {
                  mainFrame.src = message.url;
                }
                break;
            }
          });
        </script>
      </body>
      </html>
    `
  }

  /**
   * Returns a loading screen HTML
   */
  private _getLoadingHtml(): string {
    return `
      <!DOCTYPE html>
      <html lang="en">
      <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Loading...</title>
        <style>
          body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, 'Open Sans', 'Helvetica Neue', sans-serif;
            padding: 20px;
            color: #333;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            height: 100vh;
            margin: 0;
          }
          .loader {
            border: 5px solid #f3f3f3;
            border-top: 5px solid #3498db;
            border-radius: 50%;
            width: 50px;
            height: 50px;
            animation: spin 2s linear infinite;
            margin-bottom: 20px;
          }
          @keyframes spin {
            0% { transform: rotate(0deg); }
            100% { transform: rotate(360deg); }
          }
        </style>
      </head>
      <body>
        <div class="loader"></div>
        <p>Loading Wandelbots viewer...</p>
      </body>
      </html>
    `
  }

  /**
   * Returns an error screen HTML
   */
  private _getErrorHtml(error: unknown): string {
    const message = (error as Error)?.message ?? 'Unknown error'
    return `
      <!DOCTYPE html>
      <html lang="en">
      <head>
        <meta charset="UTF-8">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <title>Error</title>
        <style>
          body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, 'Open Sans', 'Helvetica Neue', sans-serif;
            padding: 20px;
            color: #333;
          }
          .error {
            color: #e74c3c;
            background-color: #fceaea;
            padding: 10px;
            border-left: 4px solid #e74c3c;
            margin-bottom: 20px;
          }
          button {
            background-color: #3498db;
            color: white;
            border: none;
            padding: 8px 16px;
            border-radius: 4px;
            cursor: pointer;
          }
        </style>
      </head>
      <body>
        <h2>Error Loading Wandelbots</h2>
        <div class="error">
          <p>Failed to load Wandelbots viewer: ${message}</p>
        </div>
        <button onclick="window.location.reload()">Try Again</button>
      </body>
      </html>
    `
  }

  /**
   * Returns the HTML content for the React app
   */
  private _getReactAppHtml(webview: vscode.Webview): string {
    try {
      // Get path to index.html in build directory
      const indexPath = vscode.Uri.joinPath(
        this._extensionUri,
        'app',
        'build',
        'index.html',
      )

      // Read the HTML file content
      const fs = require('fs')
      const htmlContent = fs.readFileSync(indexPath.fsPath, 'utf8')

      // Convert any resource URIs to webview URIs
      const buildUri = webview.asWebviewUri(
        vscode.Uri.joinPath(this._extensionUri, 'app', 'build'),
      )

      // Replace paths to be webview-friendly
      const updatedHtml = htmlContent.replace(
        /(href|src)="([^"]*)"/g,
        (match: string, attr: string, path: string) => {
          if (path.startsWith('/')) {
            path = path.slice(1)
          }
          return `${attr}="${buildUri}/${path}"`
        },
      )

      return updatedHtml
    } catch (error) {
      console.error('Error loading React app:', error)
      return this._getErrorHtml(error)
    }
  }
}

/**
 * Setup monitoring for Python script execution
 */
export function setupPythonScriptMonitoring(
  context: vscode.ExtensionContext,
  provider: WandelbotsNovaViewerProvider,
): void {
  // Track active terminal processes and their content
  const activeTerminals = new Map<number, vscode.Terminal>()
  const lastTerminalOutput = new Map<number, string>()

  // Cleaner debounce implementation
  type DebounceFn = () => Promise<void> | void
  interface Debouncer {
    execute: (fn: DebounceFn, reason?: string) => void
    cancel: () => void
  }

  const createDebouncer = (delay: number): Debouncer => {
    let timeoutId: NodeJS.Timeout | null = null

    return {
      execute: (fn: DebounceFn, reason = ''): void => {
        // Clear existing timeout
        if (timeoutId) {
          clearTimeout(timeoutId)
          console.log(`🔄 Refresh timer reset: ${reason}`)
        }

        // Set new timeout
        timeoutId = setTimeout(async () => {
          console.log(`⏰ Executing debounced refresh: ${reason}`)
          try {
            await fn()
          } finally {
            timeoutId = null
          }
        }, delay)

        console.log(`⏳ Refresh scheduled in ${delay}ms: ${reason}`)
      },

      cancel: (): void => {
        if (timeoutId) {
          clearTimeout(timeoutId)
          console.log('🧹 Debounce timer cancelled')
          timeoutId = null
        }
      },
    }
  }

  // Create debouncer with 1 second delay
  const refreshDebouncer = createDebouncer(1000)

  // Cleanup function
  context.subscriptions.push({ dispose: () => refreshDebouncer.cancel() })

  // Listen for terminal creation
  context.subscriptions.push(
    vscode.window.onDidOpenTerminal((terminal) => {
      console.log('Terminal opened:', terminal.name)
      terminal.processId?.then((pid) => {
        if (pid !== undefined) {
          activeTerminals.set(pid, terminal)
        }
      })
    }),
  )

  // Listen for terminal closure
  context.subscriptions.push(
    vscode.window.onDidCloseTerminal((terminal) => {
      console.log('Terminal closed:', terminal.name)
      // Remove by matching terminal instance (processId may not be available here)
      for (const [pid, t] of activeTerminals.entries()) {
        if (t === terminal) {
          activeTerminals.delete(pid)
          lastTerminalOutput.delete(pid)
        }
      }
    }),
  )

  // Monitor nova.rrd file changes for Nova program completion
  console.log('Setting up nova.rrd file monitoring...')

  // Create file watcher for nova.rrd files
  const rrdWatcher = vscode.workspace.createFileSystemWatcher('**/nova.rrd')
  context.subscriptions.push(rrdWatcher)

  // Watch for nova.rrd file changes (indicates Nova program completion)
  context.subscriptions.push(
    rrdWatcher.onDidChange(async (uri) => {
      console.log(`🔄 Nova.rrd file CHANGED: ${uri.fsPath}`)
      console.log('✅ Nova program COMPLETED - scheduling debounced refresh')

      refreshDebouncer.execute(async () => {
        await provider.refreshView()
        await provider.reveal()
        console.log('📱 Viewer refreshed and opened due to nova.rrd change')
      }, 'nova.rrd file changed')
    }),
  )

  // Watch for nova.rrd file creation (first run in workspace)
  context.subscriptions.push(
    rrdWatcher.onDidCreate(async (uri) => {
      console.log(`🆕 Nova.rrd file CREATED: ${uri.fsPath}`)
      console.log(
        '✅ Nova program COMPLETED (first run) - scheduling debounced refresh',
      )

      refreshDebouncer.execute(async () => {
        await provider.refreshView()
        await provider.reveal()
        console.log('📱 Viewer refreshed and opened due to nova.rrd creation')
      }, 'nova.rrd file created')
    }),
  )

  console.log(
    '✅ Nova.rrd file monitoring setup complete - this is the ONLY refresh trigger for Nova programs',
  )

  // Only monitor Nova-specific debug sessions that might not use nova.rrd
  context.subscriptions.push(
    vscode.debug.onDidTerminateDebugSession((session) => {
      if (session.type === 'python') {
        console.log('Python debug session ended')
        const config = session.configuration as vscode.DebugConfiguration & {
          name?: string
        }
        if (config?.name?.includes('Nova Program')) {
          console.log('Nova debug session ended - checking for nova.rrd file')
          // Small delay to allow nova.rrd to be written if this was a Nova program
          setTimeout(async () => {
            const hasNovaRrd = await provider.hasNovaRrdFile()
            if (hasNovaRrd) {
              console.log('Nova.rrd file found - refreshing view')
              await provider.refreshView()
              await provider.reveal()
            } else {
              console.log(
                'No nova.rrd file found - debug session may not have been a Nova program',
              )
            }
          }, 1000)
        }
      }
    }),
  )
}
