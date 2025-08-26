import * as vscode from 'vscode'

import { SETTINGS_NOVA_API, SETTINGS_RERUN_ADDRESS, VIEWER_ID } from './consts.js'

/**
 * Gets the protocol from the environment
 * @returns The protocol ("http://" or "https://")
 */
export function getProtocolFromEnvironment(): 'http://' | 'https://' {
  try {
    // Prefer VSCODE_PROXY_URI which reflects the actual protocol used
    const proxyUri = process.env.VSCODE_PROXY_URI
    if (proxyUri && proxyUri.includes('://')) {
      // Only allow http/https; fall back to https otherwise
      const m = proxyUri.match(/^(https?):\/\//i)
      if (m) {
        return m[1].toLowerCase() === 'http' ? 'http://' : 'https://'
      }
    }

    // Check VS Code's URI scheme
    const scheme = vscode.env.uriScheme
    if (scheme === 'https' || scheme === 'vscode-https') return 'https://'
    if (scheme === 'http' || scheme === 'vscode-http') return 'http://'

    // Codespaces are always HTTPS
    if (process.env.CODESPACE_NAME) return 'https://'

    // Default to https
    return 'https://'
  } catch (error) {
    console.error('Error determining protocol:', error)
    return 'https://'
  }
}

/**
 * Gets the host address using TypeScript implementation of Python logic
 * @returns The host address (e.g., "https://example.instance.wandelbots.io" or "http://localhost")
 */
export function getNovaApiAddress(): string {
  try {
    // Configured instance?
    const config = vscode.workspace.getConfiguration(VIEWER_ID)
    const novaApi = config.get<string>(SETTINGS_NOVA_API, '')

    if (novaApi) {
      const protocol = getProtocolFromEnvironment()
      console.log(`Using configured Wandelbots NOVA API address: ${novaApi}`)
      return `${protocol}${novaApi}`
    }

    // VS Code proxy env (e.g., remote/server scenarios)
    const proxyUri = process.env.VSCODE_PROXY_URI
    if (proxyUri && proxyUri.includes('://')) {
      // Avoid URL parsing (proxy examples may include placeholders)
      const [protocolRaw, rest] = proxyUri.split('://', 2)
      const parts = rest.split('/')
      const host = parts[0]

      // Wandelbots instance through proxy
      if (host.includes('.instance.wandelbots.io')) {
        return `${protocolRaw}://${host}`
      }

      // Extract "cell" name preceding "proxy"
      if (parts.length >= 3) {
        const proxyIndex = parts.indexOf('proxy')
        if (proxyIndex >= 2) {
          const cellName = parts[proxyIndex - 2]
          return `${protocolRaw}://${host}/${cellName}`
        }
      }

      // Fallback: just protocol + host
      return `${protocolRaw}://${host}`
    }

    // GitHub Codespaces
    if (
      process.env.CODESPACE_NAME &&
      process.env.GITHUB_CODESPACES_PORT_FORWARDING_DOMAIN
    ) {
      const port = vscode.env.uriScheme === 'codespace' ? '8000' : '8080'
      return `https://${process.env.CODESPACE_NAME}-${port}.${
        process.env.GITHUB_CODESPACES_PORT_FORWARDING_DOMAIN
      }`
    }

    // Local fallback
    const protocol = getProtocolFromEnvironment()
    return `${protocol}localhost`
  } catch (error) {
    console.error('Error in getNovaApiAddress:', error)
    const protocol = getProtocolFromEnvironment()
    return `${protocol}localhost`
  }
}

/**
 * Gets the rerun address based on the host address
 * @returns The rerun address or null if using localhost
 */
export function getRerunAddress(): string | null {
  const host = getNovaApiAddress()
  if (host && !host.includes('localhost')) {
    if (host.includes('.instance.wandelbots.io')) {
      // Wandelbots instance
      return `${host}/cell/visual-studio-code/rerun/?url=${host}/cell/visual-studio-code/nova.rrd`
    }
    // Generic host
    return `${host}/visual-studio-code/rerun/?url=${host}/visual-studio-code/nova.rrd`
  }
  return null
}

/**
 * Gets the configured URL based on settings and environment
 * @returns The configured URL
 */
export async function getConfiguredUrl(): Promise<string> {
  try {
    // Diagnostics
    console.log('VS Code Env Info:')
    console.log('- VSCODE_PROXY_URI:', process.env.VSCODE_PROXY_URI)
    console.log('- CODESPACE_NAME:', process.env.CODESPACE_NAME)
    console.log('- VS Code URI Scheme:', vscode.env.uriScheme)

    const novaApiAddress = getNovaApiAddress()
    const rerunAddress = getRerunAddress()

    console.log('Resolved addresses:')
    console.log('- NOVA API Address:', novaApiAddress)
    console.log('- Rerun Address:', rerunAddress)

    const config = vscode.workspace.getConfiguration(VIEWER_ID)
    const urlType = config.get<
      typeof SETTINGS_NOVA_API | typeof SETTINGS_RERUN_ADDRESS
    >('urlType', SETTINGS_RERUN_ADDRESS)

    console.log('Configuration:')
    console.log('- URL Type:', urlType)

    // Use rerunAddress or hostAddress depending on settings
    if (urlType === SETTINGS_RERUN_ADDRESS && rerunAddress) {
      console.log('Using rerun address:', rerunAddress)
      return rerunAddress
    } else if (
      urlType === SETTINGS_NOVA_API &&
      novaApiAddress &&
      !novaApiAddress.includes('localhost')
    ) {
      const hostUrl = `${novaApiAddress}/visual-studio-code`
      console.log('Using host address:', hostUrl)
      return hostUrl
    } else if (
      urlType === SETTINGS_RERUN_ADDRESS &&
      novaApiAddress.includes('localhost')
    ) {
      // For localhost, provide a local rerun server URL
      console.log('Using local rerun address')
      return 'http://localhost:9090'
    } else {
      console.log('Using default address: https://wandelbots.com')
      return 'https://wandelbots.com'
    }
  } catch (error) {
    console.error('Error in getConfiguredUrl:', error)
    return 'https://wandelbots.com'
  }
}
