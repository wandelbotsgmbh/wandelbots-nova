/**
 * Build a NATS connection string from a NOVA API host and access token.
 * - If both novaApi and novaAccessToken are provided:
 *   - Strips http/https and trailing slashes from novaApi
 *   - Uses ws/ws**s** and 80/443 depending on the original scheme
 *   - Returns: `${scheme}://${token}@${cleanHost}:${port}/api/nats`
 * - Otherwise falls back to process.env.NATS_BROKER (or null if unset).
 */
export function buildNatsConnectionString(
  novaApi?: string | null,
  novaAccessToken?: string | null,
  opts?: {
    env?: Record<string, string | undefined>
    logger?: Pick<Console, 'debug' | 'warn'>
  },
): string | null {
  const logger = opts?.logger ?? console

  const host = (novaApi ?? '').trim()
  const token = (novaAccessToken ?? '').trim()

  if (host && token) {
    const isHttp = host.startsWith('http://')
    // Remove protocol and trailing slashes
    const cleanHost = host.replace(/^https?:\/\//, '').replace(/\/+$/, '')
    const scheme = isHttp ? 'ws' : 'wss'
    const port = isHttp ? 80 : 443
    const auth = `${token}@`

    return `${scheme}://${auth}${cleanHost}:${port}/api/nats`
  }

  logger.debug('Host and token not both set')
  return null
}
