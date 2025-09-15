import * as vscode from 'vscode'

const channel = vscode.window.createOutputChannel('Wandelbots NOVA', {
  log: true,
})

type Level = 'trace' | 'debug' | 'info' | 'warn' | 'error'

function serialize(v: unknown): string {
  if (v instanceof Error) return v.stack ?? `${v.name}: ${v.message}`
  if (typeof v === 'string') return v
  try {
    return JSON.stringify(v)
  } catch {
    return String(v)
  }
}

function write(level: Level, scope: string | undefined, parts: unknown[]) {
  const msg = (scope ? `[${scope}] ` : '') + parts.map(serialize).join(' ')
  switch (level) {
    case 'trace':
      channel.trace(msg)
      break
    case 'debug':
      channel.debug(msg)
      break
    case 'info':
      channel.info(msg)
      break
    case 'warn':
      channel.warn(msg)
      break
    case 'error':
      channel.error(msg)
      break
  }

  // Mirror to console only when the global log level is Debug or lower
  if (vscode.env.logLevel <= vscode.LogLevel.Debug) {
    const c =
      level === 'error'
        ? console.error
        : level === 'warn'
          ? console.warn
          : level === 'info'
            ? console.info
            : level === 'debug'
              ? console.debug
              : console.log
    c(`[Wandelbots NOVA] ${msg}`)
  }
}

export const logger = {
  trace: (...a: unknown[]) => write('trace', undefined, a),
  debug: (...a: unknown[]) => write('debug', undefined, a),
  info: (...a: unknown[]) => write('info', undefined, a),
  warn: (...a: unknown[]) => write('warn', undefined, a),
  error: (...a: unknown[]) => write('error', undefined, a),
  show: (preserveFocus = true) => channel.show(preserveFocus),
  channel,
  child(scope: string) {
    return {
      trace: (...a: unknown[]) => write('trace', scope, a),
      debug: (...a: unknown[]) => write('debug', scope, a),
      info: (...a: unknown[]) => write('info', scope, a),
      warn: (...a: unknown[]) => write('warn', scope, a),
      error: (...a: unknown[]) => write('error', scope, a),
      show: (preserveFocus = true) => channel.show(preserveFocus),
    }
  },
}
