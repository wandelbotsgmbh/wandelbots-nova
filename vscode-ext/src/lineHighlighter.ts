import {
  type NatsConnection,
  StringCodec,
  type Subscription,
  connect,
} from 'nats'
import * as vscode from 'vscode'

/** Internal NATS state */
let nc: NatsConnection | undefined
let sub: Subscription | undefined

/** Remember last requested line so we can apply it when an editor becomes active. */
let pendingLine: number | null = null

/** One decoration type we reuse so only one line is highlighted at any time. */
const lineHighlightDecoration = vscode.window.createTextEditorDecorationType({
  isWholeLine: true,
  // Use theme selection colors so it always looks consistent.
  backgroundColor: new vscode.ThemeColor('editor.selectionBackground'),
  border: '1px solid',
  borderColor: new vscode.ThemeColor('editor.selectionHighlightBorder'),
})

/** Options for starting the subscriber. */
export type NatsLineSubscriberOptions = {
  servers: string | string[]
  subject: string
  /** client name visible in NATS monitoring (optional) */
  name?: string
}

/** Establish NATS connection (fresh mechanism using the `nats` Node client). */
async function ensureConnection(
  opts: NatsLineSubscriberOptions,
): Promise<NatsConnection> {
  if (nc) return nc
  nc = await connect({
    servers: opts.servers,
    name:
      opts.name ?? `vscode-highlighter-${Math.random().toString(16).slice(2)}`,
  })

  // Log/cleanup on close
  nc.closed().then((err) => {
    if (err) console.error('[NATS] closed with error:', err)
    nc = undefined
  })

  console.log('[NATS] connected to', nc.getServer())
  return nc
}

/** Parse incoming text for a `line: <number>` hint. Accepts simple variants. */
function parseLineNumber(raw: string): number | null {
  // Accept: "line: 42", "Line : 10", "42", {"line": 7}
  // 1) Try JSON with { line: number }
  try {
    const obj = JSON.parse(raw)
    if (obj && typeof obj.line === 'number' && Number.isFinite(obj.line)) {
      return obj.line
    }
  } catch {
    // ignore
  }
  // 2) Try "line: <number>" pattern
  const m = /line\s*:\s*(\d+)/i.exec(raw)
  if (m) return Number(m[1])
  // 3) Try plain integer body
  const n = Number(raw.trim())
  if (Number.isFinite(n)) return n
  return null
}

/** Select and reveal the (0-based) line in the active editor; ensure only one selection. */
function selectLineInActiveEditor(zeroBasedLine: number) {
  const editor = vscode.window.activeTextEditor
  if (!editor) return

  const doc = editor.document
  if (doc.lineCount === 0) return

  const clamped = Math.max(0, Math.min(zeroBasedLine, doc.lineCount - 1))
  const lineRange = doc.lineAt(clamped).range

  // Set a single selection spanning exactly that line
  editor.selection = new vscode.Selection(lineRange.start, lineRange.end)
  editor.revealRange(
    lineRange,
    vscode.TextEditorRevealType.InCenterIfOutsideViewport,
  )

  // Apply highlight with our single reusable decoration type — this replaces the old range.
  editor.setDecorations(lineHighlightDecoration, [lineRange])
}

/** Apply a pending line when/if an editor becomes available. */
function applyPendingIfPossible() {
  if (pendingLine === null) return
  if (!vscode.window.activeTextEditor) return
  selectLineInActiveEditor(pendingLine)
  pendingLine = null
}

/** Starts a NATS subscription that selects a single line in the active editor. */
export async function startNatsLineSubscriber(
  context: vscode.ExtensionContext,
  opts: NatsLineSubscriberOptions,
): Promise<void> {
  // Be idempotent across reloads
  await stopNatsLineSubscriber()

  const conn = await ensureConnection(opts)
  const sc = StringCodec()

  sub = conn.subscribe(opts.subject)
  ;(async () => {
    for await (const msg of sub!) {
      const body = sc.decode(msg.data)
      const ln = parseLineNumber(body)
      if (ln === null) {
        console.warn('[NATS] ignoring message without line number:', body)
        continue
      }
      // Convert to 0-based if a 1-based line index is sent (most human messages are 1-based)
      const zeroBased = ln > 0 ? ln - 1 : 0

      // If there is an editor, select immediately; otherwise remember for next time
      if (vscode.window.activeTextEditor) {
        selectLineInActiveEditor(zeroBased)
      } else {
        pendingLine = zeroBased
      }
    }
  })().catch((err) => console.error('[NATS] subscriber error:', err))

  // If user switches editors after a message arrived, apply the last pending line
  context.subscriptions.push(
    vscode.window.onDidChangeActiveTextEditor(() => applyPendingIfPossible()),
    vscode.workspace.onDidOpenTextDocument(() => applyPendingIfPossible()),
  )
}

/** Stops the subscription and closes the connection. */
export async function stopNatsLineSubscriber(): Promise<void> {
  try {
    if (sub) {
      sub.unsubscribe()
      sub = undefined
    }
    // drain waits for all messages/acks then closes
    if (nc) {
      await nc.drain().catch(async () => {
        // If drain fails (e.g., already closed), attempt hard close
        try {
          await nc!.close()
        } catch {
          /* ignore */
        }
      })
      nc = undefined
    }
  } catch (err) {
    console.error('[NATS] error on shutdown:', err)
  }
}
