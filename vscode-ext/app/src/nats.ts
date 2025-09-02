import { type NatsConnection, wsconnect } from '@nats-io/nats-core'

let nc: NatsConnection | undefined

type Message = {}
export async function connectToNats(): Promise<NatsConnection> {
  if (nc) return nc

  // Use ws:// for local dev (no TLS). Use wss:// with TLS in prod.
  const servers = 'ws://localhost:9222'

  nc = await wsconnect({
    servers,
    name: `webui-${crypto.randomUUID()}`,
  })

  // optional: handle disconnects
  nc.closed().then((err) => {
    if (err) console.error('NATS closed with error:', err)
    nc = undefined
  })

  console.log(`Connected to NATS at ${nc.getServer()}`)
  return nc
}

export async function sendNatsMessage(
  subject: string,
  message: any,
): Promise<void> {
  if (!nc) {
    await connectToNats()
  }

  if (!nc) {
    throw new Error('NATS connection not available')
  }

  try {
    const messageStr = JSON.stringify(message)
    await nc.publish(subject, JSON.stringify(message))
    console.log(`Sent message to ${subject}:`, messageStr)
  } catch (err) {
    console.error('Error sending NATS message:', err)
    throw err
  }
}

export async function disconnectFromNats(): Promise<void> {
  if (nc) {
    await nc.close()
    nc = undefined
    console.log('Disconnected from NATS')
  }
}

export function getNatsConnection(): NatsConnection | undefined {
  return nc
}
