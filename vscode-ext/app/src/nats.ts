import { connect, NatsConnection, StringCodec } from 'nats'

let nc: NatsConnection | null = null
const sc = StringCodec()

type Message = {

}

export async function connectToNats(): Promise<NatsConnection> {
  if (nc) {
    return nc
  }

  try {
    // Try to connect to localhost first, then fallback to demo servers
    const servers = [
      { servers: 'localhost:4222' },
      { servers: ['demo.nats.io:4442', 'demo.nats.io:4222'] },
      { servers: 'demo.nats.io:4443' },
    ]

    for (const config of servers) {
      try {
        nc = await connect(config)
        console.log(`Connected to NATS at ${nc.getServer()}`)
        return nc
      } catch (err) {
        console.log(`Failed to connect to ${JSON.stringify(config)}:`, err)
      }
    }

    throw new Error('Failed to connect to any NATS server')
  } catch (err) {
    console.error('Error connecting to NATS:', err)
    throw err
  }
}

export async function sendNatsMessage(
  subject: string,
  message: any
): Promise<void> {
  if (!nc) {
    await connectToNats()
  }

  if (!nc) {
    throw new Error('NATS connection not available')
  }

  try {
    const messageStr = JSON.stringify(message)
    await nc.publish(subject, sc.encode(messageStr))
    console.log(`Sent message to ${subject}:`, messageStr)
  } catch (err) {
    console.error('Error sending NATS message:', err)
    throw err
  }
}

export async function disconnectFromNats(): Promise<void> {
  if (nc) {
    await nc.close()
    nc = null
    console.log('Disconnected from NATS')
  }
}

export function getNatsConnection(): NatsConnection | null {
  return nc
}
