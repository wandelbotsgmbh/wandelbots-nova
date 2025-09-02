import { connect } from 'nats'

const servers = [
  {},
  { servers: ['demo.nats.io:4442', 'demo.nats.io:4222'] },
  { servers: 'demo.nats.io:4443' },
  { port: 4222 },
  { servers: 'localhost' },
]

export async function connectToNats() {
  await servers.forEach(async (v) => {
    try {
      const nc = await connect(v)
      console.log(`connected to ${nc.getServer()}`)
      // this promise indicates the client closed
      const done = nc.closed()
      // do something with the connection

      // close the connection
      await nc.close()
      // check if the close was OK
      const err = await done
      if (err) {
        console.log(`error closing:`, err)
      }
    } catch (err) {
      console.log(`error connecting to ${JSON.stringify(v)}`)
    }
  })
}
