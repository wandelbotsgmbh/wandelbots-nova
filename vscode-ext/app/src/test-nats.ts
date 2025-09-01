import { sendNatsMessage, connectToNats, disconnectFromNats } from './nats'

async function testNatsMessages() {
  try {
    console.log('Connecting to NATS...')
    await connectToNats()

    console.log('Sending test messages...')

    // Test forward movement with snap enabled
    await sendNatsMessage('robot.movement', {
      direction: 'forward',
      snap: true,
      timestamp: new Date().toISOString(),
      action: 'start'
    })

    // Test backward movement with snap disabled
    await sendNatsMessage('robot.movement', {
      direction: 'backward',
      snap: false,
      timestamp: new Date().toISOString(),
      action: 'start'
    })

    // Test stop movement
    await sendNatsMessage('robot.movement', {
      direction: null,
      snap: true,
      timestamp: new Date().toISOString(),
      action: 'stop'
    })

    console.log('All test messages sent successfully!')

  } catch (error) {
    console.error('Test failed:', error)
  } finally {
    await disconnectFromNats()
  }
}

// Run the test if this file is executed directly
if (require.main === module) {
  testNatsMessages()
}

export { testNatsMessages }
