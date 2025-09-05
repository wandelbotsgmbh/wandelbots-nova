import { type MotionStreamConnection } from '@wandelbots/nova-js/v1'
import {
  JoggingPanel as LibraryJoggingPanel,
  PoseCartesianValues,
} from '@wandelbots/wandelbots-js-react-components'
import React, { useEffect, useState } from 'react'

import { useConnectMotionStream, useNovaClient } from '../useNovaClient'

export default function JoggingPanel({
  motionGroupId,
}: {
  motionGroupId: string
}) {
  const [motionStreamConnection, setMotionStreamConnection] =
    useState<MotionStreamConnection | null>(null)

  const novaClient = useNovaClient()

  useEffect(() => {
    const connectToMotionGroup = async () => {
      if (!motionGroupId) {
        return
      }

      try {
        console.log('Connecting to motion group:', motionGroupId)

        const motionStreamConnection =
          await useConnectMotionStream(motionGroupId)

        setMotionStreamConnection(motionStreamConnection)
        // setIsConnected(true)
        console.log('Successfully connected to motion group:', motionGroupId)

        // Log real-time motion state updates
        if (motionStreamConnection.rapidlyChangingMotionState) {
          console.log(
            'Motion group state:',
            motionStreamConnection.rapidlyChangingMotionState.state,
          )
        }
      } catch (error) {
        console.error('Failed to connect to motion group:', error)
        //setSnackbarMessage(`Failed to connect to motion group: ${error}`)
        //setSnackbarOpen(true)
        //setIsConnected(false)
      }
    }

    connectToMotionGroup()

    // Cleanup when selectedMotionGroupId changes
    return () => {
      if (motionStreamConnection) {
        motionStreamConnection.dispose?.()
        setMotionStreamConnection(null)
        // setIsConnected(false)
      }
    }
  }, [motionGroupId])

  function getCurrentPoseString(motionStream: MotionStreamConnection) {
    const tcpPose = motionStream.rapidlyChangingMotionState.tcp_pose
    if (!tcpPose) return 'No Pose'
    return String(tcpPose)
  }

  return (
    <div>
      <LibraryJoggingPanel nova={novaClient} motionGroupId={motionGroupId} />
      {motionStreamConnection ? (
        <div className="mt-8">
          {getCurrentPoseString(motionStreamConnection)}
        </div>
      ) : null}
    </div>
  )
}
