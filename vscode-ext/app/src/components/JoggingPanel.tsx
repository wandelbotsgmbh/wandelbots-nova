import type { ConnectedMotionGroup } from '@wandelbots/nova-js/v1'
import {
  JoggingPanel as LibraryJoggingPanel,
  PoseCartesianValues,
  PoseJointValues,
} from '@wandelbots/wandelbots-js-react-components'
import type { Joints } from '@wandelbots/nova-js/v1'
import React, { useEffect, useState } from 'react'

import { useConnectMotionGroup, useNovaClient } from '../useNovaClient'

export default function JoggingPanel({
  motionGroupId,
}: {
  motionGroupId: string
}) {
  const [connectedMotionGroup, setConnectedMotionGroup] =
    useState<ConnectedMotionGroup | null>(null)

  const novaClient = useNovaClient()

  useEffect(() => {
    async function connectToMotionGroup() {
      if (!motionGroupId) {
        return
      }

      try {
        console.log('Connecting to motion group:', motionGroupId)

        const fetchedConnectedMotionGroup =
          await useConnectMotionGroup(motionGroupId)

        setConnectedMotionGroup(fetchedConnectedMotionGroup)

        console.log('Connected motion group:', fetchedConnectedMotionGroup)
        console.log('TCP pose:', fetchedConnectedMotionGroup?.rapidlyChangingMotionState.tcp_pose?.position)
        // setIsConnected(true)
        console.log('Successfully connected to motion group:', motionGroupId)
      } catch (error) {
        console.error('Failed to connect to motion group:', error)
        //setSnackbarMessage(`Failed to connect to motion group: ${error}`)
        //setSnackbarOpen(true)
        //setIsConnected(false)
      }
    }

    connectToMotionGroup()

    return () => {
      if (connectedMotionGroup) {
        connectedMotionGroup.dispose?.()
        setConnectedMotionGroup(null)
        // setIsConnected(false)
      }
    }
  }, [motionGroupId])

  return (
    <div className="flex flex-col gap-3 items-center justify-center">
      <LibraryJoggingPanel nova={novaClient} motionGroupId={motionGroupId} />
      {
        connectedMotionGroup && (
          <>
          <PoseCartesianValues
            showCopyButton={true}
            tcpPose={(() => {
              const motionState = connectedMotionGroup.rapidlyChangingMotionState
              const state = motionState?.state
              const tcpPose = state?.tcp_pose

              const pose = tcpPose || {
                tcp: "TCP1",
                position: { x: 0, y: 0, z: 0 },
                orientation: { x: 0, y: 0, z: 0 },
              }
              return pose
            })()}
          />
          <PoseJointValues
            showCopyButton={true}
            joints={(() => {
              const motionState = connectedMotionGroup.rapidlyChangingMotionState
              const state = motionState?.state
              const joints = state?.joint_position

              const pose = joints || ({ joints: [0, 0, 0, 0, 0, 0] } as Joints)
              return pose
            })()}
          />
          </>
        )
      }
    </div>
  )
}
