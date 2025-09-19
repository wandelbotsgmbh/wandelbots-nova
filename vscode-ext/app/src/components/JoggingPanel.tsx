import type { ConnectedMotionGroup } from '@wandelbots/nova-js/v1'
import type { Joints } from '@wandelbots/nova-js/v1'
import {
  JoggingStore,
  JoggingPanel as LibraryJoggingPanel,
  PoseCartesianValues,
  PoseJointValues,
} from '@wandelbots/wandelbots-js-react-components'
import React, { useEffect, useState } from 'react'
import { runInAction } from "mobx"
import { useLocalObservable, observer } from "mobx-react-lite"

import { useConnectMotionGroup, useNovaClient } from '../useNovaClient'

type JoggingPanelProps = {
  motionGroupId: string
}

const JoggingPanel = observer(({ props }: { props: JoggingPanelProps }) => {
  const [connectedMotionGroup, setConnectedMotionGroup] =
    useState<ConnectedMotionGroup | null>(null)

  const novaClient = useNovaClient()

  useEffect(() => {
    async function connectToMotionGroup() {
      if (!props.motionGroupId) return

      try {
        console.log('Connecting to motion group:', props.motionGroupId)

        const fetchedConnectedMotionGroup =
          await useConnectMotionGroup(props.motionGroupId)

        setConnectedMotionGroup(fetchedConnectedMotionGroup)

        console.log('Connected motion group:', fetchedConnectedMotionGroup)
        console.log(
          'TCP pose:',
          fetchedConnectedMotionGroup?.rapidlyChangingMotionState.tcp_pose
            ?.position,
        )
        // setIsConnected(true)
        console.log('Successfully connected to motion group:', props.motionGroupId)
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
  }, [props.motionGroupId])

  const state = useLocalObservable(() => ({
    joggingStore: null as JoggingStore | null,
  }))

  const joggingPanelFooter = (tabId: string, connectedMotionGroup: ConnectedMotionGroup) => {
    switch (tabId) {
      case 'cartesian':
        return <PoseCartesianValues
        showCopyButton={true}
        tcpPose={(() => {
          const motionState =
            connectedMotionGroup.rapidlyChangingMotionState
          const state = motionState?.state
          const tcpPose = state?.tcp_pose
          return tcpPose
        })()}
      />
      case 'joint':
        return <PoseJointValues
          showCopyButton={true}
          joints={(() => {
            const motionState =
              connectedMotionGroup.rapidlyChangingMotionState
            const state = motionState?.state
            const joints = state?.joint_position

            const pose = joints || ({ joints: [0, 0, 0, 0, 0, 0] } as Joints)
            return pose
          })()}
        />
      default:
        return null
    }
  }

  return (
    <div className="flex flex-col gap-3 items-center justify-center">
      {state.joggingStore && String(state.joggingStore.currentTab.label)}
      <LibraryJoggingPanel
        nova={novaClient}
        motionGroupId={props.motionGroupId}
        onSetup={(store) => runInAction(() => (state.joggingStore = store))}
      />
      {
        connectedMotionGroup && state.joggingStore &&
          <>
            <PoseCartesianValues
              showCopyButton={true}
              tcpPose={(() => {
                const motionState =
                  connectedMotionGroup.rapidlyChangingMotionState
                const state = motionState?.state
                const tcpPose = state?.tcp_pose
                return tcpPose
              })()}
            />
            <PoseJointValues
              showCopyButton={true}
              joints={(() => {
                const motionState =
                  connectedMotionGroup.rapidlyChangingMotionState
                const state = motionState?.state
                const joints = state?.joint_position

                const pose = joints || ({ joints: [0, 0, 0, 0, 0, 0] } as Joints)
                return pose
              })()}
            />
          </>
      }
    </div>
  )
})

export default JoggingPanel
