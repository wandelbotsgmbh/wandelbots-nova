import { type ConnectedMotionGroup, NovaClient } from '@wandelbots/nova-js/v1'

import { accessToken, cellId, novaApi } from './config'

export function useNovaClient(): NovaClient {
  try {
    return new NovaClient({
      instanceUrl: novaApi,
      cellId: cellId,
      accessToken: accessToken,
    })
  } catch (error) {
    console.error('Failed to initialize NovaClient:', error)
    throw error
  }
}

export async function useConnectMotionGroup(
  motionGroupId: string,
): Promise<ConnectedMotionGroup> {
  const novaClient = useNovaClient()
  return await novaClient.connectMotionGroup(motionGroupId)
}
