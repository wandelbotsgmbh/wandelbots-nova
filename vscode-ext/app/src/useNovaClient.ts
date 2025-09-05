import { type MotionStreamConnection, NovaClient } from '@wandelbots/nova-js/v1'

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

export async function useConnectMotionStream(
  motionGroupId: string,
): Promise<MotionStreamConnection> {
  const novaClient = useNovaClient()
  return await novaClient.connectMotionStream(motionGroupId)
}
