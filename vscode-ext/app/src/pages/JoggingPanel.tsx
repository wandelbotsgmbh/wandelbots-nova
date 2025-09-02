import { NovaClient } from '@wandelbots/nova-js/v1'
import { JoggingPanel as LibraryJoggingPanel } from '@wandelbots/wandelbots-js-react-components'
import React, { useMemo } from 'react'

function getConfig() {
  const ls = typeof window !== 'undefined' ? window.localStorage : undefined
  const injected =
    (typeof window !== 'undefined' && (window as any).__NOVA_CONFIG__) || {}
  const novaApi =
    injected.novaApi || ls?.getItem('wandelbots-nova-viewer.novaApi') || ''
  const cellId =
    injected.cellId || ls?.getItem('wandelbots-nova-viewer.cellId') || 'cell'
  const accessToken =
    injected.accessToken ||
    ls?.getItem('wandelbots-nova-viewer.accessToken') ||
    ''
  const motionGroupId =
    ls?.getItem('wandelbots-nova-viewer.motionGroupId') || '0@robot'
  return { novaApi, cellId, accessToken, motionGroupId }
}

export default function JoggingPanel() {
  const { novaApi, cellId, accessToken, motionGroupId } = useMemo(
    () => getConfig(),
    [],
  )

  const novaClient = new NovaClient({
    instanceUrl: novaApi || 'http://localhost',
    cellId,
    accessToken,
  })

  return (
    <div>
      <div>{novaApi}</div>
      <div>{cellId}</div>
      <div>{accessToken}</div>
      <div>{motionGroupId}</div>
      <LibraryJoggingPanel nova={novaClient} motionGroupId={motionGroupId} />
    </div>
  )
}
