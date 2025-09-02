import { JoggingPanel as LibraryJoggingPanel } from '@wandelbots/wandelbots-js-react-components'
import React, { useMemo } from 'react'

function getConfig() {
  const ls = typeof window !== 'undefined' ? window.localStorage : undefined
  const novaApi = ls?.getItem('wandelbots-nova-viewer.novaApi') || ''
  const cellId = ls?.getItem('wandelbots-nova-viewer.cellId') || 'cell'
  const motionGroupId =
    ls?.getItem('wandelbots-nova-viewer.motionGroupId') || '0@robot'
  return { novaApi, cellId, motionGroupId }
}

export default function JoggingPanel() {
  const { novaApi, motionGroupId } = useMemo(() => getConfig(), [])

  return (
    <LibraryJoggingPanel
      nova={novaApi || 'http://localhost:8000'}
      motionGroupId={motionGroupId}
    />
  )
}
