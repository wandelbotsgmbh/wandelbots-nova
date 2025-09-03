import { NovaClient } from '@wandelbots/nova-js/v1'
import { JoggingPanel as LibraryJoggingPanel } from '@wandelbots/wandelbots-js-react-components'
import React from 'react'

import { accessToken, cellId, novaApi } from '../config'

export default function JoggingPanel({
  motionGroupId,
}: {
  motionGroupId: string
}) {
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
      <LibraryJoggingPanel nova={novaClient} motionGroupId={motionGroupId} />
    </div>
  )
}
