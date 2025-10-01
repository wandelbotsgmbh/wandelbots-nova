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
  const natsBroker =
    injected.natsBroker ||
    ls?.getItem('wandelbots-nova-viewer.natsBroker') ||
    'nats://localhost:4222'
  return { novaApi, cellId, accessToken, natsBroker }
}

export const { novaApi, cellId, accessToken, natsBroker } = getConfig()
