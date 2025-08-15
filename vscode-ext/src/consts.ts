export const VIEWER_ID = 'wandelbots-nova-viewer'
export const EXPLORER_ID = 'wandelbots-nova-explorer'
export const COMMAND_PREFIX = 'wandelbots-nova'
export const COMMAND_RUN_NOVA_PROGRAM = `${COMMAND_PREFIX}.runNovaProgram`
export const COMMAND_DEBUG_NOVA_PROGRAM = `${COMMAND_PREFIX}.debugNovaProgram`
export const COMMAND_REFRESH_CODE_LENS = `${COMMAND_PREFIX}.refreshCodeLens`
export const COMMAND_OPEN_NOVA_VIEWER = `${COMMAND_PREFIX}.open`
export const COMMAND_REFRESH_NOVA_VIEWER = `${COMMAND_PREFIX}.refresh`

export const URL_TYPE_NOVA_API = 'novaApi' as const
export const URL_TYPE_RERUN = 'rerunAddress' as const
export const URL_TYPE_DEFAULT = 'default' as const
