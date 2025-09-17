type MotionEventType = 'STARTED' | 'STOPPED'

type UnknownRecord = Record<string, unknown>

export interface ActionMessage {
  metas?: {
    line_number?: number
  } & UnknownRecord
  // other action fields are intentionally not modeled here
  [key: string]: unknown
}

export interface MotionEventMessage {
  type: MotionEventType
  current_location: number
  current_action: ActionMessage
  target_location: number
  target_action: ActionMessage
  // allow unknown extra fields to avoid being brittle to backend changes
  [key: string]: unknown
}
