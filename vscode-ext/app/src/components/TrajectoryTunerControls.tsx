import { Button } from '@mui/material'
import React, { useRef } from 'react'

import { sendNatsMessage } from '../utils/nats'
import { ArrowFirst, ArrowLast, ArrowPlayLeft, ArrowPlayRight } from './icons'

export const TrajectoryTunerControls = ({
  onStart,
  onStop,
  snap,
  speed,
  canMoveBackward = true,
  canMoveForward = true,
}) => {
  const isFinishingRef = useRef(false)

  async function handleButtonClick(event: any, direction: string) {
    // Prevent default behavior and stop propagation
    event?.preventDefault?.()
    event?.stopPropagation?.()

    console.log('Button clicked:', direction, 'Event type:', event?.type)

    try {
      if (direction === 'forward' && !canMoveForward) {
        // When forward is not allowed, send finish event instead
        isFinishingRef.current = true
        console.log('Sending NATS finish command (forward blocked)')
        await sendNatsMessage('trajectory-cursor', {
          command: 'finish',
        })
        return
      }

      const command = snap
        ? direction === 'forward'
          ? 'step-forward'
          : 'step-backward'
        : direction

      console.log(
        'Sending NATS message with movement command and speed',
        command,
        speed,
      )

      // Send NATS message with movement command and speed
      await sendNatsMessage('trajectory-cursor', {
        command,
        speed,
      })

      // Call the original onStart handler
      onStart(direction)
    } catch (error) {
      console.error('Failed to send NATS message:', error)
      // Still call onStart even if NATS fails
      if (!(direction === 'forward' && !canMoveForward)) {
        onStart(direction)
      }
    }
  }

  async function handleButtonRelease() {
    if (isFinishingRef.current) {
      // Skip sending pause after finish trigger
      isFinishingRef.current = false
      return
    }

    try {
      console.log('Sending NATS message to pause movement')

      // Send NATS message to pause movement
      await sendNatsMessage('trajectory-cursor', {
        command: 'pause',
      })

      // Call the original onStop handler
      onStop()
    } catch (error) {
      console.error('Failed to send NATS message:', error)
      // Still call onStop even if NATS fails
      onStop()
    }
  }

  return (
    <div className="flex flex-col items-center gap-4">
      <p className="text-sm text-slate-400">Press and Hold to move</p>
      <div className="space-y-3 flex flex-col items-center justify-center">
        <div className="flex gap-4">
          <div className="flex flex-col items-center gap-1">
            <Button
              onMouseDown={(e) => handleButtonClick(e, 'backward')}
              onMouseUp={handleButtonRelease}
              variant="contained"
              color={canMoveBackward ? 'secondary' : 'primary'}
              disabled={!canMoveBackward}
              sx={{
                opacity: canMoveBackward ? 1 : 0.5,
                cursor: canMoveBackward ? 'pointer' : 'not-allowed',
              }}
            >
              {snap ? (
                <ArrowFirst className="size-8 mx-9 my-3" />
              ) : (
                <ArrowPlayLeft className="size-8 mx-9 my-3" />
              )}
            </Button>
            <div className="text-sm font-medium text-slate-400">bwd</div>
          </div>
          <div className="flex flex-col items-center gap-1">
            <Button
              onMouseDown={(e) => handleButtonClick(e, 'forward')}
              onMouseUp={handleButtonRelease}
              variant="contained"
              color={canMoveForward ? 'secondary' : 'primary'}
              sx={{
                cursor: 'pointer',
              }}
            >
              {snap ? (
                <ArrowLast className="size-8 mx-9 my-3" />
              ) : (
                <ArrowPlayRight className="size-8 mx-9 my-3" />
              )}
            </Button>
            <div className="text-sm font-medium text-slate-400">
              {canMoveForward ? 'fwd' : 'next'}
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}
