import Box from '@mui/material/Box'
import Button from '@mui/material/Button'
import FormControlLabel from '@mui/material/FormControlLabel'
import FormGroup from '@mui/material/FormGroup'
import Modal from '@mui/material/Modal'
import Snackbar from '@mui/material/Snackbar'
import Switch from '@mui/material/Switch'
import { VelocitySlider } from '@wandelbots/wandelbots-js-react-components'
import {
  ChevronFirst,
  ChevronLast,
  ChevronLeft,
  ChevronRight,
  GitBranch,
  Play,
  Route,
  SplinePointer,
  X,
} from 'lucide-react'
import React, { useEffect, useRef, useState } from 'react'

import MotionGroupSelection from '../components/MotionGroupSelection'
import { accessToken, cellId, natsBroker, novaApi } from '../config'
import {
  connectToNats,
  disconnectFromNats,
  sendNatsMessage,
  subscribeToNatsMessage,
} from '../utils/nats'
import { NovaApi } from '../utils/novaAPI'
import JoggingPanel from './JoggingPanel'

const Range = ({ value, onChange, min = 0, max = 100, step = 1 }) => (
  <VelocitySlider
    velocity={value}
    onVelocityChange={onChange}
    min={min}
    max={max}
    store={{} as any}
  />
)

const MoveControls = ({
  onStart,
  onStop,
  moving,
  snap,
  speed,
  canMoveBackward = true,
  canMoveForward = true,
}) => {
  const handleButtonClick = async (event: any, direction: string) => {
    // Prevent default behavior and stop propagation
    event?.preventDefault?.()
    event?.stopPropagation?.()

    console.log('Button clicked:', direction, 'Event type:', event?.type)

    try {
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
      onStart(direction)
    }
  }

  const handleButtonRelease = async () => {
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
          <Button
            onMouseDown={(e) => handleButtonClick(e, 'backward')}
            onMouseUp={handleButtonRelease}
            variant="contained"
            color="secondary"
            disabled={!canMoveBackward}
            sx={{
              opacity: canMoveBackward ? 1 : 0.5,
              cursor: canMoveBackward ? 'pointer' : 'not-allowed',
            }}
          >
            {snap ? (
              <ChevronFirst className="size-8 mx-9 my-3" />
            ) : (
              <ChevronLeft className="size-8 mx-9 my-3" />
            )}
          </Button>
          <Button
            onMouseDown={(e) => handleButtonClick(e, 'forward')}
            onMouseUp={handleButtonRelease}
            variant="contained"
            color="secondary"
            disabled={!canMoveForward}
            sx={{
              opacity: canMoveForward ? 1 : 0.5,
              cursor: canMoveForward ? 'pointer' : 'not-allowed',
            }}
          >
            {snap ? (
              <ChevronLast className="size-8 mx-9 my-3" />
            ) : (
              <ChevronRight className="size-8 mx-9 my-3" />
            )}
          </Button>
        </div>
        <div className="grid grid-cols-2 gap-9 text-slate-400">
          <span className="text-sm font-medium">backward</span>
          <span className="text-sm font-medium">forward</span>
        </div>
      </div>
    </div>
  )
}

/****************************
 * Main Panel – Motion Group
 ****************************/
const MotionGroupPanel = () => {
  const [speed, setSpeed] = useState(40)
  const [snap, setSnap] = useState(true)
  const [moving, setMoving] = useState<'backward' | 'forward' | null>(null)
  const [pose, setPose] = useState({
    x: 120.2,
    y: 34.1,
    z: 512.7,
    rX: -1.57,
    rY: 0,
    rZ: 3.14,
  })
  const [selectedMotionGroupId, setSelectedMotionGroupId] = useState<
    string | null
  >(null)
  const [snackbarOpen, setSnackbarOpen] = useState(false)
  const [snackbarMessage, setSnackbarMessage] = useState('')
  const [movementOptions, setMovementOptions] = useState<string[]>([])
  const [unsubscribeMovementOptions, setUnsubscribeMovementOptions] = useState<
    (() => void) | null
  >(null)

  // Connect to NATS and fetch motion groups when component mounts
  useEffect(() => {
    const connectNats = async () => {
      try {
        await connectToNats()
        console.log('Connected to NATS successfully')
      } catch (error) {
        console.error('Failed to connect to NATS:', error)
      }
    }

    const setupNatsSubscriptions = async () => {
      try {
        // Subscribe to movement options
        const unsubscribe = await subscribeToNatsMessage(
          'editor.movement.options',
          (message) => {
            console.log('Received movement options:', message)
            if (message && Array.isArray(message.options)) {
              setMovementOptions(message.options)
            }
          },
        )
        setUnsubscribeMovementOptions(() => unsubscribe)
      } catch (error) {
        console.error('Failed to subscribe to movement options:', error)
      }
    }

    connectNats()
    setupNatsSubscriptions()

    // Cleanup: disconnect when component unmounts
    return () => {
      disconnectFromNats().catch(console.error)
      if (unsubscribeMovementOptions) {
        unsubscribeMovementOptions()
      }
    }
  }, [])

  const handleSnapChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSnap(event.target.checked)
  }

  // Simulate movement & pose drift while holding a button
  const moveInterval = useRef<number | null>(null)
  useEffect(() => {
    if (moving) {
      moveInterval.current = setInterval(() => {
        setPose((p) => ({
          ...p,
          z: Number(
            (p.z + (moving === 'forward' ? 1 : -1) * speed * 0.1).toFixed(2),
          ),
        }))
      }, 120)
    }
    return () => clearInterval(moveInterval.current ?? 0)
  }, [moving, speed])

  const handleStart = (dir: 'backward' | 'forward') => setMoving(dir)
  const handleStop = () => setMoving(null)

  const runTest = () => {
    const ok = Math.random() > 0.12
    console.log('Test result:', ok)
  }

  async function getCurrentPose() {
    if (!selectedMotionGroupId) return

    try {
      const nova = new NovaApi()
      await nova.connect({
        apiUrl: novaApi,
        accessToken,
        cellId,
      })

      const controller = selectedMotionGroupId.split('@')[1]
      if (!controller) {
        console.error('Controller not found')
        return
      }

      const pose = await nova.getRobotPose(controller, selectedMotionGroupId)

      // Format pose as Pose((x, y, z, rx, ry, rz))
      const poseString = `Pose((${pose.x}, ${pose.y}, ${pose.z}, ${pose.rx}, ${pose.ry}, ${pose.rz}))`

      // Copy to clipboard
      await navigator.clipboard.writeText(poseString)
      console.log('Pose copied to clipboard:', poseString)

      // Show success message
      setSnackbarMessage('Pose saved to clipboard!')
      setSnackbarOpen(true)
    } catch (error) {
      console.error('Failed to get current pose:', error)
    }
  }

  async function finishTrajectoryTuning() {
    try {
      console.log('Sending NATS finish command')
      await sendNatsMessage('trajectory-cursor', {
        command: 'finish',
      })

      // Show snackbar message
      setSnackbarMessage('Continue program run')
      setSnackbarOpen(true)
    } catch (error) {
      console.error('Failed to send finish command:', error)
    }
  }

  const [isJoggingOpen, setIsJoggingOpen] = useState(false)

  const handleOpenJogging = () => setIsJoggingOpen(true)
  const handleCloseJogging = () => setIsJoggingOpen(false)

  // Check if forward movement is allowed
  const canMoveBackward = movementOptions.includes('can_move_backward')
  const canMoveForward = movementOptions.includes('can_move_forward')

  function handleMotionGroupChange(motionGroupId: string) {
    setSelectedMotionGroupId(motionGroupId)
  }

  return (
    <div className="px-3 py-6">
      {/* Motion Group Selection */}
      <div className="mt-2 flex items-center justify-between gap-4">
        <div className="flex-1">
          <p className="text-sm font-medium text-slate-300">Motion Group</p>
          <div className="mt-2">
            <MotionGroupSelection onChange={handleMotionGroupChange} />
          </div>
        </div>
      </div>
      {/* Execution Speed */}
      <div className="mt-8 grid grid-cols-1 items-center gap-4 md:grid-cols-[1fr_auto]">
        <div>
          <p className="text-sm font-medium text-slate-300">Execution Speed</p>
          <div className="mt-3 rounded-xl">
            <Range
              value={speed}
              onChange={setSpeed}
              min={1}
              max={100}
              step={1}
            />
          </div>
        </div>
      </div>

      <div className="mt-3">
        <FormGroup>
          <FormControlLabel
            control={<Switch checked={snap} onChange={handleSnapChange} />}
            label="Snap to point"
          />
        </FormGroup>
      </div>

      {/* Move Controls */}
      <div className="mt-8">
        <MoveControls
          onStart={handleStart}
          onStop={handleStop}
          moving={moving}
          snap={snap}
          speed={speed}
          canMoveBackward={canMoveBackward}
          canMoveForward={canMoveForward}
        />
      </div>

      {/* Snap + Actions */}
      <div className="mt-8 flex flex-col items-center justify-between gap-4">
        <div className="flex flex-col justify-center items-center gap-3">
          <Button
            color="secondary"
            variant="contained"
            className="w-full"
            disabled={selectedMotionGroupId === null}
            onClick={getCurrentPose}
          >
            <SplinePointer className="h-4 w-4" />
            <span className="ml-2">Get current pose</span>
          </Button>
          <Button
            color="secondary"
            variant="contained"
            className="w-full"
            disabled={selectedMotionGroupId === null}
            onClick={handleOpenJogging}
          >
            <Route className="h-4 w-4" />
            <span className="ml-2">Move robot</span>
          </Button>
          <Button
            color="secondary"
            variant="contained"
            className="w-full"
            onClick={finishTrajectoryTuning}
          >
            <GitBranch className="h-4 w-4" />
            <span className="ml-2">Finish Trajectory</span>
          </Button>
        </div>

        {/*<div className="mt-8">
          <Button variant="contained" onClick={runTest} className="w-56">
            <Play className="h-4 w-4" />
            <span className="ml-2">Run Test</span>
          </Button>
        </div>*/}
      </div>

      <Modal
        open={isJoggingOpen && selectedMotionGroupId !== null}
        onClose={handleCloseJogging}
        aria-labelledby="jogging-modal-title"
        aria-describedby="jogging-modal-description"
      >
        <Box
          sx={{
            position: 'absolute',
            top: '50%',
            left: '50%',
            transform: 'translate(-50%, -50%)',
            width: 720,
            maxWidth: '95vw',
            bgcolor: 'background.paper',
            boxShadow: 24,
            p: 3,
            borderRadius: 2,
            maxHeight: '90vh',
            overflow: 'auto',
          }}
        >
          <div className="flex items-center justify-between mb-3">
            <div id="jogging-modal-title" className="text-lg font-semibold">
              Jogging Panel
            </div>
            <Button
              onClick={handleCloseJogging}
              sx={{
                minWidth: 'auto',
                p: 1,
                color: 'text.secondary',
                '&:hover': {
                  backgroundColor: 'action.hover',
                },
              }}
            >
              <X className="h-5 w-5" />
            </Button>
          </div>
          <div id="jogging-modal-description">
            <JoggingPanel motionGroupId={selectedMotionGroupId!} />
          </div>
        </Box>
      </Modal>

      <Snackbar
        open={snackbarOpen}
        autoHideDuration={3000}
        onClose={() => setSnackbarOpen(false)}
        message={snackbarMessage}
        anchorOrigin={{ vertical: 'bottom', horizontal: 'center' }}
      />
    </div>
  )
}

export default function FineTuning() {
  return (
    <div className="min-h-screen">
      <div className="flex">
        <div className="relative flex-1">
          <main className="mx-auto max-w-xl">
            <div className="space-y-6">
              <section>
                <div className="text-sm font-medium text-slate-300">
                  Debug info
                </div>
                <div>{novaApi}</div>
                <div>{cellId}</div>
                <div>{accessToken}</div>
                <div>{natsBroker}</div>
              </section>
              <section>
                <MotionGroupPanel />
              </section>
            </div>
          </main>
        </div>
      </div>
    </div>
  )
}
