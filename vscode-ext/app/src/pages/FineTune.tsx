import Box from '@mui/material/Box'
import Breadcrumbs from '@mui/material/Breadcrumbs'
import Button from '@mui/material/Button'
import FormControlLabel from '@mui/material/FormControlLabel'
import FormGroup from '@mui/material/FormGroup'
import Link from '@mui/material/Link'
import Modal from '@mui/material/Modal'
import Switch from '@mui/material/Switch'
import Typography from '@mui/material/Typography'
import {
  VelocitySlider,
  VelocitySliderLabel,
} from '@wandelbots/wandelbots-js-react-components'
import { ChevronRight, Route, SplinePointer } from 'lucide-react'
import React, { useEffect, useState } from 'react'

import { SectionCard } from '../components'
import { TrajectoryTunerControls } from '../components'
import JoggingPanel from '../components/JoggingPanel'
import MotionGroupSelection from '../components/MotionGroupSelection'
import { accessToken, cellId, natsBroker, novaApi } from '../config'
import {
  connectToNats,
  disconnectFromNats,
  sendNatsMessage,
  subscribeToNatsMessage,
} from '../utils/nats'
import { NovaApi } from '../utils/novaAPI'

export default function FineTuning() {
  const [speed, setSpeed] = useState(40)
  const [snap, setSnap] = useState(true)
  const [selectedMotionGroupId, setSelectedMotionGroupId] = useState<
    string | null
  >(null)
  const [movementOptions, setMovementOptions] = useState<string[]>([])
  const [unsubscribeMovementOptions, setUnsubscribeMovementOptions] = useState<
    (() => void) | null
  >(null)
  const [isJoggingOpen, setIsJoggingOpen] = useState(false)

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

  const handleStart = (dir: 'backward' | 'forward') => {}
  const handleStop = () => {}

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
    } catch (error) {
      console.error('Failed to get current pose:', error)
    }
  }

  const handleOpenJogging = () => setIsJoggingOpen(true)
  const handleCloseJogging = () => setIsJoggingOpen(false)

  // Check if forward movement is allowed
  const canMoveBackward = movementOptions.includes('can_move_backward')
  const canMoveForward = movementOptions.includes('can_move_forward')

  function handleMotionGroupChange(motionGroupId: string) {
    setSelectedMotionGroupId(motionGroupId)
  }

  function formatVelocitySliderLabel(value: number) {
    return `${value} mm/s`
    /*if (currentMotionType === "translate") {
      return `v = ${t("Jogging.Cartesian.Translation.velocityMmPerSec.lb", { amount: value })}`
    } else {
      return `ω = ${t("Jogging.Cartesian.Rotation.velocityDegPerSec.lb", { amount: value })}`
    }*/
  }

  return (
    <div className="h-full">
      <div className="flex h-full">
        <div className="relative flex-1">
          <main className="mx-auto max-w-xl">
            <div className="flex flex-col gap-3">
              {/*<SectionCard
                subheader="Debug info"
                color="secondary"
                className="text-xs"
              >
                <div>{novaApi}</div>
                <div>{cellId}</div>
                <div>{accessToken}</div>
                <div>{natsBroker}</div>
              </SectionCard>*/}
              {/* Motion Group Selection */}
              <SectionCard subheader="Fine Tuning" color="secondary">
                <div className="flex-1 flex justify-between items-center">
                  <p className="text-sm font-medium text-slate-300 flex-1">
                    Motion Group
                  </p>
                  <div className="flex-1">
                    <MotionGroupSelection onChange={handleMotionGroupChange} />
                  </div>
                </div>
              </SectionCard>
              {/* Execution Speed */}
              <SectionCard color="secondary">
                <div>
                  <p className="text-sm font-medium text-slate-300">
                    Execution Speed
                  </p>
                  <div className="mt-3 rounded-xl">
                    <VelocitySlider
                      velocity={speed}
                      onVelocityChange={setSpeed}
                      renderValue={(value) => (
                        <VelocitySliderLabel
                          value={formatVelocitySliderLabel(value)}
                          sx={{
                            minWidth: '111px',
                            span: {
                              transform: 'translateY(-1.5px)',
                            },
                          }}
                        />
                      )}
                      min={1}
                      max={250}
                      store={
                        {
                          showTabIcons: false,
                          showVelocitySliderLabel: true,
                          showVelocitySliderLegend: true,
                        } as any
                      }
                    />
                  </div>
                </div>
              </SectionCard>
              {/* Move Controls */}
              <SectionCard color="secondary">
                <TrajectoryTunerControls
                  onStart={handleStart}
                  onStop={handleStop}
                  snap={snap}
                  speed={speed}
                  canMoveBackward={canMoveBackward}
                  canMoveForward={canMoveForward}
                />
                <div className="mt-3">
                  <FormGroup>
                    <FormControlLabel
                      control={
                        <Switch checked={snap} onChange={handleSnapChange} />
                      }
                      label="Snap to point"
                    />
                  </FormGroup>
                </div>
              </SectionCard>
              {/* Actions */}
              <SectionCard color="secondary">
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
                  </div>
                </div>
              </SectionCard>
              <Modal
                open={isJoggingOpen && selectedMotionGroupId !== null}
                onClose={handleCloseJogging}
                aria-labelledby="jogging-modal-title"
                aria-describedby="jogging-modal-description"
              >
                <Box
                  sx={{
                    position: 'fixed',
                    top: 0,
                    left: 0,
                    right: 0,
                    bottom: 0,
                    width: '100vw',
                    height: '100vh',
                    maxWidth: '100vw',
                    maxHeight: '100vh',
                    bgcolor: 'background.paper',
                    boxShadow: 24,
                    p: 3,
                    borderRadius: 0,
                    overflow: 'auto',
                  }}
                >
                  <div className="flex items-center justify-between mb-3">
                    <div
                      id="jogging-modal-title"
                      className="text-lg font-semibold"
                    >
                      <Breadcrumbs
                        aria-label="breadcrumb"
                        separator={<ChevronRight className="h-4 w-4" />}
                      >
                        <Link
                          color="inherit"
                          underline="hover"
                          onClick={handleCloseJogging}
                          sx={{ cursor: 'pointer' }}
                        >
                          Fine-Tuning
                        </Link>
                        <Typography color="text.primary">
                          Jogging Panel
                        </Typography>
                      </Breadcrumbs>
                    </div>
                  </div>
                  <div id="jogging-modal-description">
                    <JoggingPanel motionGroupId={selectedMotionGroupId!} />
                  </div>
                </Box>
              </Modal>
            </div>
          </main>
        </div>
      </div>
    </div>
  )
}
