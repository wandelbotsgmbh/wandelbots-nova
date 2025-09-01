import Button from '@mui/material/Button'
import FormControlLabel from '@mui/material/FormControlLabel'
import FormGroup from '@mui/material/FormGroup'
import Switch from '@mui/material/Switch'
import ToggleButton from '@mui/material/ToggleButton'
import ToggleButtonGroup from '@mui/material/ToggleButtonGroup'
import { VelocitySlider } from '@wandelbots/wandelbots-js-react-components'
import {
  ChevronDown,
  ChevronLeft,
  ChevronRight,
  Cpu,
  GitBranch,
  Play,
  Wrench,
} from 'lucide-react'
import React, { useEffect, useMemo, useRef, useState } from 'react'
import { sendNatsMessage } from './nats'

const SectionHeader = ({ title, right }) => (
  <div className="flex items-center justify-between gap-3 px-6 py-4">
    <h3 className="text-lg font-semibold tracking-tight text-slate-100">
      {title}
    </h3>
    <div className="flex items-center gap-2">{right}</div>
  </div>
)

const Pill = ({ children, tone = 'violet' }) => (
  <span
    className={`inline-flex items-center gap-2 rounded-full px-3 py-1 text-sm font-medium bg-${tone}-500/15 text-${tone}-300 border border-${tone}-400/20`}
  >
    {children}
  </span>
)

const IconButton = ({ icon: Icon, label, onClick, className = '' }) => (
  <Button onClick={onClick}>
    {Icon && <Icon className="h-4 w-4 opacity-80" />}
    <span>{label}</span>
  </Button>
)

const Range = ({ value, onChange, min = 0, max = 100, step = 1 }) => (
  <VelocitySlider
    velocity={value}
    onVelocityChange={onChange}
    min={min}
    max={max}
    store={{
      showTabIcons: false,
      showVelocityLegend: false,
      showVelocitySliderLabel: true,
    }}
  />
)

const StatusPill = ({ state = 'Ready', mode = 'Auto' }) => (
  <Pill>
    <Cpu className="h-4 w-4" />
    <span>
      {state} <span className="opacity-60">/</span> {mode}
    </span>
  </Pill>
)

const MotionGroupSelector = ({ value, onChange, options }) => {
  const [open, setOpen] = useState(false)
  return (
    <div className="relative">
      <button
        onClick={() => setOpen((o) => !o)}
        className="inline-flex items-center gap-2 rounded-xl border border-white/10 bg-white/5 px-3 py-2 text-sm text-slate-200 hover:bg-white/10"
      >
        <Wrench className="h-4 w-4" />
        <span>{value}</span>
        <ChevronDown className="h-4 w-4 opacity-70" />
      </button>
      {open && (
        <div className="absolute z-20 mt-2 w-56 overflow-hidden rounded-xl border border-white/10 bg-slate-900/95 shadow-lg backdrop-blur">
          {options.map((opt) => (
            <button
              key={opt}
              onClick={() => {
                onChange(opt)
                setOpen(false)
              }}
              className={`flex w-full items-center gap-2 px-4 py-2 text-left text-sm hover:bg-white/5 ${
                opt === value ? 'text-violet-300' : 'text-slate-200'
              }`}
            >
              <Wrench className="h-4 w-4" />
              <span>{opt}</span>
            </button>
          ))}
        </div>
      )}
    </div>
  )
}

const MoveControls = ({ onStart, onStop, moving, snap }) => {
  const handleButtonClick = async (event: any, direction: string) => {
    try {
      // Send NATS message with movement direction and snap setting
      await sendNatsMessage('robot.movement', {
        direction,
        snap,
        timestamp: new Date().toISOString(),
        action: 'start'
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
      // Send NATS message to stop movement
      await sendNatsMessage('robot.movement', {
        direction: null,
        snap,
        timestamp: new Date().toISOString(),
        action: 'stop'
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
        <ToggleButtonGroup
          color="primary"
          value={moving}
          exclusive
          onChange={handleButtonClick}
          onMouseLeave={handleButtonRelease}
          onTouchEnd={handleButtonRelease}
          aria-label="Platform"
        >
          <ToggleButton value="backward" size="large">
            <ChevronLeft className="size-12 mx-6 my-3" />
          </ToggleButton>
          <ToggleButton value="forward" size="large">
            <ChevronRight className="size-12 mx-6 my-3" />
          </ToggleButton>
        </ToggleButtonGroup>
        <div className="flex items-center justify-center gap-6 text-slate-400">
          <span className="text-base font-medium">backward</span>
          <span className="text-base font-medium">forward</span>
        </div>
      </div>
    </div>
  )
}

/****************************
 * Main Panel – Motion Group
 ****************************/
const MotionGroupPanel = () => {
  const [group, setGroup] = useState('UR10e-handling')
  const [state, setState] = useState('Ready')
  const [mode, setMode] = useState('Auto')
  const [speed, setSpeed] = useState(40)
  const [snap, setSnap] = useState(true)
  const [moving, setMoving] = useState(null) // "backward" | "forward" | null
  const [pose, setPose] = useState({
    x: 120.2,
    y: 34.1,
    z: 512.7,
    rX: -1.57,
    rY: 0,
    rZ: 3.14,
  })
  const [lastTest, setLastTest] = useState(null)

  const handleSnapChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setSnap(event.target.checked)
  }

  // Simulate movement & pose drift while holding a button
  const moveInterval = useRef(null)
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
    return () => clearInterval(moveInterval.current)
  }, [moving, speed])

  const handleStart = (dir) => setMoving(dir)
  const handleStop = () => setMoving(null)

  const speedDisplay = useMemo(() => `v = ${speed} mm/s`, [speed])

  const runTest = () => {
    // Mock test result
    const ok = Math.random() > 0.12
    setLastTest({
      ok,
      at: new Date().toLocaleTimeString(),
      note: ok ? 'Trajectory validated' : 'Joint 3 exceeded threshold',
    })
  }

  return (
    <div className="px-3 py-6">
      {/* Header Row */}
      <div className="flex flex-wrap items-center justify-between gap-3">
        <div className="flex items-center gap-3">
          <MotionGroupSelector
            value={group}
            onChange={setGroup}
            options={['UR10e-handling', 'UR10e-welding', 'UR5-palletizing']}
          />
        </div>
        <StatusPill state={state} mode={mode} />
      </div>

      {/* Robot Name */}
      <div className="mt-6 border-t border-white/10 pt-6">
        <div>
          <h2 className="text-2xl font-semibold text-slate-100">UR10e</h2>
          <p className="text-sm text-slate-400">0@ur10</p>
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
        />
      </div>

      {/* Snap + Actions */}
      <div className="mt-8 flex flex-col items-center justify-between gap-4">
        <div className="flex flex-col justify-center items-center gap-3">
          <Button
            color="secondary"
            variant="contained"
            className="w-full"
            onClick={() => {
              // Mock: nudge pose a little and pretend to fetch
              setPose((p) => ({ ...p, x: Number((p.x + 0.2).toFixed(2)) }))
            }}
          >
            <GitBranch className="h-4 w-4" />
            <span className="ml-2">Get current pose</span>
          </Button>
          <Button
            color="secondary"
            variant="contained"
            className="w-full"
            onClick={() => {
              // Mock action: toggle mode
              setMode((m) => (m === 'Auto' ? 'Manual' : 'Auto'))
            }}
          >
            <GitBranch className="h-4 w-4" />
            <span className="ml-2">Move robot</span>
          </Button>
        </div>

        <div className="mt-8">
          <Button variant="contained" onClick={runTest} className="w-56">
            <Play className="h-4 w-4" />
            <span className="ml-2">Run Test</span>
          </Button>
        </div>
      </div>
    </div>
  )
}

/****************************
 * Root App
 ****************************/
export default function FineTuning() {
  return (
    <div className="min-h-screen">
      <div className="flex">
        <div className="relative flex-1">
          <main className="mx-auto max-w-5xl p-6">
            <div className="space-y-6">
              <section>
                <SectionHeader title="Motion Group" right={null} />
                <MotionGroupPanel />
              </section>
            </div>
          </main>
        </div>
      </div>
    </div>
  )
}
