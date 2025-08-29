import React, { useEffect, useMemo, useRef, useState } from "react";
import { motion } from "framer-motion";
import {
  Wrench,
  Play,
  ChevronLeft,
  ChevronDown,
  ChevronRight,
  MoveRight,
  GitBranch,
  Cpu,
} from "lucide-react";

/**
 * Single-file React + Tailwind UI inspired by the screenshot.
 * - Dark theme with purple accents
 * - All components live in this file for now (can be split later)
 * - Filled with mock data and light interactivity
 */

/****************************
 * Utility UI Primitives
 ****************************/
const Card = ({ className = "", children }) => (
  <div
    className={
      "rounded-2xl border border-white/10 bg-gradient-to-b from-slate-900/60 to-slate-950/60 shadow-xl " +
      className
    }
  >
    {children}
  </div>
);

const SectionHeader = ({ title, right }) => (
  <div className="flex items-center justify-between gap-3 px-6 py-4">
    <h3 className="text-lg font-semibold tracking-tight text-slate-100">{title}</h3>
    <div className="flex items-center gap-2">{right}</div>
  </div>
);

const Pill = ({ children, tone = "violet" }) => (
  <span
    className={`inline-flex items-center gap-2 rounded-full px-3 py-1 text-sm font-medium bg-${tone}-500/15 text-${tone}-300 border border-${tone}-400/20`}
  >
    {children}
  </span>
);

const IconButton = ({ icon: Icon, label, onClick, className = "" }) => (
  <button
    onClick={onClick}
    className={`group inline-flex h-11 items-center justify-center gap-2 rounded-xl border border-white/10 bg-white/5 px-4 text-sm font-medium text-slate-200 hover:bg-white/10 active:scale-[.98] ${className}`}
  >
    {Icon && <Icon className="h-4 w-4 opacity-80" />}
    <span>{label}</span>
  </button>
);

const PrimaryButton = ({ children, onClick, className = "" }) => (
  <motion.button
    whileTap={{ scale: 0.98 }}
    onClick={onClick}
    className={`inline-flex h-12 items-center justify-center rounded-2xl bg-violet-600 px-5 text-base font-semibold text-white shadow-lg shadow-violet-600/25 hover:bg-violet-500 focus:outline-none ${className}`}
  >
    {children}
  </motion.button>
);

const Toggle = ({ checked, onChange, label }) => (
  <button
    onClick={() => onChange(!checked)}
    className="group flex items-center gap-3 text-slate-200"
  >
    <span
      className={`relative inline-flex h-6 w-11 items-center rounded-full transition-colors duration-200 ${
        checked ? "bg-violet-600" : "bg-slate-700"
      }`}
    >
      <span
        className={`ml-1 inline-block h-4 w-4 transform rounded-full bg-white transition-transform duration-200 ${
          checked ? "translate-x-5" : "translate-x-0"
        }`}
      />
    </span>
    <span className="select-none text-sm font-medium">{label}</span>
  </button>
);

const Range = ({ value, onChange, min = 0, max = 100, step = 1 }) => (
  <input
    type="range"
    min={min}
    max={max}
    step={step}
    value={value}
    onChange={(e) => onChange(Number(e.target.value))}
    className="h-2 w-full appearance-none rounded-full bg-slate-800 accent-violet-500"
  />
);


const StatusPill = ({ state = "Ready", mode = "Auto" }) => (
  <Pill>
    <Cpu className="h-4 w-4" />
    <span>
      {state} <span className="opacity-60">/</span> {mode}
    </span>
  </Pill>
);

const MotionGroupSelector = ({ value, onChange, options }) => {
  const [open, setOpen] = useState(false);
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
                onChange(opt);
                setOpen(false);
              }}
              className={`flex w-full items-center gap-2 px-4 py-2 text-left text-sm hover:bg-white/5 ${
                opt === value ? "text-violet-300" : "text-slate-200"
              }`}
            >
              <Wrench className="h-4 w-4" />
              <span>{opt}</span>
            </button>
          ))}
        </div>
      )}
    </div>
  );
};

const MoveControls = ({ onStart, onStop, moving }) => (
  <div className="flex flex-col items-center gap-4">
    <p className="text-sm text-slate-400">Press and Hold to move</p>
    <div className="flex items-center gap-4">
      <motion.button
        onMouseDown={() => onStart("backward")}
        onMouseUp={onStop}
        onMouseLeave={onStop}
        whileTap={{ scale: 0.98 }}
        className={`group h-20 w-44 rounded-2xl border border-white/10 bg-white/5 text-slate-200 shadow-inner hover:bg-white/10 ${
          moving === "backward" ? "ring-2 ring-violet-500" : ""
        }`}
      >
        <div className="flex items-center justify-center gap-3">
          <ChevronLeft className="h-6 w-6" />
          <span className="text-lg font-medium">backward</span>
        </div>
      </motion.button>
      <motion.button
        onMouseDown={() => onStart("forward")}
        onMouseUp={onStop}
        onMouseLeave={onStop}
        whileTap={{ scale: 0.98 }}
        className={`group h-20 w-44 rounded-2xl border border-white/10 bg-white/5 text-slate-200 shadow-inner hover:bg-white/10 ${
          moving === "forward" ? "ring-2 ring-violet-500" : ""
        }`}
      >
        <div className="flex items-center justify-center gap-3">
          <span className="text-lg font-medium">forward</span>
          <ChevronRight className="h-6 w-6" />
        </div>
      </motion.button>
    </div>
  </div>
);

/****************************
 * Main Panel – Motion Group
 ****************************/
const MotionGroupPanel = () => {
  const [group, setGroup] = useState("UR10e-handling");
  const [state, setState] = useState("Ready");
  const [mode, setMode] = useState("Auto");
  const [speed, setSpeed] = useState(40);
  const [snap, setSnap] = useState(true);
  const [moving, setMoving] = useState(null); // "backward" | "forward" | null
  const [pose, setPose] = useState({ x: 120.2, y: 34.1, z: 512.7, rX: -1.57, rY: 0, rZ: 3.14 });
  const [lastTest, setLastTest] = useState(null);

  // Simulate movement & pose drift while holding a button
  const moveInterval = useRef(null);
  useEffect(() => {
    if (moving) {
      moveInterval.current = setInterval(() => {
        setPose((p) => ({
          ...p,
          z: Number((p.z + (moving === "forward" ? 1 : -1) * speed * 0.1).toFixed(2)),
        }));
      }, 120);
    }
    return () => clearInterval(moveInterval.current);
  }, [moving, speed]);

  const handleStart = (dir) => setMoving(dir);
  const handleStop = () => setMoving(null);

  const speedDisplay = useMemo(() => `v = ${speed} mm/s`, [speed]);

  const runTest = () => {
    // Mock test result
    const ok = Math.random() > 0.12;
    setLastTest({
      ok,
      at: new Date().toLocaleTimeString(),
      note: ok ? "Trajectory validated" : "Joint 3 exceeded threshold",
    });
  };

  return (
    <Card className="p-6">
      {/* Header Row */}
      <div className="flex flex-wrap items-center justify-between gap-3">
        <div className="flex items-center gap-3">
          <MotionGroupSelector
            value={group}
            onChange={setGroup}
            options={["UR10e-handling", "UR10e-welding", "UR5-palletizing"]}
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
          <div className="mt-3 rounded-xl border border-white/10 bg-white/5 p-4">
            <Range value={speed} onChange={setSpeed} min={10} max={120} step={1} />
          </div>
        </div>
        <div className="flex justify-end">
          <div className="rounded-xl border border-white/10 bg-white/5 px-4 py-2 text-sm text-slate-200">
            {speedDisplay}
          </div>
        </div>
      </div>

      {/* Move Controls */}
      <div className="mt-8">
        <MoveControls onStart={handleStart} onStop={handleStop} moving={moving} />
      </div>

      {/* Snap + Actions */}
      <div className="mt-8 flex flex-wrap items-center justify-between gap-4">
        <Toggle checked={snap} onChange={setSnap} label="Snap to point" />

        <div className="flex flex-wrap items-center gap-3">
          <IconButton icon={GitBranch} label="Get current pose" onClick={() => {
            // Mock: nudge pose a little and pretend to fetch
            setPose((p) => ({ ...p, x: Number((p.x + 0.2).toFixed(2)) }));
          }} />
          <IconButton icon={MoveRight} label="Move robot" onClick={() => {
            // Mock action: toggle mode
            setMode((m) => (m === "Auto" ? "Manual" : "Auto"));
          }} className="relative">
            {/* external icon via absolute isn't necessary when label passed */}
          </IconButton>
        </div>
      </div>

      {/* Run Test */}
      <div className="mt-8">
        <PrimaryButton className="w-56" onClick={runTest}>
          <div className="flex items-center gap-3">
            <Play className="h-5 w-5" />
            <span>Run Test</span>
          </div>
        </PrimaryButton>
      </div>
    </Card>
  );
};

/****************************
 * Root App
 ****************************/
export default function FineTuning() {
  return (
    <div className="min-h-screen bg-gradient-to-b from-slate-950 to-slate-900 text-slate-100">
      <div className="flex">
        <div className="relative flex-1">
          <main className="mx-auto max-w-5xl p-6">
            <div className="space-y-6">
              <section>
                <SectionHeader title="Motion Group" />
                <MotionGroupPanel />
              </section>
            </div>
          </main>
        </div>
      </div>
    </div>
  );
}
