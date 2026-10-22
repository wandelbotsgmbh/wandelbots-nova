# Async / concurrency (ASY)

## ASY-001 · No blocking calls inside the event loop
`severity: error` · `scope: python-async` · `detect: static`

**Rule.** Inside `async def` (and anything called from it) never use `time.sleep`, `requests`,
blocking sockets, synchronous file IO in loops, `subprocess.run`, or CPU-heavy work. Use
`asyncio.sleep`, async clients (`httpx.AsyncClient`), or `asyncio.to_thread` / an executor.

**Observed.** One gripper helper called `time.sleep` ~50× per command. That stalled the safety
loop, home check and telemetry, and dropped the NATS connection. Each reconnect cost 100–500 ms.

**Detect.** `time.sleep(`, `requests.`, `urllib.request`, `socket.recv`, `open(` inside loops,
`subprocess.run(` in modules that define `async def`; sync helpers called from async code.

NOVA: the SDK's state streams, bus IO waits (`wait_for_bus_io`) and the NATS connection run on
the same event loop as your program.

---

## ASY-002 · Every external await is time-bounded
`severity: error` · `scope: python-async` · `detect: static`

**Rule.** Every await on a network, robot, PLC or device operation has a timeout
(`asyncio.wait_for`, `asyncio.timeout`, or a client-level timeout). Session open gets its own
timeout.

---

## ASY-003 · Exactly one motion consumer per motion group
`severity: error` · `scope: robotics` · `detect: review`

**Rule.** All motion for a motion group goes through one serial consumer (queue + single worker).
Parallel work during a motion (camera trigger, gripper pre-open) runs as side tasks of that motion,
never as a second motion command.

**Detect.** Multiple tasks that can call `execute`/`plan_and_execute` on the same motion group
concurrently; motion issued from GUI callbacks, FastAPI handlers or IO handlers.

---

## ASY-004 · Cancellation is handled, not swallowed
`severity: error` · `scope: python-async` · `detect: static`

**Rule.** Catch `asyncio.CancelledError` only to clean up, then re-raise. Distinguish intentional
cancellation (shutdown, fault pause) from unexpected cancellation, and report only the latter as a
fault.

**Detect.** `except asyncio.CancelledError: pass`; `except BaseException` without re-raise;
`contextlib.suppress(asyncio.CancelledError)` around anything other than awaiting an
already-cancelled task.

NOVA: cancelling the task that runs `execute` / `plan_and_execute` is how motion is stopped. A
swallowed `CancelledError` around motion code hides that stop from the caller.

---

## ASY-005 · Keep references to background tasks
`severity: warning` · `scope: python-async` · `detect: static`

**Rule.** Store every `create_task` result, add a done-callback that logs exceptions, and cancel and
await the task on shutdown. Prefer `asyncio.TaskGroup` where the lifetimes match.

**Detect.** `asyncio.create_task(...)` / `ensure_future(...)` whose result is not assigned or added
to a set.

---

## ASY-006 · Lifecycle transitions are serialised
`severity: warning` · `scope: python-async` · `detect: review`

**Rule.** Protect start, stop, pause-for-fault, resume and shutdown with one async lock so that they
cannot interleave (e.g. fault arriving during startup).

---

## ASY-007 · GUI threads talk to the loop thread-safely
`severity: warning` · `scope: python-async` · `detect: static+review`

**Rule.** GUI toolkits (Tkinter, Qt) run in their own thread. They submit work with
`asyncio.run_coroutine_threadsafe` / `loop.call_soon_threadsafe` and read state through a locked
store. A GUI must never issue motion directly (see ASY-003).

---

## ASY-008 · Safety-relevant loops are never starved
`severity: warning` · `scope: python-async` · `detect: review`

**Rule.** Assign explicit priorities: fault/mode supervision > motion worker > job dispatch >
telemetry > UI. Higher-priority loops must not share awaits with lower ones, and nothing
CPU-heavy runs on the loop. A Python asyncio stack is **soft** real-time; do not claim millisecond
guarantees.

---

## ASY-009 · Shared physical state has one thread-safe owner
`severity: info` · `scope: robotics` · `detect: review`

**Rule.** Physical facts (what is in the gripper, current home pose, last snapshot) live in one
locked state object with explicit setters. No module-level mutable globals scattered across jobs.
