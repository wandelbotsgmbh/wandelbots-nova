# Trajectory Execution State Machine

This module provides `TrajectoryExecutionMachine`, a finite-state machine that encapsulates the state handling logic for trajectory execution lifecycle, shared across movement controllers (`move_forward`, `TrajectoryCursor`, etc.).

## States

| State | Description |
|-------|-------------|
| `idle` | Initial state — no trajectory active, waiting for `start` |
| `executing` | Robot is moving (`TrajectoryRunning`) |
| `ending` | `TrajectoryEnded` received but robot not yet at standstill |
| `pausing` | `TrajectoryPausedByUser` received, not yet at standstill |
| `paused` | Robot paused and at standstill — may `start` again to resume |
| `completed` | Trajectory finished **and** robot at standstill |
| `error` | Unrecoverable error — terminal state |

## Transitions

### External Commands
- `start` — begin or resume execution (from `idle`, `paused`, or `completed`)
- `fail` — signal an error from any non-terminal state

### Internal Transitions (via `process_motion_state`)
- `TrajectoryRunning` → stay in `executing`
- `TrajectoryEnded` + standstill → `completed`
- `TrajectoryEnded` (no standstill) → `ending` → (on standstill) → `completed`
- `TrajectoryPausedByUser` + standstill → `paused`
- `TrajectoryPausedByUser` (no standstill) → `pausing` → (on standstill) → `paused`

---

## PlantUML Diagram

```plantuml
@startuml TrajectoryExecutionMachine
skinparam state {
    BackgroundColor<<initial>> LightBlue
    BackgroundColor<<final>> LightGray
}

[*] --> idle

state idle <<initial>>
state error <<final>>

idle --> executing : start
paused --> executing : start / resume
completed --> executing : start

executing --> executing : TrajectoryRunning
executing --> completed : TrajectoryEnded\n[standstill]
executing --> ending : TrajectoryEnded\n[!standstill]
executing --> paused : TrajectoryPausedByUser\n[standstill]
executing --> pausing : TrajectoryPausedByUser\n[!standstill]

ending --> ending : [!standstill]
ending --> completed : [standstill]

pausing --> pausing : [!standstill]
pausing --> paused : [standstill]

idle --> error : fail
executing --> error : fail
ending --> error : fail
pausing --> error : fail
paused --> error : fail

error --> [*]

@enduml
```

---

## Mermaid Diagram

```mermaid
stateDiagram-v2
    [*] --> idle

    idle --> executing : start
    paused --> executing : start (resume)
    completed --> executing : start

    executing --> executing : TrajectoryRunning
    executing --> completed : TrajectoryEnded [standstill]
    executing --> ending : TrajectoryEnded [!standstill]
    executing --> paused : TrajectoryPausedByUser [standstill]
    executing --> pausing : TrajectoryPausedByUser [!standstill]

    ending --> ending : [!standstill]
    ending --> completed : [standstill]

    pausing --> pausing : [!standstill]
    pausing --> paused : [standstill]

    idle --> error : fail
    executing --> error : fail
    ending --> error : fail
    pausing --> error : fail
    paused --> error : fail

    error --> [*]

    note right of idle : Initial state
    note right of error : Terminal state
```

---

## Usage Example

```python
machine = TrajectoryExecutionMachine()
machine.send("start")

async for state in motion_group_states:
    result = machine.process_motion_state(state)

    if result.location is not None:
        update_location(result.location)

    if machine.is_completed:
        break
    if machine.is_paused:
        handle_pause()
```
