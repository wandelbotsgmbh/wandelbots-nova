# TODO: Replace `server_speed_ratio` with real controller time from state stream

## Context

The waypoint jogging server's internal planner consumes waypoints ~9% faster than
wall-clock time (measured ratio: 1.09x on UR10e). We compensate by scaling outgoing
timestamps in `WaypointJoggingSession._make_waypoints_request()` using a static
`server_speed_ratio` configured via `WaypointConfig`.

This works but requires per-robot calibration and can't adapt to runtime variations.

## When

Next week — colleague will publish the actual controller elapsed time in the
jogging state stream response.

## Changes required (~10-15 lines, all in `policy/jogging/waypoint_session.py`)

### 1. Capture controller time from state stream

In `_stream_state()`, add:

```python
if hasattr(state, "controller_time_ms"):
    self._controller_time_ms = state.controller_time_ms
```

### 2. Use real time in timestamp generation

In `_make_waypoints_request()`, replace static ratio with auto-computed one:

```python
# Current:
ratio = self._server_speed_ratio

# New:
if self._controller_time_ms is not None and now_ms > 100:
    ratio = self._controller_time_ms / now_ms
else:
    ratio = self._server_speed_ratio  # fallback for older servers
```

### 3. Optionally expose via `session_elapsed_ms`

So policies can index using the real server clock:

```python
@property
def session_elapsed_ms(self) -> int:
    if self._controller_time_ms is not None:
        return self._controller_time_ms
    return int((time.monotonic() - self._session_start_time) * 1000)
```

### 4. No changes needed in:

- `policy/types.py` — `WaypointConfig.server_speed_ratio` stays as fallback
- `policy/executor.py` — unchanged
- User scripts — unchanged (they send in trajectory time)

## Verification

Run the circular waypoint jogging test with `SPEED_RATIO=1.0` (disabled) and
confirm the auto-computed ratio from the state stream produces equivalent results:

```bash
NOVA_API=http://172.31.11.129 PYTHONPATH=. REVOLUTIONS=3 DURATION=30 \
  CIRCLE_PAUSE_S=0 SPEED_RATIO=1.0 \
  uv run python policy/examples/test_circular_waypoint_jogging.py
```

Expected: mean tracking error < 5mm (same as with `SPEED_RATIO=1.09`).
