# Reference numbers

Measured in a production packaging/assembly cell (NOVA + Siemens PLC via PROFINET bus IO, IO-Link
gripper, 2D vision sensor, force-torque sensor). Use them to size fixes; tune per cell.

| Quantity | Value | Context |
|---|---|---|
| Stop timeout | 5 s | must be the shortest motion timeout |
| Plan timeout | 20 s | |
| Execute timeout | 90 s | longest legitimate motion |
| Session open timeout | 45 s, 3 retries with backoff | |
| Shutdown step / runtime budget | 3 s / 8 s | |
| Supervisor loop | 4–20 ms | bounded by IO transport RTT |
| Dispatcher loop | 20 ms | |
| Home check / telemetry | 20–50 ms | |
| HMI refresh | 500 ms | |
| Home joint tolerance | 0.15 rad | tune per joint |
| First vs warm plan | 4.03 s vs 0.14 s | warm up at startup |
| Event-loop block → NATS reconnect | 100–500 ms | per incident |
| Fixed sleeps removed | ~12 s/cycle | |
| Settle timeout removed | up to 1 s/job | |
| IO polling before cache | >400 req/s | 5 loops on one REST endpoint |
