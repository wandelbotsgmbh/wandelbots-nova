# Testing (TST)

## TST-001 · Jobs runnable standalone against a simulated cell
`severity: info` · `scope: robotics` · `detect: review`

**Rule.** Provide a job runner that executes a single job (with parameters, optional repeats)
against a virtual robot and virtual PLC, presetting the PLC preconditions.

NOVA: `ProgramPreconditions(controllers=[virtual_controller(...)])` on `@nova.program` creates a
virtual controller when the program runs; `nova.run_program(...)` runs one program locally.

---

## TST-002 · Device plugins testable without the orchestrator
`severity: info` · `scope: robotics` · `detect: review`

**Rule.** Each plugin can be initialised, exercised and shut down from a standalone script, using
the same init/shutdown order as production.

---

## TST-003 · Decision logic is pure and unit-tested
`severity: info` · `scope: general` · `detect: static+review`

**Rule.** Job selection, parameter building and precondition checks are pure functions of a
snapshot (`check(snapshot, params) -> bool`), and unit-tested without robot or PLC.
