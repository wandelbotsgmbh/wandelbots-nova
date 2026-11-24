# Notes moved out of the README

Material removed from `novapolicy/README.md` when it was rewritten as a customer front door
(usage only). The text is verbatim; only the relative links were changed to resolve from this
folder, and the `▶` bullet marker was dropped.

## From "Policy clients"

> The executor maps and transports: it builds the observations, converts the actions back to NOVA
> and streams them as timestamped action chunks, and it stops a run on an e-stop, a stale input or
> a stop condition. How the arm follows the waypoints is NOVA's job ([contracts](../contracts.md)).

## From "Checking units against the checkpoint" (a limitation)

> One asymmetry worth knowing:
> bounds catch values that are too *large* for the training range — degrees where radians were
> expected. The reverse sits comfortably inside a degree range and cannot be caught this way.

## From "Teleoperation" (a gap, an ADR link and a measurement)

> There's no built-in teleop device, but the standalone jogging layer is the
> building block: feed `set_target(...)` from whatever input you have (a leader
> arm, keyboard, gamepad, spacemouse, a VR controller) in your own script. See
> [docs/jogging.md](../../docs/jogging.md).

> A hand reverses faster than the arm can brake, and a lone live target makes the arm overshoot a
> sudden stop. `ServoHorizon` runs a critically damped spring-damper on the live targets and sends
> its path, braking onto the newest target, as a 400 ms horizon
> ([jogging.md](../../docs/jogging.md#following-a-hand-servohorizon),
> [ADR 0022](../adr/0022-servo-horizon-for-live-targets.md)):

> On a virtual UR10e following a mocked Meta Quest controller, with the client on the NOVA
> instance, it tracked 190 ms behind with 7 mm rms and no overshoot; each pose sent alone set the
> arm ringing by about 60 mm ([`jogging_quest.py`](../../examples/jogging/jogging_quest.py)).

## From "Further reading" (developer docs)

> | [docs/architecture.md](../architecture.md) | How the parts fit: mapping, the send pipeline and its modules, execution modes, adding a policy provider |
> | [docs/contracts.md](../contracts.md) | What the executor promises, as numbered and tested invariants for mapping (M) and timing (T); the reasons in [docs/adr/](../adr/README.md) |

## From "Examples" (an internal checkpoint name)

> [`pick_and_place_umi_ur10e.py`](../../examples/pick_and_place_umi_ur10e.py): the same
> pick-and-place the choreo3 checkpoints learn, written out by hand — a way to prove a cell before
> a policy runs on it
