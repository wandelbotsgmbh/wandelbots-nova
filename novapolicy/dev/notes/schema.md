# Notes: schema reference

These notes were moved out of the customer guide. They keep development history,
measurements and design rationale. The text is verbatim.

## From docs/schema.md, Joint velocities

The controller's measured velocities are used where it sends them (read from the raw state
stream); otherwise they are finite-differenced from consecutive joint positions, which is noisier
and logged once. UR controllers do not send them today; NOVA's state stream is expected to, and
the observation then uses them without changes. The executor starts the measurement with the run;
outside it, wrap `build_observation` in `await schema.start_sources()` / `stop_sources()`.

## From docs/schema.md, Units

NOVA speaks radians and millimetres. A dataset does not necessarily, and a policy trained on
degrees normalizes cleanly against its own statistics before moving the arm to the wrong place —
because those statistics are in the *recording* robot's units.

The inversion happens in the executor **before** relative targets resolve: a delta in degrees added
to a state in radians is exactly the error this prevents.

This is **not** normalization. A policy server applies the checkpoint's own normalization and
unnormalization; duplicating it here would corrupt the values twice over. What is unguarded, and
what these operators are for, is units and scale.

`ops` are distinct from the IO `Mapping` above on purpose: a `Mapping` crosses a *type* boundary
(hardware bools and analogue levels to the policy's floats, and the first step's value back to
hardware), while a `ValueOp` is a float-to-float *units* transform. The schema applies both, so
every policy client gets them the same way.

