# Notes: LeRobot provider guide

These notes were moved out of the customer guide. They keep development history,
measurements and design rationale. The text is verbatim.

## From providers/lerobot/README.md, install

If you run the LeRobot server in a separate Python environment, install LeRobot there too. LeRobot
currently requires Python 3.12:

```bash
mamba create -y -n lerobot-server python=3.12 pip
conda activate lerobot-server
python -m pip install --upgrade pip
python -m pip install 'wandelbots-nova[novapolicy-lerobot]'
```

Why the guide changed it: the core package pins `pillow>=13.0.0` while LeRobot 0.6 caps Pillow
below 13, so pip cannot resolve this extra (see `[tool.uv] override-dependencies` in the root
`pyproject.toml`). The server only needs `lerobot[async]`.

## From providers/lerobot/README.md, cell setup

The LeRobot examples here run on a UR10e carrying the UMI gripper, whose
`umi_corrected` TCP is the frame the demonstrations were recorded in — not the cell's `Flange`,
221.7 mm further down the tool. Fed the wrong frame a policy does not fail, it stalls in a hover.

## From providers/lerobot/README.md, server start

Keep this process running. A successful startup logs that the gRPC server is listening on port 8080.
The first client connection sends `policy_type`, checkpoint path, action-chunk size, and inference
device. Loading happens while `PolicyExecutor` is in `CONNECTING`, before its execution timeout
starts.

## From providers/lerobot/README.md, `control_freq`

This is separate from the LeRobot server's optional `--fps` CLI flag. The server flag is used by
LeRobot when it creates `TimedAction` timestamps; `LeRobotPolicyClient` does not use those
timestamps. It decodes the returned action tensors into NOVA `ActionChunk`s and sets the chunk
`dt_ms` from this client-side `control_freq`. `PolicyExecutor` then uses that `dt_ms` when scheduling the
returned chunk.

## From providers/lerobot/README.md, checkpoint execution settings

Values are read through LeRobot's own `PreTrainedConfig`, not by parsing `config.json` field by
field, so **every LeRobot policy works without a per-policy table here**. Policies disagree about
what the action chunk is called — ACT says `chunk_size`, diffusion says `horizon`, fastwam says
`action_horizon`, vqbet names it nothing — but every policy config must implement the abstract
`action_delta_indices` property, and its length is the chunk length for all of them. A new or
renamed LeRobot policy needs no change in novapolicy.

The client also holds the schema against the checkpoint's `input_features` / `output_features`
before the setup message goes out: the observation key set, the shape behind each key, and the
width of the flat action vector. A mismatch fails at startup rather than after the robot has
started moving. This runs automatically; there is nothing to call. A checkpoint that declares no
features (they were inferred from the dataset at train time) is warned about and skipped. Only
`STATE` and `VISUAL` inputs are compared: a checkpoint may declare `LANGUAGE` or `ENV` features a
schema never emits, and demanding them would fail every task-conditioned checkpoint.

Settings are resolved once, since `connect` may run again on a reconnect, and LeRobot is imported
only then: its policy registry costs seconds and loads torch. Only a path to a `config.json` maps to
its directory; any other file would silently read the `config.json` beside it. A checkpoint the
client cannot read, or whose config needs packages this environment lacks, falls back to the
explicit arguments; one that LeRobot rejects raises, without draccus' exception type.

**Derivation needs a checkpoint the client can read.** If `pretrained_name_or_path` names a path
that exists only on the inference server, the NOVA client cannot inspect it — LeRobot's async RPC
has no checkpoint-metadata method. Pass `config_path` pointing at a client-local copy of the
checkpoint directory or its `config.json`:

Without one, the client warns that neither derivation nor validation could run and falls back to
the explicit arguments; `policy_type` and `actions_per_chunk` then both have to be supplied. The
client never assumes a policy type it could not read — sending the wrong one makes the server build
the wrong policy class.

## From providers/lerobot/README.md, `actions_per_chunk`

The LeRobot async server does not infer this. It is part of LeRobot's `RemotePolicyConfig`, and the
server slices `policy.predict_action_chunk(...)` to the requested length.

## From providers/lerobot/README.md, Settled ACT chunks

This prevents the policy observation from being captured while the robot is still moving and avoids
executing the lower-confidence tail beyond the checkpoint's configured execution horizon. The
`SequentialExecution` automatically waits for exact NOVA standstill and, if ACT's first target is
farther away than the spacing inside its executed horizon, prepends a
same-`dt_ms` interpolated bridge to one continuous motion request. IO and computed actions fire when
NOVA's server clock reaches policy waypoint zero; the
robot does not stop at that boundary. Endpoint interpolation allocates additional same-`dt_ms`
intervals for acceleration and braking of each settled request. See
[`docs/executor.md`](../../docs/executor.md#bridging-a-distant-first-waypoint).

## From providers/lerobot/README.md, Continuous execution

`AVERAGE` measured lower peak path curvature on physical UR3 plug-task runs and a simulated
UR10e pick-and-place. The merged plan is smoothed by the executor's default pipeline (`Smoothing`)
after its targets are resolved to NOVA units, and the steps already sent stay exact; for tasks
that need sharp contact transitions, pass the defaults without it:

## From providers/lerobot/README.md, Protocol notes

The LeRobot async server flow, implemented inside `LeRobotPolicyClient`, is:

1. `Ready` when the policy client connects
2. `SendPolicyInstructions` during the optional policy `prepare(...)` step, while the executor is
   still in `CONNECTING`
3. `SendObservations` with one observation per `get_actions` call (`must_go=True`), numbered by
   the action steps received so far
4. `GetActions` right after it, in the same call; its `TimedAction`s become one `ActionChunk`

The client keeps no queue of its own: overlapping chunks are merged by `QueuedPolicyClient` (see
[continuous execution](../../providers/lerobot/README.md#continuous-execution-with-queuedpolicyclient)).

`PolicyExecutor` triggers this indirectly by calling the `PolicyClient` methods; it does not send
LeRobot RPCs itself. The executor switches to `EXECUTING` after preparation, so
readiness/model-loading time is excluded from `timeout_s`.

`SendPolicyInstructions` contains a pickled LeRobot `RemotePolicyConfig` with:

- `policy_type`
- `pretrained_name_or_path`
- `lerobot_features`
- `actions_per_chunk`
- `device`
- `rename_map` (left empty by this client)

The server does not expose model metadata before setup, which is why the client reads the
checkpoint itself. Keep `control_freq` (the dataset's `fps`) in your deployment configuration next to the checkpoint path, and
make sure the client can reach a copy of `config.json` — see
[Checkpoint execution settings](../../providers/lerobot/README.md#checkpoint-execution-settings).
