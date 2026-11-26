# Notes: GR00T guide

These notes were moved out of the customer guide. They keep development history,
measurements and design rationale. The text is verbatim.

## From docs/gr00t.md, introduction

The wire format, serialization and array shapes match Isaac GR00T N1.7 (`v1.7.0`,
`novapolicy.providers.gr00t.GROOT_PROTOCOL_VERSION`); N1 and N1.5 servers (`gr00t/eval/service.py`)
take a flat observation and are not supported.

## From docs/gr00t.md, Usage

The client uses `PolicySchema` observations to build GR00T-compatible numpy array observations and decode the returned action arrays.
An action key the server leaves out (one the policy was not trained on) is skipped; a key that is
present without the expected array raises, since that is what a server that changed its array
serialization looks like, and skipping it once left the robot standing still without a word.

## From docs/gr00t.md, step 1

Old wording (the call was not awaited there; `get_server_info` is a coroutine):

`Gr00tPolicyClient(host, port=...).get_server_info()` lists the state, action, video and
language keys and the action horizon.

## From docs/gr00t.md, step 2

The TCP must be known: nothing in the data recovers it unless the training tool rotated a lot.
Rotations pivot about it, so a TCP d mm off shifts the position by about d × angle while rotating.

## From docs/gr00t.md, step 3

If the policy was trained in simulation, replay one training episode's recorded joint path on the
real robot before running the policy (a `jog_joints` chunk of the episode's joints at its rate). A
replay that picks and places proves the robot frame, table and target positions match; whatever
goes wrong afterwards is the policy. It does not check the TCP: joints replay the same path
whatever the TCP is.

