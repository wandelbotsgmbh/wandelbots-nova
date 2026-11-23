# Executor contracts

What `PolicyExecutor` promises, as numbered invariants: **M** for mapping (robot ↔ policy),
**T** for timing (when each waypoint, IO and hook happens). The executor maps and transports;
how the arm follows the waypoints is NOVA's job (see `AGENTS.md`). Each invariant names the test
that asserts it; one marked *untested* is a promise without a guard yet. The reasons behind the
choices are in the [decision records](adr/).

## Clocks and units

| Name | What it is | In code |
| --- | --- | --- |
| Session clock | ms since NOVA started executing the session's first chunk (`jogger_session_timestamp_ms`). Every waypoint timestamp is one. | `ServerMs` (`jogging/clock.py`) |
| Client clock | `time.monotonic()` of this process: run timeout, tick grid, inference timeout | `float` seconds |
| State timestamp | the session-clock time a state packet was measured at; arrives late and in bursts | `current_state_server_ms` |

Session-clock values are estimated on the client by `JoggingTimeClock`: the last acknowledged
timestamp plus the client time since that state was measured, never rescaled
(`test_server_time_is_never_rescaled_by_a_derived_rate`,
`test_delayed_state_uses_generation_time_not_receipt_time`). Placeholder state timestamps from
before 2000 are ignored (`test_a_placeholder_state_timestamp_is_not_read_as_delivery_delay`).

Units at NOVA's side: joints in rad, TCP positions in mm, orientations as rotation vectors (rad),
in world coordinates. Everything the policy sees is converted from these and back.

## M: Mapping

**M1 Operators run both ways.** A channel's `ops` convert NOVA → policy on observations, front
to back, and are inverted policy → NOVA on actions, back to front. An explicit `Action`'s ops
win over its observation's. Every client applies them; a NATS policy takes NOVA units in the
layout it declares, so ops on its schema are refused.
Tests: `test_operator_lists_reverse_on_the_way_back`, `test_joint_observations_are_converted_on_the_way_in`,
`test_joint_actions_are_converted_on_the_way_out`, `test_an_explicit_action_overrides_the_observations_operators`,
`test_joint_observations_apply_the_schema_operators`, `test_validate_schema_rejects_joint_operators`.

**M2 Joint observations** concatenate the source groups' joints in list order, keyed
`<key>_1..n`. Tests: `test_dual_arm_observation`, `test_concatenated_observation`.

**M3 TCP observations** are converted into the declared `coordinate_system`, then to the
declared `orientation_type` (quaternions scalar-last), then through `position_ops` and
`orientation_ops`. Tests: `test_quaternion_observations_are_named_and_converted`,
`test_coordinate_systems_chain_their_references`, `test_world_is_an_exact_no_op`,
`test_tcp_position_and_orientation_convert_independently`.

**M4 Action resolution order:** inverse ops, then relative targets, then frame to world.
Tests: `test_units_are_inverted_before_relative_deltas_resolve`,
`test_absolute_targets_reach_the_robot_in_nova_units`,
`test_relative_targets_compose_in_the_coordinate_system_and_return_to_world`.

**M5 Relative targets are relative to the observation** the policy saw (the snapshot before
inference), not to the live state or the previous target. Joint deltas accumulate: step `i` is
`q_obs + Σ d[0..i]`. TCP rotations compose as rotations. Tests:
`test_relative_joint_steps_are_the_running_cumulative_sum`, `test_relative_tcp_rotations_compose_instead_of_adding`,
`test_accumulated_tcp_deltas_chain_from_step_to_step`, `test_tool_frame_tcp_deltas_follow_the_tool_axes`.

**M6 Trimming.** A chunk is cut to `n_action_steps` (argument, else the policy's own, else all)
after resolution; IO and metadata are kept. Tests: `test_trim_bounds_each_arm_to_n_and_preserves_a_prefix`,
`test_the_policys_declared_horizon_is_used_when_none_is_given`.

**M7 Joint limits.** With `JointClamp` in the pipeline (the executor's default), joint targets are
held inside NOVA's limits minus 2° before they are sent, bridges included; the first clip per
joint is logged. Tests: `test_a_target_below_the_bound_is_held_at_it`,
`test_the_first_clip_per_joint_is_reported`, `test_joint_clamp_holds_targets_inside_the_limits`,
`test_the_session_reads_joint_limits_only_for_a_joint_clamp`; the wire capture's
`out_of_limits` scenario.

**M8 IO values** in `ActionChunk.ios` are in hardware units and written once per change. A
client that receives IO for every step of a chunk takes the first step's, since IO fires when
that step is due (T7). Tests: `test_io_action_decoding`, `test_bool_mapping`,
`test_io_actions_come_from_the_chunks_first_step`.

**M9 Joint velocities** are the controller's measurement when it sends one, otherwise
differenced from consecutive positions. Tests: `test_measured_velocities_are_used_when_the_controller_sends_them`,
`test_without_a_measurement_positions_are_finite_differenced`.

**M10 Stale inputs.** A camera frame older than its `max_age_s`, or an inference past
`inference_timeout_s`, ends the run with `RuntimeError`. Tests: `test_a_stale_camera_aborts_by_default`,
`test_an_inference_deadline_ends_the_run`.

**M11 Step spacing.** `dt_ms` comes from the policy: the server's value where it reports one
(GR00T), otherwise the client's `control_freq` (30 Hz by default). A chunk with `resolved=True`
is already in NOVA units and is sent as it is. Tests: `test_a_callback_control_freq_fills_chunks_without_step_spacing`,
`test_control_freq_defaults_to_30_hz_and_says_so`, `test_a_resolved_chunk_is_sent_without_resolving_it_again`.

## T: Timing

**T1 One placement rule.** Step `k` of a chunk is due
`reference + max_expected_latency_ms + k·dt` on the session clock. The reference is the
observation in `ContinuousExecution` and the send in `SequentialExecution`, where the robot was
at rest when it was observed.
Tests: `test_a_lead_chunk_of_several_steps_is_dated_by_its_observation`,
`test_a_lead_chunk_dated_from_the_send_keeps_every_step`, `test_continuous_chunks_are_dated_by_their_observation`,
`test_a_sequential_chunk_is_dated_from_its_send_and_keeps_its_hold`, `test_sequential_execution_uses_the_same_lead_rule`.

**T2 Timelines of the policy's own are kept.** Exceptions to T1, each used verbatim:
an explicit `first_timestamp_ms` (never bridged); a timestep queue's chunk at
`origin + Δtimestep·dt`; a continuous
chunk with a bridge in front at "now".
Tests: `test_an_explicit_start_time_always_wins`, `test_a_chunk_with_its_own_timestamp_is_not_bridged`,
`test_async_queue_replacements_preserve_the_initial_policy_timeline`, `test_a_chunk_on_its_own_timeline_is_not_dated_by_the_observation`,


**T3 Late steps are dropped, never retimed** (the `DropLate` filter, in every default pipeline). A
step due within `MIN_LEAD_MS` (20 ms) of now is not sent; the others keep their times, exactly
`anchor + int(k·dt)`, and every step's time is still reported. A chunk late entirely is not sent.
A chunk stamped at the send and the final target of a run always pass. Under T1 in sequential
execution nothing is ever late.
Tests: `test_waypoints_already_in_the_past_are_trimmed_before_sending`,
`test_a_trimmed_chunk_still_reports_where_each_caller_step_landed`, `test_a_lead_chunk_that_is_late_entirely_is_not_sent_and_not_guarded`.

**T4 One-step chunks in continuous execution** are spaced one policy step apart by a playback
clock that follows the observations by at most 10% of a step per step and jumps to them when
more than 100 ms off. This is the one bounded retiming. Test:
`test_one_step_waypoints_are_spaced_by_the_policy_step_despite_bursty_observations`.

**T5 Rolling window.** A request dated by T1 re-sends the earlier waypoints that are not due
yet. Tests: `test_lead_chunks_resend_the_earlier_waypoints_that_are_not_yet_due`,
`test_a_lead_chunk_resends_the_earlier_waypoints_due_before_it`.

**T6 End of a chunk.** `HoldPadding(min_horizon_ms=500)` (in every default pipeline) extends a
chunk with its last target up to the horizon, except one-step chunks in continuous execution,
which the next step replaces before they are due, and live targets. The padding is not motion
anyone waits for. Tests: `test_hold_padding_holds_the_last_target_up_to_the_horizon`,
`test_a_short_chunk_holds_its_final_target_to_fill_the_buffer`,
`test_the_padded_tail_of_a_short_chunk_is_not_motion_anyone_waits_for`.

**T7 IO and computed actions run when the policy's first step is due**, on every path and
without blocking the loop: policy waypoint zero of what is sent (behind a bridge too). A chunk
replaced before it is sent runs neither; one pending when the session stops is cancelled; a hook
that raises ends the run, and none runs once its motion was stopped (stop condition, failure).
IO without motion runs at once. IO is a separate
write, so it reaches the controller one network delay later; NOVA's action chunks are to carry
it.
Tests: `test_io_of_a_continuous_chunk_goes_with_its_first_step`,
`test_a_bridged_chunk_hands_its_io_and_computed_action_to_the_policys_first_step`,
`test_a_chunk_action_runs_when_its_step_is_due_or_at_once_when_past`,
`test_a_replaced_chunk_never_runs_its_action_and_stop_cancels_the_rest`, `test_a_chunk_action_fails_the_session_when_it_raises`,
`test_a_chunk_action_is_skipped_when_the_motion_stopped_before_it_was_due`.

**T8 Pacing.** Sequential: the next observation waits until the chunk's last step is due and the
robot stands still. Continuous with `rate_hz`: ticks on an absolute grid; more than one period
behind, the grid restarts instead of firing catch-up ticks (*restart untested*).
Tests: `test_sequential_mode_delays_next_inference_until_deadline_and_standstill`,
`test_sequential_mode_waits_out_the_braking_ramp_after_the_deadline`,
`test_fixed_rate_makes_up_slow_ticks_instead_of_lowering_the_rate`.

**T10 End of a run.** A normal end waits until the session clock passes the last waypoint and
stops on the last target; an error drops what was accepted. Tests:
`test_a_normal_end_drains_before_stopping`, `test_drain_stops_on_the_last_target_instead_of_keeping_the_velocity`,
`test_an_error_drops_the_accepted_waypoints_instead_of_draining`.

**T11 Garbage collection.** Objects allocated up to policy preparation are frozen out of
collection while the loop runs and released when the run ends. Test:
`test_setup_objects_are_frozen_only_while_the_loop_runs`.

**T12 Setups the executor would run wrongly are refused at construction:** under sequential
execution a timestep queue (`QueuedPolicyClient`, LeRobot's async queue) and a `LiveBuffer`
in the pipeline; under continuous execution a timestep queue without its `TimestepTimeline`,
which keeps its chunks on one timeline. Tests:
`test_a_timestep_queue_or_a_live_buffer_under_sequential_execution_is_rejected`,
`test_a_timestep_queue_without_its_timeline_is_refused_before_anything_moves`.

**T13 The send pipeline.** Everything that reaches NOVA passes one ordered list per motion group:
the common chunk, chunk modules, `Timestamps()` (T1, T2), filters (T3), waypoint modules, the
sender (T5, T7). The list is exactly what runs: `pipeline=None` on `PolicyExecutor`, `jog_joints`,
`jog_tcp` and the session runs `default_pipeline()`, and an explicit list replaces it. Each motion
group runs its own copy; a misordered list is refused at construction.

| Default | Joggers | Executor, sequential | Executor, continuous | Executor, continuous, timestep queue |
| --- | --- | --- | --- | --- |
| Modules | `HoldPadding`, `Timestamps`, `DropLate` | `Bridge`, `EndpointRamp`, `JointClamp`, then the joggers' | `JointClamp`, then the joggers' | the queue's `default_modules` (`Smoothing`, `TimestepTimeline`, `Bridge(always_anchor=True)`), `JointClamp`, then the joggers' |

Chunk modules see every chunk when it is queued and may change steps and timing, keeping
`when_due` and `start_step` on the steps they belong to. `Smoothing(passes)` filters a queue's
plan and keeps the steps it already sent exact (`retained_prefix_steps`). `Bridge(always_anchor)`
connects the measured position to the first step, dates a bridged chunk at the send in continuous execution and leaves chunks on their own timeline alone. `EndpointRamp` eases
each chunk in and out. `JointClamp` holds joint targets inside the limits the session reads at its
start. `LiveBuffer(window_ms)` turns live targets into a horizon on its own timeline (T2) and moves
IO to the newest target; it times each target by `observed_at_ms`, which for a live target means
when it was produced (the jogger's loop time, shared by all arms), and a lead chunk by the session
clock. `JointClamp` is M7, `HoldPadding` T6. Filters drop waypoints without retiming them (`DropLate`, T3).
Waypoint modules run at every send, on raw values, and return values only, so no timing contract
changes: `EaseIn(duration_s)` blends from where the robot stood by each waypoint's timestamp.
What reaches NOVA per scenario is pinned by `test_what_reaches_nova_is_unchanged`; a queued LeRobot
client by `test_a_queued_lerobot_client_merges_its_chunks_on_one_timeline`.
Tests: `test_split_pipeline_keeps_each_kind_in_order`, `test_a_filter_after_a_waypoint_module_or_a_chunk_module_after_a_filter_is_refused`, `test_a_chunk_module_after_the_timestamps_is_refused`, `test_a_misordered_pipeline_is_refused_before_anything_moves`, `test_the_pipeline_is_handed_to_every_session`, `test_each_session_runs_its_own_copy_of_a_module`, `test_the_defaults_are_valid_pipelines`, `test_smoothing_keeps_the_steps_it_already_sent_exact`, `test_bridge_connects_the_robot_to_the_first_step_at_the_chunks_spacing`, `test_a_bridged_chunk_dated_by_its_observation_is_dated_at_the_send_instead`, `test_endpoint_ramp_keeps_every_step_and_moves_io_with_its_step`, `test_joint_clamp_holds_targets_inside_the_limits`, `test_the_session_reads_joint_limits_only_for_a_joint_clamp`, `test_drop_late_keeps_what_is_still_reachable_and_never_retimes`, `test_waypoint_modules_reshape_the_values_but_never_the_timestamps`, `test_a_waypoint_module_may_not_add_or_drop_waypoints`, `test_a_filled_live_buffer_is_replayed_as_a_horizon`, `test_a_live_buffer_moves_when_due_to_the_newest_target`, `test_sessions_given_one_live_buffer_do_not_share_its_points`, `test_a_live_buffer_times_targets_by_when_they_were_produced`, `test_a_live_buffer_times_lead_chunks_by_the_session_clock`, `test_live_targets_carry_the_loop_time_in_exact_grid_steps`, `test_ease_in_gives_a_moment_the_same_value_whenever_it_is_sent`.

## Known gaps

Behaviour the contracts above do not cover yet, found while writing them:

- **Missing data is filled silently.** A missing group's joints are skipped (later indices
  shift), a missing pose drops its keys, a missing IO reads `0.0`; relative targets for a group
  without state are sent as absolute.
- **Relative mode is per motion group:** one relative observation makes the group's joints and
  TCP relative.
- **IO names are not scoped by controller:** the same name on two controllers resolves to the
  last.
- **The observed TCP is the controller's active TCP**, not the one named for the action.
- **LEAPP and NATS policies map no IO actions.**
- **GR00T's TCP default** (metres and rot6d in, mm and rotation vectors out) bypasses M3.
