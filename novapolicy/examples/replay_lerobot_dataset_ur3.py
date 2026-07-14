"""Example: replay one LeRobot dataset episode on the virtual UR3/cobot setup.

This example does not contact a LeRobot policy server. It loads one episode from a
LeRobot dataset and replays the recorded absolute joint actions plus the 7th
recorded gripper action through NOVA waypoint jogging.

Run from the repository root. First review the arguments:

    PYTHONPATH=. uv run --extra novapolicy-lerobot --extra nova-rerun-bridge \
        python novapolicy/examples/replay_lerobot_dataset_ur3.py --help

Add ``--yes`` only when you are ready to execute robot motion.

Typical local setup:

    PYTHONPATH=. uv run --extra novapolicy-lerobot --extra nova-rerun-bridge \
        python novapolicy/examples/replay_lerobot_dataset_ur3.py \
        --nova-api http://172.31.11.129 \
        --dataset wandelbotsgmbh/09_Handover \
        --episode 0 \
        --move-to-start

Notes:
- The default controller is ``cobot`` because this virtual UR3e setup exposes
  the UR3 motion group through that controller name.
- Replay follows ``examples/replay/replay_episode.py``: each policy tick indexes
  the dataset by elapsed wall time and sends a timestamped lookahead chunk with
  ``first_timestamp_ms`` anchored to the original episode timeline.
- The dataset action is interpreted as 6 absolute joint targets in radians
  followed by one gripper command. The first frame's gripper command in each
  chunk is mapped with ``BoolMapping(off=0.0, on=100.0)`` and written to NOVA IO.
- Rerun is enabled through the NOVA program viewer and the executor's policy
  logger. Install/run with ``--extra nova-rerun-bridge``.
"""

from __future__ import annotations

import argparse
from bisect import bisect_right
from dataclasses import dataclass
from pathlib import Path
import time
from typing import TYPE_CHECKING, Any

from lerobot.datasets.lerobot_dataset import LeRobotDataset
import numpy as np

from nova import NovaConfig, ProgramContext, program, run_program, viewers
from novapolicy import BoolMapping, Observation, PolicyExecutor, PolicySchema, WaypointConfig
from novapolicy.types import ActionChunk

if TYPE_CHECKING:
    from nova.types import RobotState

DEFAULT_NOVA_API = "http://172.31.11.129"
DEFAULT_DATASET = "wandelbotsgmbh/09_Handover"
DEFAULT_CONTROLLER = "cobot"
DEFAULT_CELL = "cell"
DEFAULT_FPS = 15.0

_ARGS: argparse.Namespace | None = None


@dataclass(frozen=True, slots=True)
class _ReplayFrame:
    joints: list[float]
    gripper: bool
    state_joints: list[float]
    timestamp_s: float


class DatasetEpisodeReplayPolicy:
    """PolicyClient that emits recorded dataset actions as trajectory chunks."""

    def __init__(
        self,
        *,
        dataset: str,
        episode: int,
        revision: str | None,
        root: Path | None,
        fps: float,
        gripper_io: str,
        gripper_threshold: float,
        start_frame: int,
        max_frames: int | None,
        chunk_size: int,
    ) -> None:
        self._dataset = dataset
        self._episode = episode
        self._revision = revision
        self._root = root
        self._fps = fps
        self._gripper_io = gripper_io
        self._gripper_threshold = gripper_threshold
        self._start_frame = start_frame
        self._max_frames = max_frames
        self._chunk_size = chunk_size
        self._motion_group_ids: list[str] = []
        self._frames = self._load_frames()
        self._timestamps_from_start_s = [
            frame.timestamp_s - self._frames[0].timestamp_s for frame in self._frames
        ]
        self._replay_start: float | None = None
        self._last_step = 0

    @property
    def frame_count(self) -> int:
        return len(self._frames)

    @property
    def replayed_frames(self) -> int:
        return min(self._last_step + 1, len(self._frames))

    @property
    def first_state_joints(self) -> list[float]:
        return list(self._frames[0].state_joints)

    @property
    def duration_s(self) -> float:
        return self._timestamps_from_start_s[-1]

    async def connect(self, motion_group_ids: list[str]) -> None:
        self._motion_group_ids = motion_group_ids

    async def validate_schema(self, schema: PolicySchema) -> None:
        joint_dof = 0
        for _key, motion_groups in schema.joint_action_keys:
            joint_dof += 6 * len(motion_groups)
        io_dof = len(schema.io_action_keys)
        expected = joint_dof + io_dof
        if expected != 7:
            msg = (
                "Dataset replay expects a UR3 schema with 6 joint actions and "
                f"1 IO action, got {joint_dof} joint values and {io_dof} IO actions."
            )
            raise ValueError(msg)

    async def get_actions(
        self,
        states: dict[str, RobotState],
        schema: PolicySchema,
        images: dict[str, Any] | None = None,
        io_values: dict[str, object] | None = None,
    ) -> ActionChunk:
        del states, schema, images, io_values
        if not self._motion_group_ids:
            msg = "Dataset replay policy was not connected before get_actions()."
            raise RuntimeError(msg)

        if self._replay_start is None:
            self._replay_start = time.monotonic()
        elapsed_s = time.monotonic() - self._replay_start
        step = bisect_right(self._timestamps_from_start_s, elapsed_s) - 1
        step = min(max(step, 0), len(self._frames) - 1)
        self._last_step = step

        chunk_end = min(step + self._chunk_size, len(self._frames))
        chunk_frames = (
            self._frames[step:chunk_end] if step < len(self._frames) else [self._frames[-1]]
        )

        mg_id = self._motion_group_ids[0]
        return ActionChunk(
            joints={mg_id: [frame.joints for frame in chunk_frames]},
            ios={mg_id: {self._gripper_io: chunk_frames[0].gripper}},
            dt_ms=1000.0 / self._fps,
            first_timestamp_ms=int(self._timestamps_from_start_s[step] * 1000),
        )

    async def close(self) -> None:
        pass

    def _load_frames(self) -> list[_ReplayFrame]:
        dataset = LeRobotDataset(
            self._dataset,
            root=self._root,
            episodes=[self._episode],
            revision=self._revision,
            download_videos=False,
        )
        rows = dataset.hf_dataset
        end_frame = len(rows) if self._max_frames is None else self._start_frame + self._max_frames
        selected = range(self._start_frame, min(end_frame, len(rows)))
        frames = [self._frame_from_row(rows[idx]) for idx in selected]
        if not frames:
            msg = (
                f"Episode {self._episode} of {self._dataset!r} produced no replay frames "
                f"for start_frame={self._start_frame}, max_frames={self._max_frames}."
            )
            raise ValueError(msg)
        return frames

    def _frame_from_row(self, row: dict[str, Any]) -> _ReplayFrame:
        action = _as_float_list(row["action"])
        state = _as_float_list(row["observation.state"])
        if len(action) != 7:
            msg = f"Expected dataset action dimension 7, got {len(action)}"
            raise ValueError(msg)
        if len(state) < 6:
            msg = f"Expected dataset state dimension at least 6, got {len(state)}"
            raise ValueError(msg)
        return _ReplayFrame(
            joints=action[:6],
            gripper=action[6] >= self._gripper_threshold,
            state_joints=state[:6],
            timestamp_s=float(row["timestamp"]),
        )


def _as_float_list(value: Any) -> list[float]:
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return [float(item) for item in np.asarray(value, dtype=np.float32).reshape(-1)]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nova-api", default=DEFAULT_NOVA_API, help="NOVA API base URL")
    parser.add_argument("--cell", default=DEFAULT_CELL, help="NOVA cell id")
    parser.add_argument("--controller", default=DEFAULT_CONTROLLER, help="NOVA controller id")
    parser.add_argument("--motion-group", type=int, default=0, help="Motion group index")
    parser.add_argument("--dataset", default=DEFAULT_DATASET, help="LeRobot dataset repo id")
    parser.add_argument(
        "--dataset-root", type=Path, default=None, help="Optional local dataset root"
    )
    parser.add_argument("--revision", default="main", help="Dataset revision/tag")
    parser.add_argument("--episode", type=int, default=0, help="Episode index to replay")
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS, help="Recorded dataset frequency")
    parser.add_argument(
        "--policy-rate-hz",
        type=float,
        default=20.0,
        help="How often to resend timestamped lookahead chunks",
    )
    parser.add_argument("--start-frame", type=int, default=0, help="First episode frame to replay")
    parser.add_argument("--max-frames", type=int, default=None, help="Optional frame limit")
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=32,
        help="Maximum number of dataset frames sent as one waypoint chunk",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=None,
        help="Execution timeout; defaults to the selected episode duration",
    )
    parser.add_argument(
        "--gripper-io",
        default="digital_out[0]",
        help="NOVA IO key used for the replayed gripper action",
    )
    parser.add_argument(
        "--gripper-threshold",
        type=float,
        default=50.0,
        help="Dataset gripper action threshold mapped to hardware True",
    )
    parser.add_argument("--move-to-start", action="store_true", help="Move cobot to episode start")
    parser.add_argument("--yes", action="store_true", help="Confirm execution")
    return parser.parse_args()


def _require_args() -> argparse.Namespace:
    if _ARGS is None:
        raise RuntimeError("Arguments were not parsed before program execution")
    return _ARGS


@program(
    id="replay_lerobot_dataset_ur3",
    name="Replay LeRobot Dataset UR3",
    viewer=viewers.Rerun(),
)
async def replay_lerobot_dataset_ur3(ctx: ProgramContext) -> None:
    args = _require_args()
    cell = ctx.nova.cell(args.cell)
    motion_group = (await cell.controller(args.controller))[args.motion_group]

    schema = PolicySchema(
        observations=[
            Observation.joint_positions("arm", source=motion_group),
            Observation.io(
                "gripper",
                source=motion_group,
                io=args.gripper_io,
                mapping=BoolMapping(off=0.0, on=100.0, threshold=args.gripper_threshold),
                action=True,
            ),
        ]
    )

    policy = DatasetEpisodeReplayPolicy(
        dataset=args.dataset,
        episode=args.episode,
        revision=args.revision,
        root=args.dataset_root,
        fps=args.fps,
        gripper_io=args.gripper_io,
        gripper_threshold=args.gripper_threshold,
        start_frame=args.start_frame,
        max_frames=args.max_frames,
        chunk_size=args.chunk_size,
    )
    timeout_s = args.timeout_s if args.timeout_s is not None else policy.duration_s + 5.0

    executor = PolicyExecutor(
        schema,
        policy,
        timeout_s=timeout_s,
        policy_rate_hz=args.policy_rate_hz,
        motion=WaypointConfig(state_rate_ms=10),
        start_joint_position={motion_group: policy.first_state_joints}
        if args.move_to_start
        else None,
    )

    print("Replaying LeRobot dataset episode...")
    print(f"  NOVA:          {args.nova_api}")
    print(f"  controller:    {args.controller}[{args.motion_group}]")
    print(f"  dataset:       {args.dataset}")
    print(f"  revision:      {args.revision}")
    print(f"  episode:       {args.episode}")
    print(f"  frames:        {policy.frame_count}")
    print(f"  fps:           {args.fps:g}")
    print(f"  chunk size:    {args.chunk_size}")
    print(f"  policy rate:   {args.policy_rate_hz:g} Hz")
    print(f"  timeout:       {timeout_s:.2f}s")
    print(f"  gripper IO:    {args.gripper_io}")
    result = await executor.run()
    print(
        f"Done: reason={result.reason} executor_steps={result.steps} "
        f"replayed_frames={policy.replayed_frames} duration={result.duration_s:.2f}s"
    )


def main() -> None:
    global _ARGS
    _ARGS = _parse_args()
    if not _ARGS.yes:
        raise SystemExit(
            "This example can move the robot. Re-run with --yes after checking the arguments."
        )
    if _ARGS.chunk_size <= 0:
        raise SystemExit("--chunk-size must be greater than 0")
    run_program(replay_lerobot_dataset_ur3, nova_config=NovaConfig(host=_ARGS.nova_api))


if __name__ == "__main__":
    main()
