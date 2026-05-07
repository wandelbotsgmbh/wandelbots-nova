# FeatureMap Design

## Core Insight

**The policy operates on a flat dictionary of named features — it never knows about motion groups, controllers, or hardware topology.**

This matches how LeRobot handles dual-arm policies. Feature names ARE the contract, defined at training time and baked into the dataset.

## What the policy sees

```python
# Observation (flat dict):
{
    "left_joint_position_1": 0.1,
    "left_joint_position_2": -1.5,
    ...
    "left_gripper": 0.0,       # IO value (float, from digital bool)
    "right_joint_position_1": 0.2,
    ...
    "right_gripper": 100.0,
}

# Action (same flat structure):
{
    "left_joint_position_1": 0.15,
    ...
    "left_gripper": 50.0,
    "right_joint_position_1": 0.25,
    ...
}
```

## FeatureGroup API

```python
@dataclass
class FeatureGroup:
    motion_group: MotionGroup
    name: str                          # default prefix for feature keys
    ios: dict[str, str] | None         # policy_name → hardware_io_key
    joint_key: str = ""                # override joint feature name (default: "{name}_joint_position")
    tcp_key: str = ""                  # override TCP feature name (default: "{name}_tcp")
    tcp_format: TcpFormat = NONE       # TCP representation (NONE, ROTATION_VECTOR, QUATERNION, ROT6D)
    model_dof: int = 0                 # expected joint count (0 = auto from robot)
    io_threshold: float = 0.5          # bool conversion threshold for IO actions
```

Key resolution:
- **Joints**: `{joint_key}_{i}` where `joint_key` defaults to `{name}_joint_position` → e.g. `left_joint_position_1`
- **TCP**: `{tcp_key}_{i}` where `tcp_key` defaults to `{name}_tcp` (only if `tcp_format != NONE`)
- **IOs**: dict keys used directly as feature names

Example:
```python
FeatureGroup(
    motion_group=mg1,
    name="left",
    ios={"left_gripper": "digital_out[0]", "left_conveyor_sensor": "digital_in[3]"},
)
```

Produces features:
- `left_joint_position_1` through `left_joint_position_6`
- `left_gripper` (mapped to `digital_out[0]`)
- `left_conveyor_sensor` (mapped to `digital_in[3]`)

For GR00T (array-based), override `joint_key` to match the server's expected keys:
```python
FeatureGroup(
    motion_group=mg1,
    name="left",
    joint_key="left_arm",
    tcp_key="left_eef_9d",
    tcp_format=TcpFormat.ROT6D,
    ios={"left_gripper": "digital_out[0]"},
)
```

## FeatureMap

```python
@dataclass
class FeatureMap:
    groups: list[FeatureGroup]

    async def start() -> None          # opens IO WebSocket streams
    async def stop() -> None           # closes IO streams
    async def build_observation(states) -> dict[str, float]
    def parse_action(features) -> tuple[joints_dict, ios_dict]
```

### IO Streaming

`FeatureMap.start()` opens one WebSocket per controller for all declared IO keys. Values update at controller rate (~100Hz). Guards and observations read from this shared cache — no HTTP polling needed.

### IO Writes

When `parse_action()` extracts IO features from the policy output, the executor writes them via fire-and-forget HTTP calls with deduplication (only writes when value changes). This avoids 429 rate limiting.

## Usage

```python
feature_map = FeatureMap(groups=[
    FeatureGroup(
        motion_group=mg1,
        name="left",
        ios={"left_gripper": "digital_out[0]"},
    ),
    FeatureGroup(
        motion_group=mg2,
        name="right",
        ios={"right_gripper": "digital_out[0]"},
    ),
])

executor = PolicyExecutor(
    feature_map=feature_map,
    policy=NatsPolicyClient(nc, subject="nova.policy.predict"),
    timeout_s=10.0,
)
result = await executor.run()
```

## Benefits

- Policy trained with LeRobot works directly (same feature names)
- Policy is hardware-agnostic (works on any setup with matching joint count)
- Feature names are self-documenting
- IO is part of the feature space, not a separate channel
- IO values available to safety guards at full rate via streaming
- Validation at startup: joint count mismatch caught immediately
