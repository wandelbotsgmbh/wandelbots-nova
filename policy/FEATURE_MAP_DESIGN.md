# FeatureMap Design

## Core Insight

**The policy operates on a flat dictionary of named features — it never knows about motion groups, controllers, or hardware topology.**

This matches how LeRobot handles dual-arm policies. Feature names ARE the contract, defined at training time and baked into the dataset.

## What the policy sees

```python
# Observation (flat dict):
{
    "left_joint_1.pos": 0.1,
    "left_joint_2.pos": -1.5,
    ...
    "left_gripper": 0.0,       # IO value (float, from digital bool)
    "right_joint_1.pos": 0.2,
    ...
    "right_gripper": 100.0,
}

# Action (same flat structure):
{
    "left_joint_1.pos": 0.15,
    ...
    "left_gripper": 50.0,
    "right_joint_1.pos": 0.25,
    ...
}
```

## FeatureGroup API

```python
@dataclass
class FeatureGroup:
    motion_group: MotionGroup
    name: str                           # prefix for feature keys (e.g. "left", "right")
    ios: dict[str, str] = field(...)    # feature_name → hardware_io_key

    def joint_key(self, joint_index: int) -> str:
        return f"{self.name}_joint_{joint_index + 1}.pos"

    def io_feature_key(self, io_name: str) -> str:
        return f"{self.name}_{io_name}"
```

Example:
```python
FeatureGroup(
    motion_group=mg1,
    name="left",
    ios={"gripper": "digital_out[0]", "conveyor_sensor": "digital_in[3]"},
)
```

Produces features:
- `left_joint_1.pos` through `left_joint_6.pos`
- `left_gripper` (mapped to `digital_out[0]`)
- `left_conveyor_sensor` (mapped to `digital_in[3]`)

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
        ios={"gripper": "digital_out[0]"},
    ),
    FeatureGroup(
        motion_group=mg2,
        name="right",
        ios={"gripper": "digital_out[0]"},
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
