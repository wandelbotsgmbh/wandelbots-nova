Based on how LeRobot handles dual-arm policies, here's the key insight:

**The policy operates on a flat dictionary of named features — it never knows about motion groups, controllers, or hardware topology.**

## LeRobot's Pattern

```python
# What the policy sees (flat feature dict):
observation = {
    "left_joint_1.pos": 0.1,
    "left_joint_2.pos": -1.5,
    ...
    "left_gripper.pos": 0.0,
    "right_joint_1.pos": 0.2,
    ...
    "right_gripper.pos": 100.0,
}

# What the policy outputs (same flat structure):
action = {
    "left_joint_1.pos": 0.15,
    ...
    "left_gripper.pos": 50.0,
    "right_joint_1.pos": 0.25,
    ...
}
```

The feature names ARE the contract. They're defined at training time and baked into the dataset. The policy is a pure function: flat dict in → flat dict out.

## What this means for nova_policy

The executor needs a **feature mapping** that translates between:
- LeRobot flat features ↔ NOVA motion groups + IOs

```python
# User defines the mapping once:
feature_map = FeatureMap(groups=[
    FeatureGroup(
        motion_group=mg1,
        role="left",                      # derives left_joint_1.pos ... left_gripper.pos
        gripper_io="digital_out[0]",     # hardware output for that gripper feature
        gripper_threshold=50.0,
    ),
    FeatureGroup(
        motion_group=mg2,
        role="right",
        gripper_io="digital_out[0]",
        gripper_threshold=50.0,
    ),
])

executor = PolicyExecutor(
    feature_map=feature_map,
    policy=WebSocketPolicyClient(url),
    on_reset=reset_robots,
)
```

The executor:
1. Reads joint positions from each motion group
2. Builds the flat observation dict using the feature map (adds prefixes)
3. Sends flat dict to policy
4. Receives flat action dict from policy
5. Splits it back into per-motion-group joints + IO commands using the feature map
6. Sends to PID runner

The policy never sees `0@ur10e` — it only sees `left_joint_1.pos`. This is the same contract as LeRobot training data.

## Benefits

- Policy trained with LeRobot works directly (same feature names)
- Policy is hardware-agnostic (works on any setup with matching joint count)
- Feature names are self-documenting (you know what left_joint_3.pos means)
- Validation at startup: feature map declares expected joints, executor can verify against hardware
- IO (gripper) is part of the feature space, not a separate channel
