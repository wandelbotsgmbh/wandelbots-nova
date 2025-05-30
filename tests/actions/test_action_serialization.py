import json

from nova.actions import cartesian_ptp, joint_ptp
from nova.actions.base import Action
from nova.api import models
from nova.types import MotionSettings, Pose


def test_program_serialization_deserialization():
    """Test that a program can be serialized and deserialized correctly."""
    # Create a sample program with some actions
    home_joints = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    target_pose = Pose((1.0, 2.0, 3.0, 0.0, 0.0, 0.0))

    # Create actions with custom settings
    actions = [
        joint_ptp(home_joints, settings=MotionSettings(tcp_velocity_limit=200)),
        cartesian_ptp(target_pose, settings=MotionSettings(tcp_velocity_limit=150)),
        joint_ptp(home_joints, settings=MotionSettings(tcp_velocity_limit=200)),
    ]

    # Serialize actions
    serialized_actions = []
    for action in actions:
        action_data = action.model_dump()
        serialized_actions.append(action_data)

    # Create a complete serializable program
    serialized_program = {
        "joint_trajectory": "dummy_trajectory_data",
        "tcp": "flange",
        "actions": serialized_actions,
    }

    # Simulate saving and loading from a file
    json_data = json.dumps(serialized_program)
    loaded_program = json.loads(json_data)

    # Deserialize actions
    deserialized_actions = []
    for action_data in loaded_program["actions"]:
        deserialized_actions.append(Action.from_dict(action_data))

    # Verify we have the right number of actions
    assert len(deserialized_actions) == len(actions)

    # Verify each action was correctly deserialized
    for i, original_action in enumerate(actions):
        deserialized_action = deserialized_actions[i]

        # Verify action type
        assert type(deserialized_action) is type(original_action)
        assert deserialized_action.type == original_action.type

        # Verify action target
        if hasattr(original_action, "target"):
            if isinstance(original_action.target, Pose):
                # For Pose targets, compare position and orientation
                assert deserialized_action.target.position == original_action.target.position
                assert deserialized_action.target.orientation == original_action.target.orientation
            else:
                # For joint targets, compare tuples
                assert deserialized_action.target == original_action.target

        # Verify action settings
        assert (
            deserialized_action.settings.tcp_velocity_limit
            == original_action.settings.tcp_velocity_limit
        )


def test_program_serialization_deserialization_collision_scene():
    """Test that a program can be serialized and deserialized correctly."""
    # Create a sample program with some actions
    home_joints = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    target_pose = Pose((1.0, 2.0, 3.0, 0.0, 0.0, 0.0))
    collision_scene = models.CollisionScene(
        colliders={
            "box1": models.Collider(
                shape=models.ColliderShape(
                    models.Box2(size_x=976, size_y=676, size_z=1, shape_type="box", box_type="FULL")
                ),
                pose=models.Pose2(position=[1080, -480, -82]),
            ),
            "cylinder1": models.Collider(
                shape=models.ColliderShape(
                    models.Cylinder2(radius=976, height=676, shape_type="cylinder")
                ),
                pose=models.Pose2(position=[1080, -480, -82]),
            ),
        },
        motion_groups={"robot_arm": models.CollisionMotionGroup(link_chain=[], tool=None)},
    )

    # Create actions with custom settings
    actions = [
        joint_ptp(
            home_joints,
            settings=MotionSettings(tcp_velocity_limit=200),
            collision_scene=collision_scene,
        ),
        cartesian_ptp(
            target_pose,
            settings=MotionSettings(tcp_velocity_limit=150),
            collision_scene=collision_scene,
        ),
        joint_ptp(
            home_joints,
            settings=MotionSettings(tcp_velocity_limit=200),
            collision_scene=collision_scene,
        ),
    ]

    # Serialize actions
    serialized_actions = []
    for action in actions:
        action_data = action.model_dump_json()
        serialized_actions.append(action_data)

    # Create a complete serializable program
    serialized_program = {
        "joint_trajectory": "dummy_trajectory_data",
        "tcp": "flange",
        "actions": serialized_actions,
    }

    # Simulate saving and loading from a file
    json_data = json.dumps(serialized_program)
    loaded_program = json.loads(json_data)

    # Deserialize actions
    deserialized_actions = []
    for action_data in loaded_program["actions"]:
        as_dict = json.loads(action_data)
        deserialized_actions.append(Action.from_dict(as_dict))

    # Verify we have the right number of actions
    assert len(deserialized_actions) == len(actions)

    # Verify each action was correctly deserialized
    for i, original_action in enumerate(actions):
        deserialized_action = deserialized_actions[i]

        # Verify action type
        assert type(deserialized_action) is type(original_action)
        assert deserialized_action.type == original_action.type

        # Verify action target
        if hasattr(original_action, "target"):
            if isinstance(original_action.target, Pose):
                # For Pose targets, compare position and orientation
                assert deserialized_action.target.position == original_action.target.position
                assert deserialized_action.target.orientation == original_action.target.orientation
            else:
                # For joint targets, compare tuples
                assert deserialized_action.target == original_action.target

        # Verify action settings
        assert (
            deserialized_action.settings.tcp_velocity_limit
            == original_action.settings.tcp_velocity_limit
        )
