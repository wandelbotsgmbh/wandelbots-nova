import pytest

from wandelscript import migration


@pytest.mark.parametrize(
    "code, migrated_code",
    [
        ("pose = [test[0], test[1], test[2]]", "pose = (test[0], test[1], test[2])"),
        (
            "pose = [test[0], test[1], test[2], test[3], test[4], test[5]]",
            "pose = (test[0], test[1], test[2], test[3], test[4], test[5])",
        ),
        ("pose = [..., test[3], test[4], test[5]]", "pose = (..., test[3], test[4], test[5])"),
        (
            """write(device, value, key)
write(device, "a string" + "another string", key)
write(device, 4, "a string key")
write(device, 4 + 6, "a string key" + "another string")""",
            """write(device, key, value)
write(device, key, "a string" + "another string")
write(device, "a string key", 4)
write(device, "a string key" + "another string", 4 + 6)""",
        ),
        (
            """write(io, True, "tool_out[1]")
def gripper_off():
    pass""",
            """write(io, "tool_out[1]", True)
def gripper_off():
    pass""",
        ),
    ],
    ids=[
        "nested_position",
        "nested_pose",
        "nested_orientation",
        "write_key_value_order",
        "write_key_value_order_with_expressions",
    ],
)
def test_migration_nested_orientation(code, migrated_code):
    assert migration.migrate_v1_0_0(code) == migrated_code
