import re


def migrate_v1_0_0(content: str) -> str:
    # Regex to match [x, y, z, rx, ry, rz] and convert to (x, y, z, rx, ry, rz)
    pose_pattern = re.compile(
        r"\[\s*"
        r"((?:\w+\[\d+\]|[^,\]]+)),\s*"
        r"((?:\w+\[\d+\]|[^,\]]+)),\s*"
        r"((?:\w+\[\d+\]|[^,\]]+)),\s*"
        r"((?:\w+\[\d+\]|[^,\]]+)),\s*"
        r"((?:\w+\[\d+\]|[^,\]]+)),\s*"
        r"((?:\w+\[\d+\]|[^,\]]+))"
        r"\s*\]"
    )
    content = pose_pattern.sub(r"(\1, \2, \3, \4, \5, \6)", content)

    # Regex to match [..., rx, ry, rz] and convert to (..., rx, ry, rz)
    orientation_pattern = re.compile(
        r"\[\s*\.\.\.\s*,\s*"
        r"((?:\w+\[\d+\]|[^,\]]+)),\s*"
        r"((?:\w+\[\d+\]|[^,\]]+)),\s*"
        r"((?:\w+\[\d+\]|[^,\]]+))"
        r"\s*\]"
    )

    content = orientation_pattern.sub(r"(..., \1, \2, \3)", content)

    # Note: the order here matters, since this will also match on the orientation pattern above
    # Regex to match [x, y, z] and convert to (x, y, z)
    position_pattern = re.compile(
        r"\[\s*((?:\w+\[\d+\]|[^,\]]+)),\s*((?:\w+\[\d+\]|[^,\]]+)),\s*((?:\w+\[\d+\]|[^,\]]+))\s*\]"
    )
    content = position_pattern.sub(r"(\1, \2, \3)", content)

    # Convert all { to [ and } to ]
    content = content.replace("{", "[").replace("}", "]")

    # Convert write key value order from write(device, value, key) to write(device, key, value)
    write_pattern = re.compile(r"write\(\s*([^\n,]+)\s*,\s*([^\n,]+)\s*,\s*([^\n,]+)\s*\)")
    content = write_pattern.sub(r"write(\1, \3, \2)", content)

    return content
