from dataclasses import dataclass


@dataclass
class SystemVersion:
    """Represents the system version information"""

    major: int
    minor: int
    patch: int
    build: int
    version_string: str


@dataclass
class SystemInfo:
    """Represents system information"""

    name: str
    description: str
    version: SystemVersion
