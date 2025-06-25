"""
Discovery Package

This package handles automatic program discovery and template synchronization
for the Nova Python Framework.

Note: This package is deprecated. Use ProgramDiscoveryService directly for new code.
"""

from .startup import (
    auto_discover_programs,
    initialize_discovery,
    initialize_discovery_with_service,
    setup_program_discovery,
    sync_templates_to_database,
)

__all__ = [
    "auto_discover_programs",
    "sync_templates_to_database",
    "initialize_discovery",
    "initialize_discovery_with_service",
    "setup_program_discovery",
]
