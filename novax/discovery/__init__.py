"""
Discovery Package

This package handles automatic program discovery and template synchronization
for the Nova Python Framework.
"""

from .auto_discovery import auto_discover_programs, sync_templates_to_database
from .startup import initialize_discovery, initialize_discovery_with_service

__all__ = [
    "auto_discover_programs",
    "sync_templates_to_database", 
    "initialize_discovery",
    "initialize_discovery_with_service"
]
