"""
Database module that provides access to the separated store modules.

This module maintains backward compatibility by using the store container
to provide the same interface as before.
"""

from .store import DatabaseConnection, ProgramInstanceStore, ProgramRunStore, ProgramTemplateStore

# Re-export everything for backward compatibility
__all__ = ["DatabaseConnection", "ProgramTemplateStore", "ProgramInstanceStore", "ProgramRunStore"]
