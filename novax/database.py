"""
Database module that provides access to the separated store modules.

This module maintains backward compatibility by using the store container
to provide the same interface as before.
"""

from .store import (
    DatabaseConnection,
    ProgramTemplateStore,
    ProgramInstanceStore,
    ProgramRunStore,
    store_container
)

# Create global instances using the store container for backward compatibility
db_connection = store_container.database_connection()
template_store = store_container.program_template_store()
instance_store = store_container.program_instance_store()
run_store = store_container.program_run_store()

# Re-export everything for backward compatibility
__all__ = [
    'DatabaseConnection',
    'db_connection',
    'ProgramTemplateStore',
    'ProgramInstanceStore', 
    'ProgramRunStore',
    'template_store',
    'instance_store',
    'run_store'
]
