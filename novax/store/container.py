"""
Store Dependency Injection Container

This container is decoupled from other Nova components and only manages
store-related dependencies. It follows the decoupled packages pattern from
dependency-injector documentation.

The container provides:
- Database connection management
- Store implementations (Template, Instance, Run)
- Easy store configuration and extensibility
"""

from dependency_injector import containers, providers
from .base import DatabaseConnection
from .program_template import ProgramTemplateStore
from .program_instance import ProgramInstanceStore
from .program_run import ProgramRunStore


class StoreContainer(containers.DeclarativeContainer):
    """
    Dependency injection container for store layer.
    
    This container is designed to be completely independent from other Nova
    components, following the decoupled packages pattern.
    """
    
    # Configuration for stores
    config = providers.Configuration()
    
    # Database connection - singleton to ensure single connection
    database_connection = providers.Singleton(DatabaseConnection)
    
    # Store implementations
    program_template_store = providers.Factory(
        ProgramTemplateStore,
        db_connection=database_connection
    )
    
    program_instance_store = providers.Factory(
        ProgramInstanceStore,
        db_connection=database_connection
    )
    
    program_run_store = providers.Factory(
        ProgramRunStore,
        db_connection=database_connection
    )


# Export the container instance
store_container = StoreContainer()

# Set default configuration
store_container.config.from_dict({
    'database_path': 'nova_programs.db'
})
