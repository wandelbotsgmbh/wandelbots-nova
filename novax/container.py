"""
Dependency Injection Container for the Nova Python Framework

This module provides a clean and simple DI container configuration following
the multiple containers pattern from dependency-injector documentation.
"""

from dependency_injector import containers, providers

from .interfaces import (
    DatabaseConnectionInterface,
    ProgramAPIServiceInterface,
    ProgramDiscoveryServiceInterface,
    ProgramExecutionServiceInterface,
    ProgramInstanceStoreInterface,
    ProgramRunAPIServiceInterface,
    ProgramRunStoreInterface,
    ProgramTemplateStoreInterface,
)
from .processors.interface import ProgramRunProcessorInterface


class ProcessorsContainer(containers.DeclarativeContainer):
    """Container for program run processors"""

    config: providers.Configuration = providers.Configuration()

    # Import processor implementations
    from .processors.implementations import (
        AsyncioProgramRunProcessor,
        ProcessProgramRunProcessor,
        ThreadProgramRunProcessor,
    )

    # Processor implementations
    asyncio_processor: ProgramRunProcessorInterface = providers.Factory(AsyncioProgramRunProcessor)
    thread_processor: ProgramRunProcessorInterface = providers.Factory(ThreadProgramRunProcessor)
    process_processor: ProgramRunProcessorInterface = providers.Factory(ProcessProgramRunProcessor)

    # Current active processor - defaults to asyncio
    current_processor: providers.Selector = providers.Selector(
        config.processor_type,
        asyncio=asyncio_processor,
        thread=thread_processor,
        process=process_processor,
    )


class StoreContainer(containers.DeclarativeContainer):
    """Container for data store layer"""

    config: providers.Configuration = providers.Configuration()

    # Import store implementations
    from .store.base import DatabaseConnection
    from .store.program_instance import ProgramInstanceStore
    from .store.program_run import ProgramRunStore
    from .store.program_template import ProgramTemplateStore

    # Database connection - singleton to ensure single connection
    database_connection: DatabaseConnectionInterface = providers.Singleton(
        DatabaseConnection, db_path=config.database_path
    )

    # Store implementations
    program_template_store: ProgramTemplateStoreInterface = providers.Factory(
        ProgramTemplateStore, db_connection=database_connection
    )

    program_instance_store: ProgramInstanceStoreInterface = providers.Factory(
        ProgramInstanceStore, db_connection=database_connection
    )

    program_run_store: ProgramRunStoreInterface = providers.Factory(
        ProgramRunStore, db_connection=database_connection
    )


class ServicesContainer(containers.DeclarativeContainer):
    """Container for business logic services"""

    config: providers.Configuration = providers.Configuration()
    stores: providers.DependenciesContainer = providers.DependenciesContainer()
    processors: providers.DependenciesContainer = providers.DependenciesContainer()

    # Import services
    from .services import (
        ProgramDiscoveryService,
        ProgramExecutionService,
        ProgramRunService,
        ProgramService,
    )

    # Business logic services
    program_service: ProgramAPIServiceInterface = providers.Factory(
        ProgramService,
        template_store=stores.program_template_store,
        instance_store=stores.program_instance_store,
    )

    program_run_service: ProgramRunAPIServiceInterface = providers.Factory(
        ProgramRunService,
        run_store=stores.program_run_store,
        instance_store=stores.program_instance_store,
    )

    program_execution_service: ProgramExecutionServiceInterface = providers.Factory(
        ProgramExecutionService,
        processor=processors.current_processor,
        template_store=stores.program_template_store,
        instance_store=stores.program_instance_store,
        run_store=stores.program_run_store,
    )

    program_discovery_service: ProgramDiscoveryServiceInterface = providers.Factory(
        ProgramDiscoveryService,
        template_store=stores.program_template_store,
        default_package=config.default_programs_package,
    )


class NovaContainer(containers.DeclarativeContainer):
    """Main application container that composes all sub-containers"""

    # Configuration
    config: providers.Configuration = providers.Configuration()

    # Sub-containers - instantiated here, not at package level
    processors: ProcessorsContainer = providers.Container(
        ProcessorsContainer, config=config.processors
    )

    stores: StoreContainer = providers.Container(StoreContainer, config=config.stores)

    services: ServicesContainer = providers.Container(
        ServicesContainer, config=config.services, stores=stores, processors=processors
    )

    # Wiring configuration
    wiring_config = containers.WiringConfiguration(
        modules=["novax.api.programs", "novax.api.runs", "novax.api.discovery"]
    )


def create_container() -> NovaContainer:
    """
    Create and configure the Nova container with default settings.

    Returns:
        Configured NovaContainer instance
    """
    container = NovaContainer()

    # Default configuration
    container.config.from_dict(
        {
            "processors": {"processor_type": "asyncio"},
            "stores": {"database_path": "nova_programs.db"},
            "services": {"default_programs_package": "nova_python_app.programs"},
        }
    )

    container.wire(modules=["novax.api.programs", "novax.api.runs", "novax.api.discovery"])

    return container
