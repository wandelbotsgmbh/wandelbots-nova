"""
Dependency Injection Container for the Nova Python Framework

This module provides a clean and simple DI container configuration following
the multiple containers pattern from dependency-injector documentation.
"""

from dependency_injector import containers, providers


class ProcessorsContainer(containers.DeclarativeContainer):
    """Container for program run processors"""
    
    config: providers.Configuration = providers.Configuration()
    
    # Import processor implementations
    from .processors.implementations import (
        AsyncioProgramRunProcessor,
        ThreadProgramRunProcessor,
        ProcessProgramRunProcessor,
    )
    
    # Processor implementations
    asyncio_processor: AsyncioProgramRunProcessor = providers.Factory(AsyncioProgramRunProcessor)
    thread_processor: ThreadProgramRunProcessor = providers.Factory(ThreadProgramRunProcessor)
    process_processor: ProcessProgramRunProcessor = providers.Factory(ProcessProgramRunProcessor)
    
    # Current active processor - defaults to asyncio
    current_processor: providers.Selector = providers.Selector(
        config.processor_type,
        asyncio=asyncio_processor,
        thread=thread_processor,
        process=process_processor
    )


class StoreContainer(containers.DeclarativeContainer):
    """Container for data store layer"""
    
    config: providers.Configuration = providers.Configuration()
    
    # Import store implementations
    from .store.base import DatabaseConnection
    from .store.program_template import ProgramTemplateStore
    from .store.program_instance import ProgramInstanceStore
    from .store.program_run import ProgramRunStore
    
    # Database connection - singleton to ensure single connection
    database_connection: DatabaseConnection = providers.Singleton(
        DatabaseConnection,
        db_path=config.database_path
    )
    
    # Store implementations
    program_template_store: ProgramTemplateStore = providers.Factory(
        ProgramTemplateStore,
        db_connection=database_connection
    )
    
    program_instance_store: ProgramInstanceStore = providers.Factory(
        ProgramInstanceStore,
        db_connection=database_connection
    )
    
    program_run_store: ProgramRunStore = providers.Factory(
        ProgramRunStore,
        db_connection=database_connection
    )


class ServicesContainer(containers.DeclarativeContainer):
    """Container for business logic services"""
    
    config: providers.Configuration = providers.Configuration()
    stores: providers.DependenciesContainer = providers.DependenciesContainer()
    processors: providers.DependenciesContainer = providers.DependenciesContainer()
    
    # Import services
    from .services import (
        ProgramService,
        ProgramRunService,
        ProgramExecutionService,
        ProgramDiscoveryService
    )
    
    # Business logic services
    program_service: ProgramService = providers.Factory(
        ProgramService,
        template_store=stores.program_template_store,
        instance_store=stores.program_instance_store
    )
    
    program_run_service: ProgramRunService = providers.Factory(
        ProgramRunService,
        run_store=stores.program_run_store,
        instance_store=stores.program_instance_store
    )
    
    program_execution_service: ProgramExecutionService = providers.Factory(
        ProgramExecutionService,
        processor=processors.current_processor,
        template_store=stores.program_template_store,
        instance_store=stores.program_instance_store,
        run_store=stores.program_run_store
    )
    
    program_discovery_service: ProgramDiscoveryService = providers.Factory(
        ProgramDiscoveryService,
        template_store=stores.program_template_store,
        default_package=config.default_programs_package
    )


class NovaContainer(containers.DeclarativeContainer):
    """Main application container that composes all sub-containers"""
    
    # Configuration 
    config: providers.Configuration = providers.Configuration()
    
    # Sub-containers - instantiated here, not at package level
    processors: ProcessorsContainer = providers.Container(
        ProcessorsContainer,
        config=config.processors
    )
    
    stores: StoreContainer = providers.Container(
        StoreContainer,
        config=config.stores  
    )
    
    services: ServicesContainer = providers.Container(
        ServicesContainer,
        config=config.services,
        stores=stores,
        processors=processors
    )
    
    # Wiring configuration
    wiring_config = containers.WiringConfiguration(modules=[
        "nova_python_app.backbone.api.programs",
        "nova_python_app.backbone.api.runs", 
        "nova_python_app.backbone.api.discovery"
    ])


def create_container() -> NovaContainer:
    """
    Create and configure the Nova container with default settings.
    
    Returns:
        Configured NovaContainer instance
    """
    container = NovaContainer()
    
    # Default configuration
    container.config.from_dict({
        'processors': {
            'processor_type': 'asyncio'
        },
        'stores': {
            'database_path': 'nova_programs.db'
        },
        'services': {
            'default_programs_package': 'nova_python_app.programs'
        }
    })

    container.wire(modules=[
        "nova_python_app.backbone.api.programs",
        "nova_python_app.backbone.api.runs",
        "nova_python_app.backbone.api.discovery"
    ])

    
    return container
