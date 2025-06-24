"""
Processors Dependency Injection Container

This container is completely decoupled from other Nova components and only manages
processor-related dependencies. It follows the decoupled packages pattern from
dependency-injector documentation.

The container provides:
- Different processor implementations (AsyncIO, Thread, Process)
- Easy processor selection and configuration
- Extensibility for custom processor implementations
"""

from dependency_injector import containers, providers
from .implementations import (
    AsyncioProgramRunProcessor,
    ThreadProgramRunProcessor,
    ProcessProgramRunProcessor,
)


class ProcessorsContainer(containers.DeclarativeContainer):
    """
    Dependency injection container for program run processors.
    
    This container is designed to be completely independent from other Nova
    components, following the decoupled packages pattern.
    """
    
    # Configuration for processors
    config = providers.Configuration()
    
    # Processor implementations
    asyncio_processor = providers.Factory(AsyncioProgramRunProcessor)
    thread_processor = providers.Factory(ThreadProgramRunProcessor)
    process_processor = providers.Factory(ProcessProgramRunProcessor)
    
    # Current active processor - defaults to asyncio
    current_processor = providers.Selector(
        config.processor_type,
        asyncio=asyncio_processor,
        thread=thread_processor,
        process=process_processor
    )


# Export the container instance
processors_container = ProcessorsContainer()

# Set default configuration
processors_container.config.from_dict({
    'processor_type': 'asyncio'
})
