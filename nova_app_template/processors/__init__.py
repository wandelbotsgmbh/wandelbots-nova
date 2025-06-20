"""
Processors package for Nova Python App

This package provides different program execution strategies through a decoupled
dependency injection container. It implements the processor interface and provides
multiple execution backends (AsyncIO, Thread, Process).

The package is designed to be completely independent from other Nova components,
depending only on the processor interface.
"""

from .container import ProcessorsContainer, processors_container
from .interface import ProgramRunProcessorInterface
from .implementations import (
    AsyncioProgramRunProcessor,
    ThreadProgramRunProcessor,
    ProcessProgramRunProcessor,
)

__all__ = [
    'ProcessorsContainer',
    'processors_container',
    'ProgramRunProcessorInterface',
    'AsyncioProgramRunProcessor',
    'ThreadProgramRunProcessor',
    'ProcessProgramRunProcessor',
]
