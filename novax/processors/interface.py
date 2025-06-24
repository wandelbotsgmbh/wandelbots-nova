"""
Interface for program run processors.

This interface defines the contract that all program run processors must implement.
It's designed to be completely independent from other Nova components.
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Callable


class ProgramRunProcessorInterface(ABC):
    """
    Interface for program run processors.
    
    This interface defines how programs should be executed by different processing strategies.
    Implementations can use different execution models like AsyncIO, threading, multiprocessing, etc.
    """
    
    @abstractmethod
    async def execute_program(
        self,
        program_name: str,
        run_id: str,
        program_function: Callable,
        program_model_instance: Any,
        template_data: Dict[str, Any],
        instance_data: Dict[str, Any],
        parameters: Dict[str, Any],
        environment_variables: Dict[str, str]
    ) -> Dict[str, Any]:
        """
        Execute a program with the given parameters.
        
        Args:
            program_name: Name of the program being executed
            run_id: Unique identifier for this execution run
            program_function: The actual function to execute
            program_model_instance: Instance of the program's data model
            template_data: Template configuration data
            instance_data: Instance-specific data
            parameters: Runtime parameters for the execution
            environment_variables: Environment variables to set during execution
            
        Returns:
            Dictionary containing execution results with the following structure:
            {
                'status': 'success' | 'error',
                'result': Any,  # The actual result from program execution
                'error': str | None,  # Error message if status is 'error'
                'execution_time': float,  # Time taken for execution in seconds
                'processor_type': str,  # Type of processor used
                'metadata': Dict[str, Any]  # Additional processor-specific metadata
            }
        """
        pass
