"""
Program Run Service

Service for program run-related operations.
"""

from typing import Dict, List, Optional, Any
from ..interfaces import (
    ProgramRunAPIServiceInterface,
    ProgramRunStoreInterface,
    ProgramInstanceStoreInterface,
)


class ProgramRunService(ProgramRunAPIServiceInterface):
    """Service for program run-related API operations"""
    
    def __init__(
        self,
        run_store: ProgramRunStoreInterface,
        instance_store: ProgramInstanceStoreInterface
    ):
        self.run_store = run_store
        self.instance_store = instance_store
    
    def initialize(self) -> None:
        """Initialize the service"""
        # Any initialization logic can go here
        pass
    
    def get_runs(self, program_name: Optional[str] = None) -> List[Dict[str, Any]]:
        """Get runs, optionally filtered by program name"""
        if program_name:
            return self.run_store.get_by_program(program_name)
        return self.run_store.get_all()
    
    def get_run(self, run_id: str) -> Optional[Dict[str, Any]]:
        """Get a specific run by ID"""
        return self.run_store.get(run_id)
    
    def create_run(self, run_data: Dict[str, Any]) -> Optional[str]:
        """Create a new run"""
        # Validate that the program exists
        program_name = run_data.get('program_name')
        if program_name and not self.instance_store.get(program_name):
            return None
        
        # Save the run
        if self.run_store.save(run_data):
            return run_data.get('run_id')
        return None
    
    def update_run_status(self, run_id: str, status: str, **kwargs) -> bool:
        """Update run status"""
        return self.run_store.update_status(run_id, status, **kwargs)
