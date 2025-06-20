"""
Program Service

Service for program-related operations including templates and instances.
"""

from typing import Dict, List, Optional, Any
from ..interfaces import (
    ProgramAPIServiceInterface,
    ProgramTemplateStoreInterface,
    ProgramInstanceStoreInterface,
)



class ProgramService(ProgramAPIServiceInterface):
    """Service for program-related API operations"""
    
    def __init__(
        self,
        template_store: ProgramTemplateStoreInterface,
        instance_store: ProgramInstanceStoreInterface
    ):
        self.template_store = template_store
        self.instance_store = instance_store
    
    def initialize(self) -> None:
        """Initialize the service"""
        # Any initialization logic can go here
        pass
    
    def get_programs(self) -> List[Dict[str, Any]]:
        """Get all programs (instances)"""
        return self.instance_store.get_all()
    
    def get_program(self, name: str) -> Optional[Dict[str, Any]]:
        """Get a specific program by name"""
        return self.instance_store.get(name)
    
    def create_program(self, program_data: Dict[str, Any]) -> bool:
        """Create a new program instance"""
        # This would need to be implemented with proper validation
        # and creation of ProgramInstance objects
        # For now, this is a placeholder
        return True
    
    def update_program(self, name: str, program_data: Dict[str, Any]) -> bool:
        """Update an existing program"""
        if 'data' in program_data:
            return self.instance_store.update_data(name, program_data['data'])
        return False
    
    def delete_program(self, name: str) -> bool:
        """Delete a program"""
        return self.instance_store.delete(name)
    
    def get_templates(self) -> List[Dict[str, Any]]:
        """Get all program templates"""
        return self.template_store.get_all()
    
    def get_template(self, name: str) -> Optional[Dict[str, Any]]:
        """Get a specific template by name"""
        return self.template_store.get(name)
