from novax.config import APP_NAME, CELL_NAME
from novax.program_manager import ProgramManager


# Dependency to get the program manager
def get_program_manager() -> ProgramManager:
    """Dependency to get the program manager instance"""
    return ProgramManager(cell_id=CELL_NAME, app_name=APP_NAME)
