from novax.program_manager import ProgramManager


# Dependency to get the program manager
def get_program_manager() -> ProgramManager:
    """Dependency to get the program manager instance"""
    return ProgramManager()
