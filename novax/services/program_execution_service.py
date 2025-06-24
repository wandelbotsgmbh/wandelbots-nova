"""
Program Execution Service

Service for executing programs using different processors.
"""

from typing import Any

from ..decorators import REGISTERED_PROGRAM_TEMPLATES
from ..interfaces import (
    ProgramExecutionServiceInterface,
    ProgramInstanceStoreInterface,
    ProgramRunProcessorInterface,
    ProgramRunStoreInterface,
    ProgramTemplateStoreInterface,
)


class ProgramExecutionService(ProgramExecutionServiceInterface):
    """Service for executing programs using different processors"""

    def __init__(
        self,
        processor: ProgramRunProcessorInterface,
        template_store: ProgramTemplateStoreInterface,
        instance_store: ProgramInstanceStoreInterface,
        run_store: ProgramRunStoreInterface,
    ):
        self.processor = processor
        self.template_store = template_store
        self.instance_store = instance_store
        self.run_store = run_store

    async def execute_program(
        self,
        program_name: str,
        run_id: str,
        parameters: dict[str, Any],
        environment_variables: dict[str, str],
    ) -> dict[str, Any]:
        """
        Execute a program using the configured processor

        Args:
            program_name: Name of the program to execute
            run_id: Unique identifier for this run
            parameters: Runtime parameters
            environment_variables: Environment variables

        Returns:
            Dict containing execution results
        """
        try:
            # Update status to running
            self.run_store.update_status(run_id, "running")

            # Get program instance data
            instance_data = self.instance_store.get(program_name)
            if not instance_data:
                raise ValueError(f"Program instance '{program_name}' not found")

            # Get template data
            template_name = instance_data.get("template_name")
            template_data = self.template_store.get(template_name) if template_name else {}

            # Get the program template from registered templates
            program_template = None
            for template in REGISTERED_PROGRAM_TEMPLATES.values():
                if template.name == template_name:
                    program_template = template
                    break

            if not program_template:
                raise ValueError(
                    f"Program template '{template_name}' not found in registered templates"
                )

            # Create model instance with combined data
            combined_data = {**instance_data.get("data", {}), **parameters}
            model_instance = program_template.model_class(**combined_data)

            # Execute using the processor
            result = await self.processor.execute_program(
                program_name=program_name,
                run_id=run_id,
                program_function=program_template.function,
                program_model_instance=model_instance,
                template_data=template_data,
                instance_data=instance_data,
                parameters=parameters,
                environment_variables=environment_variables,
            )

            # Update run status based on result
            if result.get("status") == "completed":
                self.run_store.update_status(
                    run_id,
                    "completed",
                    finished_at=self._get_current_time(),
                    output=result.get("output"),
                    exit_code=result.get("exit_code", 0),
                )
            else:
                self.run_store.update_status(
                    run_id,
                    "failed",
                    finished_at=self._get_current_time(),
                    error_message=result.get("error_message"),
                    exit_code=result.get("exit_code", 1),
                )

            return result

        except Exception as e:
            # Update status to failed
            self.run_store.update_status(
                run_id,
                "failed",
                finished_at=self._get_current_time(),
                error_message=str(e),
                exit_code=1,
            )

            return {
                "status": "failed",
                "output": None,
                "error_message": str(e),
                "exit_code": 1,
                "execution_time": 0,
                "processor_type": "unknown",
            }

    def _get_current_time(self) -> str:
        """Get current time in ISO format"""
        from datetime import datetime

        return datetime.utcnow().isoformat()
