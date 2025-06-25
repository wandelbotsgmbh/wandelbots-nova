from typing import Any

from dependency_injector.wiring import Provide, inject
from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel

from novax.container import NovaContainer
from novax.interfaces import (
    DatabaseConnectionInterface,
    ProgramAPIServiceInterface,
    ProgramInstanceStoreInterface,
    ProgramTemplateStoreInterface,
)
from novax.store.models import ProgramInstance


# Response Models
class ProgramTemplateListResponse(BaseModel):
    program_templates: list[str]


class ProgramTemplateDetailResponse(BaseModel):
    name: str
    model_schema: dict[str, Any]


class ProgramListResponse(BaseModel):
    programs: list[str]


class CreateProgramRequest(BaseModel):
    template_name: str
    data: dict[str, Any]


class ProgramDetailResponse(BaseModel):
    name: str
    template_name: str
    data: dict[str, Any]


class DatabaseStatsResponse(BaseModel):
    template_count: int
    instance_count: int
    database_path: str


class BackupResponse(BaseModel):
    message: str
    backup_path: str


# Router for program endpoints
router = APIRouter()


# API Endpoints
@router.get("/program-templates", response_model=ProgramTemplateListResponse)
@inject
async def get_program_templates(
    template_store: ProgramTemplateStoreInterface = Depends(
        Provide[NovaContainer.stores.program_template_store]
    ),
):
    """Get list of all available program template names"""
    templates = template_store.get_all()
    template_names = [template["name"] for template in templates]
    return ProgramTemplateListResponse(program_templates=template_names)


@router.get("/program-templates/{template_name}", response_model=ProgramTemplateDetailResponse)
@inject
async def get_program_template_detail(
    template_name: str,
    template_store: ProgramTemplateStoreInterface = Depends(
        Provide[NovaContainer.stores.program_template_store]
    ),
):
    """Get detailed information about a specific program template"""
    template = template_store.get(template_name)
    if not template:
        raise HTTPException(status_code=404, detail=f"Program template '{template_name}' not found")

    return ProgramTemplateDetailResponse(
        name=template_name, model_schema=template.get("schema", {})
    )


@router.get("/programs", response_model=ProgramListResponse)
@inject
async def get_programs(
    program_service: ProgramAPIServiceInterface = Depends(
        Provide[NovaContainer.services.program_service]
    ),
):
    """Get list of all created program instance names"""
    instances = program_service.get_programs()
    program_names = [instance["name"] for instance in instances]
    return ProgramListResponse(programs=program_names)


@router.get("/programs/{program_name}", response_model=ProgramDetailResponse)
@inject
async def get_program_detail(
    program_name: str,
    program_service: ProgramAPIServiceInterface = Depends(
        Provide[NovaContainer.services.program_service]
    ),
):
    """Get detailed information about a specific program instance"""
    instance_data = program_service.get_program(program_name)
    if not instance_data:
        raise HTTPException(status_code=404, detail=f"Program instance '{program_name}' not found")

    return ProgramDetailResponse(
        name=instance_data["name"],
        template_name=instance_data["template_name"],
        data=instance_data["data"],
    )


@router.post("/programs/{program_name}")
@inject
async def create_program(
    program_name: str,
    request: CreateProgramRequest,
    program_service: ProgramAPIServiceInterface = Depends(
        Provide[NovaContainer.services.program_service]
    ),
    instance_store: ProgramInstanceStoreInterface = Depends(
        Provide[NovaContainer.stores.program_instance_store]
    ),
    template_store: ProgramTemplateStoreInterface = Depends(
        Provide[NovaContainer.stores.program_template_store]
    ),
):
    """Create a new program instance with data based on a template"""
    template_data = template_store.get(request.template_name)
    if not template_data:
        raise HTTPException(
            status_code=404, detail=f"Program template '{request.template_name}' not found"
        )

    # Check if program already exists
    existing_instance = program_service.get_program(program_name)
    if existing_instance:
        raise HTTPException(
            status_code=409, detail=f"Program instance '{program_name}' already exists"
        )

    # Get the actual template object from the store
    templates = template_store.get_all()
    template = None
    for t in templates:
        if t["name"] == request.template_name:
            template = t
            break

    if not template:
        raise HTTPException(
            status_code=404, detail=f"Template object for '{request.template_name}' not found"
        )

    try:
        request.data["name"] = program_name
        # Validate the data against the model
        template.model_class(**request.data)

        # Create and store the program instance
        program_instance = ProgramInstance(program_name, template, request.data)
        # Using the injected instance store
        success = instance_store.save(program_instance)

        if not success:
            raise HTTPException(
                status_code=500,
                detail=f"Failed to save program instance '{program_name}' to database",
            )

        return {
            "message": f"Program instance '{program_name}' created successfully",
            "template_name": request.template_name,
            "data": request.data,
        }

    except Exception as e:
        import traceback

        error_details = {
            "error": str(e),
            "type": type(e).__name__,
            "traceback": traceback.format_exc(),
        }
        raise HTTPException(
            status_code=400,
            detail=f"Invalid data for template '{request.template_name}': {error_details}",
        )


@router.put("/programs/{program_name}")
@inject
async def update_program(
    program_name: str,
    data: dict[str, Any],
    program_service: ProgramAPIServiceInterface = Depends(
        Provide[NovaContainer.services.program_service]
    ),
    template_store: ProgramTemplateStoreInterface = Depends(
        Provide[NovaContainer.stores.program_template_store]
    ),
):
    """Update an existing program instance with new data"""
    # Check if program exists
    existing_instance = program_service.get_program(program_name)
    if not existing_instance:
        raise HTTPException(status_code=404, detail=f"Program instance '{program_name}' not found")

    template_name = existing_instance["template_name"]
    template_data = template_store.get(template_name)
    if not template_data:
        raise HTTPException(
            status_code=500, detail=f"Template '{template_name}' not found in template store"
        )

    # Get the actual template object from the store
    templates = template_store.list()
    template = None
    for t in templates:
        if t.name == template_name:
            template = t
            break

    if not template:
        raise HTTPException(
            status_code=500, detail=f"Template object for '{template_name}' not found"
        )

    try:
        data["name"] = program_name
        template.model_class(**data)

        # Update using the service
        success = program_service.update_program(program_name, {"data": data})
        if not success:
            raise HTTPException(
                status_code=500,
                detail=f"Failed to update program instance '{program_name}' in database",
            )

        return {"message": f"Program instance '{program_name}' updated successfully", "data": data}

    except Exception as e:
        raise HTTPException(
            status_code=400, detail=f"Invalid data for program instance '{program_name}': {str(e)}"
        )


@router.delete("/programs/{program_name}")
@inject
async def delete_program(
    program_name: str,
    program_service: ProgramAPIServiceInterface = Depends(
        Provide[NovaContainer.services.program_service]
    ),
):
    """Delete a program instance"""
    # Check if program exists
    existing_instance = program_service.get_program(program_name)
    if not existing_instance:
        raise HTTPException(status_code=404, detail=f"Program instance '{program_name}' not found")

    # Delete using the service
    success = program_service.delete_program(program_name)
    if not success:
        raise HTTPException(
            status_code=500,
            detail=f"Failed to delete program instance '{program_name}' from database",
        )

    return {"message": f"Program instance '{program_name}' deleted successfully"}


@router.get("/database/stats", response_model=DatabaseStatsResponse)
@inject
async def get_database_stats(
    db_connection: DatabaseConnectionInterface = Depends(
        Provide[NovaContainer.stores.database_connection]
    ),
):
    """Get database statistics"""
    stats = db_connection.get_database_stats()
    return DatabaseStatsResponse(
        template_count=stats["template_count"],
        instance_count=stats["instance_count"],
        database_path=db_connection.db_path,
    )


@router.get("/programs/template/{template_name}", response_model=ProgramListResponse)
@inject
async def get_programs_by_template(
    template_name: str,
    program_service: ProgramAPIServiceInterface = Depends(
        Provide[NovaContainer.services.program_service]
    ),
    instance_store: ProgramInstanceStoreInterface = Depends(
        Provide[NovaContainer.stores.program_instance_store]
    ),
    template_store: ProgramTemplateStoreInterface = Depends(
        Provide[NovaContainer.stores.program_template_store]
    ),
):
    """Get all program instances for a specific template"""
    template_data = template_store.get(template_name)
    if not template_data:
        raise HTTPException(status_code=404, detail=f"Program template '{template_name}' not found")

    # Using the injected instance store
    instances = instance_store.get_by_template(template_name)
    program_names = [instance["name"] for instance in instances]
    return ProgramListResponse(programs=program_names)


def register_program_routes(app):
    """Register all program-related routes to the FastAPI app"""
    app.include_router(router, prefix="/api/v2", tags=["programs"])
