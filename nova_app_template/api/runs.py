"""
Program Run API

This module contains API endpoints for running and executing programs.
"""

import uuid
from datetime import datetime
from typing import Dict, List, Any, Optional
from fastapi import APIRouter, HTTPException, BackgroundTasks, Depends
from pydantic import BaseModel
from dependency_injector.wiring import inject, Provide

from nova_python_app.backbone.container import NovaContainer
from nova_python_app.backbone.interfaces import (
    ProgramRunAPIServiceInterface,
    ProgramInstanceStoreInterface,
    ProgramRunStoreInterface
)
from nova_python_app.backbone.services import ProgramExecutionService

# Request/Response models
class ProgramRunRequest(BaseModel):
    """Request body for program run configuration"""
    parameters: Dict[str, Any] = {}
    environment_variables: Dict[str, str] = {}

class ProgramRunResponse(BaseModel):
    """Response model for program run creation"""
    run_id: str
    program_name: str
    status: str
    started_at: str
    parameters: Dict[str, Any]
    environment_variables: Dict[str, str]
    message: str

class ProgramRunStatus(BaseModel):
    """Response model for program run status"""
    run_id: str
    program_name: str
    status: str
    started_at: str
    finished_at: Optional[str]
    parameters: Dict[str, Any]
    environment_variables: Dict[str, str]
    output: Optional[str]
    error_message: Optional[str]
    exit_code: Optional[int]

class ProgramRunList(BaseModel):
    """Response model for listing program runs"""
    runs: List[ProgramRunStatus]
    total_count: int

# Router for program run endpoints
router = APIRouter()


async def execute_program_async(
    program_name: str, 
    run_id: str, 
    execution_service,
    run_service,
    parameters: Dict[str, Any] = None, 
    env_vars: Dict[str, str] = None
):
    """
    Async function to handle the actual program execution using dependency injection.
    This showcases the power of the dependency injection system with different processors.
    """
    try:
        # Execute the program using the configured processor
        result = await execution_service.execute_program(
            program_name=program_name,
            run_id=run_id,
            parameters=parameters or {},
            environment_variables=env_vars or {}
        )
        
        print(f"Program execution completed with processor: {result.get('processor_type', 'unknown')}")
        
    except Exception as e:
        print(f"Program execution failed: {str(e)}")
        # Update status to failed as fallback
        if run_service:
            try:
                finished_at = datetime.utcnow().isoformat()
                run_service.update_run_status(
                    run_id, 
                    "failed", 
                    finished_at=finished_at,
                    error_message=str(e),
                    exit_code=1
                )
            except Exception as update_error:
                print(f"Failed to update run status: {str(update_error)}")
        else:
            print("Warning: Run service not available for error handling")


@router.post("/programs/{program_name}/runs", response_model=ProgramRunResponse)
@inject
async def create_program_run(
    program_name: str,
    background_tasks: BackgroundTasks,
    request_body: ProgramRunRequest = ProgramRunRequest(),
    run_service: ProgramRunAPIServiceInterface = Depends(Provide[NovaContainer.services.program_run_service]),
    instance_store: ProgramInstanceStoreInterface = Depends(Provide[NovaContainer.stores.program_instance_store]),
    execution_service: ProgramExecutionService = Depends(Provide[NovaContainer.services.program_execution_service])
):
    """
    Create and start a new program run.
    Returns immediately with run_id while execution happens in background.
    """
    # Validate that program exists using injected instance store
    program = instance_store.get(program_name)
    if not program:
        raise HTTPException(
            status_code=404, 
            detail=f"Program '{program_name}' not found"
        )
    
    # Generate unique run ID
    run_id = str(uuid.uuid4())
    started_at = datetime.utcnow().isoformat()
    
    # Create program run record
    run_data = {
        "run_id": run_id,
        "program_name": program_name,
        "status": "pending",
        "started_at": started_at,
        "parameters": request_body.parameters,
        "environment_variables": request_body.environment_variables
    }
    
    # Save the program run using the service
    created_run_id = run_service.create_run(run_data)
    if not created_run_id:
        raise HTTPException(
            status_code=500, 
            detail="Failed to save program run to database"
        )
    
    # Delegate program execution to background task
    background_tasks.add_task(
        execute_program_async,
        program_name,
        run_id,
        execution_service,
        run_service,
        request_body.parameters,
        request_body.environment_variables
    )
    
    return ProgramRunResponse(
        run_id=run_id,
        program_name=program_name,
        status="pending",
        started_at=started_at,
        parameters=request_body.parameters,
        environment_variables=request_body.environment_variables,
        message=f"Program run '{run_id}' created and queued for execution"
    )


@router.get("/programs/{program_name}/runs/{run_id}", response_model=ProgramRunStatus)
@inject
async def get_program_run_status(
    program_name: str, 
    run_id: str,
    run_service: ProgramRunAPIServiceInterface = Depends(Provide[NovaContainer.services.program_run_service])
):
    """Get the status of a specific program run"""
    run_data = run_service.get_run(run_id)
    if not run_data:
        raise HTTPException(
            status_code=404, 
            detail=f"Program run '{run_id}' not found"
        )
    
    # Verify the run belongs to the specified program
    if run_data['program_name'] != program_name:
        raise HTTPException(
            status_code=404, 
            detail=f"Program run '{run_id}' not found for program '{program_name}'"
        )
    
    return ProgramRunStatus(**run_data)


@router.get("/programs/{program_name}/runs", response_model=ProgramRunList)
@inject
async def list_program_runs(
    program_name: str, 
    limit: int = 10, 
    offset: int = 0,
    run_service: ProgramRunAPIServiceInterface = Depends(Provide[NovaContainer.services.program_run_service]),
    instance_store: ProgramInstanceStoreInterface = Depends(Provide[NovaContainer.stores.program_instance_store])
):
    """List all program runs for a specific program"""
    # Validate that program exists using injected instance store
    program = instance_store.get(program_name)
    if not program:
        raise HTTPException(
            status_code=404, 
            detail=f"Program '{program_name}' not found"
        )
    
    runs = run_service.get_runs(program_name)
    
    # Apply pagination
    total_count = len(runs)
    paginated_runs = runs[offset:offset + limit]
    
    return ProgramRunList(
        runs=[ProgramRunStatus(**run) for run in paginated_runs],
        total_count=total_count
    )


@router.get("/runs", response_model=ProgramRunList)
@inject
async def list_all_program_runs(
    limit: int = 10, 
    offset: int = 0,
    run_service: ProgramRunAPIServiceInterface = Depends(Provide[NovaContainer.services.program_run_service])
):
    """List all program runs across all programs"""
    runs = run_service.get_runs()
    
    # Apply pagination
    total_count = len(runs)
    paginated_runs = runs[offset:offset + limit]
    
    return ProgramRunList(
        runs=[ProgramRunStatus(**run) for run in paginated_runs],
        total_count=total_count
    )


@router.delete("/programs/{program_name}/runs/{run_id}")
@inject
async def delete_program_run(
    program_name: str, 
    run_id: str,
    run_service: ProgramRunAPIServiceInterface = Depends(Provide[NovaContainer.services.program_run_service]),
    run_store: ProgramRunStoreInterface = Depends(Provide[NovaContainer.stores.program_run_store])
):
    """Delete a specific program run"""
    run_data = run_service.get_run(run_id)
    if not run_data:
        raise HTTPException(
            status_code=404, 
            detail=f"Program run '{run_id}' not found"
        )
    
    # Verify the run belongs to the specified program
    if run_data['program_name'] != program_name:
        raise HTTPException(
            status_code=404, 
            detail=f"Program run '{run_id}' not found for program '{program_name}'"
        )
    
    # Don't allow deletion of running programs
    if run_data['status'] == 'running':
        raise HTTPException(
            status_code=400, 
            detail="Cannot delete a running program. Stop it first."
        )
    
    # Use injected run store
    success = run_store.delete(run_id)
    if not success:
        raise HTTPException(
            status_code=500, 
            detail="Failed to delete program run"
        )
    
    return {"message": f"Program run '{run_id}' deleted successfully"}


def register_program_run_routes(app):
    """Register all program run-related routes to the FastAPI app"""
    app.include_router(router, prefix="/api/v2", tags=["program-runs"])
