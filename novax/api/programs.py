from fastapi import APIRouter, Body, Depends, HTTPException, Path
from wandelbots_api_client.v2.models.program import Program
from wandelbots_api_client.v2.models.program_start_request import ProgramStartRequest

from novax.api.dependencies import get_program_manager
from novax.program_manager import ProgramManager, ProgramRun

router = APIRouter(prefix="/programs", tags=["programs"])


@router.get("", operation_id="getPrograms", response_model=list[Program])
async def get_programs(program_manager: ProgramManager = Depends(get_program_manager)):
    """List all programs"""
    programs = await program_manager.get_programs()
    return [program_definition for _, program_definition in programs.items()]


@router.get("/{program}", operation_id="getProgram", response_model=Program)
async def get_program(
    program: str = Path(..., description="The ID of the program"),
    program_manager: ProgramManager = Depends(get_program_manager),
):
    """Get program details"""
    program_definition = await program_manager.get_program(program)
    if not program_definition:
        raise HTTPException(status_code=404, detail="Program not found")

    return program_definition


@router.post("/{program}/start", operation_id="startProgram", response_model=ProgramRun)
async def start_program(
    program: str = Path(..., description="The ID of the program"),
    request: ProgramStartRequest = Body(...),
    program_manager: ProgramManager = Depends(get_program_manager),
):
    """Run a program"""
    if not await program_manager.get_program(program):
        raise HTTPException(status_code=404, detail="Program not found")

    if program_manager.running_program:
        raise HTTPException(status_code=400, detail="A program is already running")

    return await program_manager.start_program(program, request.arguments)


@router.post("/{program}/stop", operation_id="stopProgram")
async def stop_program(
    program: str = Path(..., description="The ID of the program"),
    program_manager: ProgramManager = Depends(get_program_manager),
):
    """Stop the run"""
    if not await program_manager.get_program(program):
        raise HTTPException(status_code=404, detail="Program not found")

    if not program_manager.running_program:
        raise HTTPException(status_code=400, detail="No program is running")

    if program_manager.running_program != program:
        raise HTTPException(
            status_code=400,
            detail="Program is not running. Currently running: {program_manager.running_program}",
        )

    await program_manager.stop_program(program)
    return None
