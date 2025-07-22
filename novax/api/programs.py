from typing import Any

from fastapi import APIRouter, Body, Depends, HTTPException, Path

from novax.api.dependencies import get_program_manager
from novax.program_manager import ProgramDetails, ProgramManager, ProgramRun, RunProgramRequest

router = APIRouter(prefix="/programs", tags=["programs"])


class ProgramResponse(ProgramDetails):
    input_schema: dict[str, Any]


@router.get("", operation_id="getPrograms", response_model=list[ProgramResponse])
async def get_programs(program_manager: ProgramManager = Depends(get_program_manager)):
    """List all programs"""
    programs = await program_manager.get_programs()
    return [
        ProgramResponse(
            **program_details.model_dump(),
            input_schema=program_manager._program_functions[program].json_schema,
        )
        for program, program_details in programs.items()
    ]


@router.get("/{program}", operation_id="getProgram", response_model=ProgramResponse)
async def get_program(
    program: str = Path(..., description="The ID of the program"),
    program_manager: ProgramManager = Depends(get_program_manager),
):
    """Get program details"""
    program_details = await program_manager.get_program(program)
    if not program_details:
        raise HTTPException(status_code=404, detail="Program not found")

    program_fn = program_manager._program_functions[program]

    return ProgramResponse(**program_details.model_dump(), input_schema=program_fn.json_schema)


@router.post("/{program}/start", operation_id="startProgram", response_model=ProgramRun)
async def start_program(
    program: str = Path(..., description="The ID of the program"),
    request: RunProgramRequest = Body(...),
    program_manager: ProgramManager = Depends(get_program_manager),
):
    """Run a program"""
    if not await program_manager.get_program(program):
        raise HTTPException(status_code=404, detail="Program not found")

    if program_manager.running_program:
        raise HTTPException(status_code=400, detail="A program is already running")

    return await program_manager.start_program(program, request.parameters)


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
