from fastapi import APIRouter, Body, Depends, HTTPException, Path

from novax.api.dependencies import get_program_manager
from novax.program_manager import ProgramDetails, ProgramManager, ProgramRun, RunProgramRequest

router = APIRouter(prefix="/programs", tags=["programs"])


@router.get("", operation_id="getPrograms", response_model=list[ProgramDetails])
async def get_programs(program_manager: ProgramManager = Depends(get_program_manager)):
    """List all programs"""
    programs = await program_manager.get_programs()
    return list(programs.values())


@router.get("/{program}", operation_id="getProgram")
async def get_program(
    program: str = Path(..., description="The ID of the program"),
    program_manager: ProgramManager = Depends(get_program_manager),
):
    """Get program details"""
    program_details = await program_manager.get_program(program)
    if not program_details:
        raise HTTPException(status_code=404, detail="Program not found")

    program_fn = program_manager._program_functions[program]

    return {
        **program_details.model_dump(),
        "input_schema": program_fn.json_schema,
        "_links": {
            "self": {"href": f"/programs/{program}", "method": "GET"},
            "runs": {"href": f"/programs/{program}/runs", "method": "POST"},
        },
    }


@router.get("/{program}/runs", operation_id="getProgramRuns", response_model=list[ProgramRun])
async def get_program_runs(
    program: str = Path(..., description="The ID of the program"),
    program_manager: ProgramManager = Depends(get_program_manager),
):
    """List all program runs"""
    if not await program_manager.get_program(program):
        raise HTTPException(status_code=404, detail="Program not found")

    return await program_manager.get_program_runs(program)


@router.post("/{program}/runs", operation_id="runProgram", response_model=ProgramRun)
async def run_program(
    program: str = Path(..., description="The ID of the program"),
    request: RunProgramRequest = Body(...),
    program_manager: ProgramManager = Depends(get_program_manager),
):
    """Run a program"""
    if not await program_manager.get_program(program):
        raise HTTPException(status_code=404, detail="Program not found")

    return await program_manager.run_program(program, request.parameters)


@router.get("/{program}/runs/{run}", operation_id="getProgramRun")
async def get_program_run(
    program: str = Path(..., description="The ID of the program"),
    run: str = Path(..., description="The ID of the run"),
    program_manager: ProgramManager = Depends(get_program_manager),
):
    """Get state of the run"""
    if not await program_manager.get_program(program):
        raise HTTPException(status_code=404, detail="Program not found")

    program_run = await program_manager.get_program_run(program, run)
    return {
        **program_run.model_dump(),
        "_links": {
            "self": {"href": f"/programs/{program}/runs/{run}", "method": "GET"},
            "stop": {"href": f"/programs/{program}/runs/{run}/stop", "method": "POST"},
        },
    }


@router.post("/{program}/runs/{run}/stop", operation_id="stopProgramRun", status_code=204)
async def stop_program_run(
    program: str = Path(..., description="The ID of the program"),
    run: str = Path(..., description="The ID of the run"),
    program_manager: ProgramManager = Depends(get_program_manager),
):
    """Stop the run"""
    if not await program_manager.get_program(program):
        raise HTTPException(status_code=404, detail="Program not found")

    await program_manager.stop_program(program, run)
    return None
