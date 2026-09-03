import inspect
from pathlib import Path


def executing_program_dir() -> Path | None:
    """Directory of the first @nova.program file executing on the call stack."""

    # Imported here rather than at module scope: `nova.program.function` imports
    # `nova.datasets`, so a module-level import would close an import cycle and break
    # `import nova`. By the time this runs, a @nova.program is executing, so
    # `nova.program.function` is guaranteed to be imported already.
    from nova.program.function import Program

    frame = inspect.currentframe()
    try:
        frame = frame.f_back if frame is not None else None  # skip this helper's own frame
        while frame is not None:
            if frame.f_code is Program.__call__.__code__:
                program = frame.f_locals.get("self")
                if isinstance(program, Program):
                    code = getattr(inspect.unwrap(program._wrapped), "__code__", None)
                    return Path(code.co_filename).resolve().parent if code is not None else None
            frame = frame.f_back
    finally:
        del frame
