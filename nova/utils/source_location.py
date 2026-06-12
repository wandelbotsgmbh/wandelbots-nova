"""Capture the exact source-code span of an action's call expression.

The interpreter exposes per-instruction position information (PEP 657,
available on Python 3.11+). This module turns that information into a
:class:`SourceLocation` so an editor can highlight one precise selection per
action (e.g. a multi-line ``circular(...)`` call) instead of a single line.
"""

import dis
import inspect
from types import CodeType, FrameType
from typing import Any

import pydantic


class SourceLocation(pydantic.BaseModel):
    """Exact source-code span of an action's call expression.

    Line numbers are 1-based and column offsets are 0-based, following the
    convention of the interpreter's position table (PEP 657, available on
    Python 3.11+). Any field may be ``None`` when the interpreter cannot
    provide that piece of information (for example for code executed from a
    synthetic source such as ``exec`` or a doctest).

    This represents the *full* selection of the call, e.g. for a multi-line
    ``circular(...)`` call ``start_line``/``end_line`` cover every line of the
    call and ``start_column``/``end_column`` the exact columns. This lets an
    editor highlight one precise selection per action instead of a single line.
    """

    start_line: int | None = None
    end_line: int | None = None
    start_column: int | None = None
    end_column: int | None = None


def _instruction_positions(
    code: CodeType, lasti: int
) -> tuple[int | None, int | None, int | None, int | None] | None:
    """Return the ``(start_line, end_line, start_col, end_col)`` of the bytecode
    instruction at offset ``lasti`` in ``code``, or ``None`` if unavailable.

    ``lasti`` may point inside the inline cache region that follows an
    instruction (e.g. the caches after ``CALL``), which ``dis.get_instructions``
    does not emit. We therefore select the instruction with the greatest offset
    not exceeding ``lasti`` (instructions are yielded in increasing offset order).
    """
    try:
        candidate = None
        for instruction in dis.get_instructions(code):
            if instruction.offset <= lasti:
                candidate = instruction
            else:
                break
        if candidate is None:
            return None
        positions = candidate.positions
        if positions is None:
            return None
        return (
            positions.lineno,
            positions.end_lineno,
            positions.col_offset,
            positions.end_col_offset,
        )
    except Exception:
        return None


def source_location_for_frame(frame: FrameType | None) -> SourceLocation | None:
    """Build a :class:`SourceLocation` for the call currently executing in ``frame``.

    Returns ``None`` for synthetic frames (``<doctest ...>``, ``<string>``,
    ``<stdin>``) that have no real source file to highlight, or when the
    interpreter does not provide position information.
    """
    if frame is None:
        return None
    code = frame.f_code
    if code.co_filename.startswith("<"):
        return None
    positions = _instruction_positions(code, frame.f_lasti)
    if positions is None:
        return None
    start_line, end_line, start_column, end_column = positions
    if start_line is None:
        return None
    return SourceLocation(
        start_line=start_line, end_line=end_line, start_column=start_column, end_column=end_column
    )


def get_caller_metas() -> dict[str, Any]:
    """Capture source-location metadata for the caller of the function that
    calls this helper.

    Intended to be called from a one-level wrapper (e.g. an action factory such
    as :func:`nova.actions.linear`); the returned dict describes the *user* call
    site of that factory.

    Returns a dict that always contains ``line_number`` (kept for backward
    compatibility) and additionally ``source_location`` (the exact span as a
    plain dict) when a real source location is available.
    """
    frame = inspect.currentframe()
    try:
        caller = frame.f_back.f_back if frame is not None and frame.f_back is not None else None
        metas: dict[str, Any] = {"line_number": caller.f_lineno if caller is not None else None}
        source_location = source_location_for_frame(caller)
        if source_location is not None:
            metas["source_location"] = source_location.model_dump()
        return metas
    finally:
        del frame
