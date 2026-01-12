import contextvars

current_program_context_var: contextvars.ContextVar["ProgramContext | None"] = (
    contextvars.ContextVar("current_program_context_var", default=None)
)
