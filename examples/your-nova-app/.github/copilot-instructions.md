# Wandelbots Nova – Structure & Python Guidelines (Beginner‑Friendly)

> This project is powered by the **Wandelbots Nova Python SDK** and the **NovaX App Framework**

---

## 1) Project Structure (What lives where?)

A Nova app exposes a FastAPI service and one or more robot programs written in Python or WandelScript.

```
<app_name>/
├─ __main__.py           # FastAPI entry point + program registry (keep this small)
└─ programs/             # Where YOU write most code
   ├─ start_here.py      # Example Python program
   ├─ hello.ws           # Example WandelScript program
   └─ ws_extensions.py   # Python↔WandelScript bridge (FFI)
```

**Quick notes**

* Work mainly in **`programs/`**. Create simple, well‑named files (one program per file is fine).
* Keep `__main__.py` minimal: register routes/programs and delegate real logic to `programs/` files.
* If you expose functions to WandelScript, keep them tiny and well‑documented in `ws_extensions.py`.
* The app may be mounted under a variable **BASE\_PATH** in production (e.g., `/cell/<app-name>`). Avoid hard‑coding `/` in links or requests; rely on the framework’s resolved base path.

---

## 2) How we write Python here (simple & safe)

These guidelines are intentionally **beginner‑friendly**. Prefer clarity over cleverness.

### 2.1 Keep it simple

* Use **plain functions** and simple control flow (`for`, `if/else`).
* Prefer short files and short functions (aim ≤50 lines per function where possible).
* Avoid advanced Python features (metaclasses, decorators of your own, complex comprehensions, heavy class hierarchies).

### 2.2 Naming & types

* Use `lower_snake_case` for function and parameter names.
* Add **type hints** but keep them basic: `int`, `float`, `str`, `bool`, and simple `list[Type]`.
* Write a short **docstring** at the top of each program explaining what it does.

### 2.3 Async done right

* Program entrypoints should be `async def`.
* **Never** use `time.sleep(...)` in async code

### 2.4 Validate inputs early

* Check parameter ranges at the top (e.g., `if steps < 1: raise ValueError(...)`).
* Use safe defaults; avoid surprises.

### 2.5 Logging (not prints)

* Use the standard `logging` module, not `print`.
* Log what’s happening (step numbers, targets), not raw dumps of large data.

### 2.6 Errors

* Catch specific exceptions when you can act on them. Avoid `except Exception:` unless you re‑raise after logging.
* Surface clear messages that help users fix inputs instead of cryptic tracebacks.

### 2.7 Imports & dependencies

* Prefer the SDK and standard library first. Keep extra dependencies to a minimum.
* Avoid circular imports: split code by responsibility (API vs. programs vs. FFI).
*


# IMPORTANT:
When you execute terminal commands always tell the user why you need to run it.
So they can understand what you are doing.
