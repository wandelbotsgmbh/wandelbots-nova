# nova_python_app

This template contains a simple python app served by [fastapi](https://github.com/tiangolo/fastapi).
It shows you how to use the [NOVA Python SDK](https://github.com/wandelbotsgmbh/wandelbots-nova) and build a basic app with it.

## Features

- **Program Templates**: Define reusable program templates with Pydantic models
- **Program Instances**: Create, update, and manage program instances based on templates
- **SQLite Database**: Persistent storage for program templates and instances
- **RESTful API**: Full CRUD operations via FastAPI endpoints
- **Auto-discovery**: Automatic program template discovery from Python packages
- **Database Management**: Built-in backup and statistics functionality

## Development Setup

* make sure you have `uv` installed
    * you can follow these steps https://docs.astral.sh/uv/getting-started/installation/
* ensure proper environment variables are set in `.env`
    * note: you might need to set/update `NOVA_ACCESS_TOKEN` and `NOVA_API`
* use `uv run python -m nova_python_app` to run the the server
    * access the API documentation on `http://localhost:8000` (Stoplight UI)
    * access the OpenAPI spec on `http://localhost:8000/openapi.json`
* build, push and install the app with `nova app install`

## Database Storage

The application uses SQLite for persistent storage with a modular store architecture:

- **Database file**: `nova_programs.db` (created automatically in the working directory)
- **DatabaseConnection**: Manages database connection and initialization
- **ProgramTemplateStore**: Handles all template-related database operations
- **ProgramInstanceStore**: Handles all instance-related database operations
- **Automatic sync**: Templates are synced to database on startup
- **Backup support**: Built-in database backup functionality

### Store Architecture

**DatabaseConnection** (`db_connection`):
- Database initialization and connection management
- Database statistics and backup operations
- Shared by both stores for consistent connection handling

**ProgramTemplateStore** (`template_store`):
- `save(template)` - Save or update a template
- `get(name)` - Get template by name
- `get_all()` - Get all templates
- `delete(name)` - Delete template (with dependency check)

**ProgramInstanceStore** (`instance_store`):
- `save(instance)` - Save or update an instance
- `get(name)` - Get instance by name
- `get_all()` - Get all instances
- `get_by_template(template_name)` - Get instances for a template
- `update_data(name, data)` - Update instance data only
- `delete(name)` - Delete instance

### Database Schema

**program_templates** table:
- `name` (TEXT PRIMARY KEY): Template name
- `model_class_name` (TEXT): Python class name of the model
- `schema` (TEXT): JSON schema of the template
- `created_at`, `updated_at` (TIMESTAMP): Audit timestamps

**program_instances** table:
- `name` (TEXT PRIMARY KEY): Instance name
- `template_name` (TEXT): Reference to template
- `data` (TEXT): JSON data of the instance
- `created_at`, `updated_at` (TIMESTAMP): Audit timestamps

## API Endpoints

### Program Templates
- `GET /api/v2/program-templates` - List all templates
- `GET /api/v2/program-templates/{template_name}` - Get template details

### Program Instances  
- `GET /api/v2/programs` - List all program instances
- `GET /api/v2/programs/{program_name}` - Get instance details
- `POST /api/v2/programs/{program_name}` - Create new instance
- `PUT /api/v2/programs/{program_name}` - Update existing instance
- `DELETE /api/v2/programs/{program_name}` - Delete instance

### Database Management
- `GET /api/v2/database/stats` - Get database statistics
- `POST /api/v2/database/backup` - Create database backup
- `GET /api/v2/programs/template/{template_name}` - Get instances by template

## Testing

Run the database integration tests:

```bash
# Test database functionality
$ python test_database.py

# Test program discovery and integration
$ python test_program_integration.py
```

## Creating Program Templates

Create a new program by defining a Pydantic model and decorating a function:

```python
from nova_python_app.backbone import program
from nova_python_app.backbone.models import BaseProgramModel

class MyProgramModel(BaseProgramModel):
    name: str
    description: str = ""
    value: int = 42

@program(name="my_template", model=MyProgramModel)
async def my_program(model: MyProgramModel):
    print(f"Running program: {model.name}")
    # Your program logic here
```

## formatting

```bash
$ ruff format
$ ruff check --select I --fix
```