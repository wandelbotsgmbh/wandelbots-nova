from novax import Novax
from novax.api.dependencies import get_program_manager


def test_novax():
    novax = Novax()
    app = novax.create_app()
    assert app is not None
    novax.include_programs_router(app)
    assert app.dependency_overrides[get_program_manager] is not None
    assert app.routes is not None
