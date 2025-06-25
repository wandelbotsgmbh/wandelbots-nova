import uvicorn

from novax import Novax

if __name__ == "__main__":
    novax = Novax()
    novax.discover_programs()
    app = novax.create_app()

    uvicorn.run(app, host="0.0.0.0", port=8000, reload=False)
