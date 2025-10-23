import uvicorn
from decouple import config
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, RedirectResponse
from novax import Novax

from your_nova_app.start_here import start

CELL_ID = config("CELL_ID", default="cell", cast=str)
BASE_PATH = config("BASE_PATH", default="", cast=str)

# Create a new FastAPI app
# See https://fastapi.tiangolo.com/ for more information
app = FastAPI(
    title="your_nova_app",
    version="0.1.0",
    description="An application that serves your robot programs 🦾",
    root_path=BASE_PATH,
)

# Include the programs router and register the fist program
# See https://github.com/wandelbotsgmbh/wandelbots-nova/blob/main/README.md#novax for more information
novax = Novax()
novax.include_programs_router(app)
novax.register_program(start)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Add redirect from root to docs
@app.get("/")
async def root():
    return RedirectResponse(url="/docs")


@app.get("/app_icon.png", summary="Services the app icon for the homescreen")
async def get_app_icon():
    try:
        return FileResponse(path="static/app_icon.png", media_type="image/png")
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail="Icon not found")


def main(host: str = "0.0.0.0", port: int = 3000):
    uvicorn.run(
        app,
        host=host,
        port=port,
        reload=False,
        log_level="info",
        proxy_headers=True,
        forwarded_allow_ips="*",
    )
