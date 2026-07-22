from pathlib import Path

import uvicorn
from decouple import config
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, HTMLResponse

from nova import Novax

CELL_ID = config("CELL_ID", default="cell", cast=str)
BASE_PATH = config("BASE_PATH", default="", cast=str)

# Create a new FastAPI app
# See https://fastapi.tiangolo.com/ for more information
app = FastAPI(
    title="Your NOVA App",
    version="0.1.0",
    description="An application that serves your robot programs 🦾",
    root_path=BASE_PATH,
)

# Include the programs router and scan the ``programs`` directory: every @nova.program
# module under it self-registers on import, so dropping in a new file is enough -- no
# manual import required. The path is anchored to this file (not the working directory)
# so it resolves correctly both locally and in the deployed container.
# See https://github.com/wandelbotsgmbh/wandelbots-nova/blob/main/README.md#novax for more information
novax = Novax(app)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Add redirect from root to docs
@app.get("/", summary="Opens the interactive API", response_class=HTMLResponse)
async def root():
    # One could serve a nice UI here as well. For simplicity, we just redirect to the Stoplight UI.
    return f"""
    <!doctype html>
        <html lang="en">
          <head>
            <meta charset="utf-8">
            <meta name="viewport" content="width=device-width, initial-scale=1, shrink-to-fit=no">
            <title>Elements in HTML</title>
            <!-- Embed elements Elements via Web Component -->
            <script src="https://unpkg.com/@stoplight/elements/web-components.min.js"></script>
            <link rel="stylesheet" href="https://unpkg.com/@stoplight/elements/styles.min.css">
          </head>
          <body>
            <elements-api
              apiDescriptionUrl="{BASE_PATH}/openapi.json"
              router="hash"
              layout="sidebar"
              tryItCredentialsPolicy="same-origin"
            />
          </body>
    </html>
    """


@app.get("/app_icon.png", summary="Services the app icon for the homescreen")
async def get_app_icon():
    try:
        return FileResponse(path="static/app_icon.png", media_type="image/png")
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail="Icon not found")


def main(host: str = "0.0.0.0", port: int = 3000):
    # In-cluster dev (Skaffold sets DEV_RELOAD=true) runs uvicorn with --reload so
    # synced program files hot-reload. The app-operator owns the Deployment and uses
    # the image's default command, so reload is opted into via env instead of args.
    if config("DEV_RELOAD", default=False, cast=bool):
        uvicorn.run(
            "app.register_programs:app",
            host=host,
            port=port,
            reload=True,
            reload_dirs=["/app/app"],
            log_level="info",
            proxy_headers=True,
            forwarded_allow_ips="*",
        )
        return
    uvicorn.run(
        app,
        host=host,
        port=port,
        reload=False,
        log_level="info",
        proxy_headers=True,
        forwarded_allow_ips="*",
    )


if __name__ == "__main__":
    main()
