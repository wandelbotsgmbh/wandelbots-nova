import uvicorn

from mock_camera_server.app import app

uvicorn.run(app, host="0.0.0.0", port=9100)
