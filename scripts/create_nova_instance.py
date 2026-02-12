#!/usr/bin/env python3
"""
Description: Retrieve access token, create instance, create default cell,
             and verify that ProgramEngine is up.
"""

import asyncio
import os
import sys
from datetime import datetime

import requests
from loguru import logger

from nova import Nova, api
from nova.config import NovaConfig

# TODO use env or command line args for these
CELL_READINESS_TIMEOUT_SECS = 300.0
SERVICES_READINESS_TIMEOUT_SECS = 300.0


async def main() -> None:
    refresh_url = os.getenv("PORTAL_PROD_REFRESH_URL")
    client_id = os.getenv("PORTAL_PROD_REFRESH_CLIENT_ID")
    refresh_token = os.getenv("PORTAL_PROD_REFRESH_TOKEN")
    if not refresh_url or not client_id or not refresh_token:
        logger.error(
            "[ERROR] Missing one of the required environment variables: "
            "PORTAL_PROD_REFRESH_URL, PORTAL_PROD_REFRESH_CLIENT_ID, PORTAL_PROD_REFRESH_TOKEN"
        )
        sys.exit(1)

    get_token_response = requests.post(
        refresh_url,
        headers={"content-type": "application/x-www-form-urlencoded"},
        data={
            "grant_type": "refresh_token",
            "client_id": client_id,
            "refresh_token": refresh_token,
        },
    )
    get_token_response.raise_for_status()
    access_token = get_token_response.json().get("access_token")

    github_run_id = os.getenv("GITHUB_RUN_ID", "local-run")
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    sandbox_name = f"svcmgr-{github_run_id}-{timestamp}"

    response = requests.post(
        "https://api.portal.wandelbots.io/v1/instances",
        headers={
            "accept": "application/json",
            "Authorization": f"Bearer {access_token}",
            "Content-Type": "application/json",
        },
        json={"sandbox_name": sandbox_name},
    )
    response.raise_for_status()
    instance_response = response.json()

    host = instance_response.get("host")
    instance_id = instance_response.get("instance_id")

    config = NovaConfig(host=f"https://{host}", access_token=access_token)
    async with Nova(config) as nova:
        # wait for cell to be ready
        logger.info("Waiting for cell readiness (%fs)...", CELL_READINESS_TIMEOUT_SECS)
        async with asyncio.timeout(CELL_READINESS_TIMEOUT_SECS):
            while True:
                try:
                    cells = await nova.api.cell_api.list_cells()
                    if len(cells) > 0 and cells[0] == "cell":
                        logger.info("✅ Default cell 'cell' created.")
                        break
                except Exception as e:
                    logger.error(f"Waiting for default cell to be created: {e}")
                finally:
                    await asyncio.sleep(5)

        # wait for all services in the cell to be ready
        # TODO: could probably use nats
        logger.info("Waiting for cell readiness (%fs)...", SERVICES_READINESS_TIMEOUT_SECS)
        async with asyncio.timeout(SERVICES_READINESS_TIMEOUT_SECS):
            while True:
                try:
                    cell_status = await nova.api.cell_api.get_cell_status("cell")
                    all_running = all(
                        a_status.status.code == api.models.ServiceStatusPhase.RUNNING
                        for a_status in cell_status.service_status
                    )
                    if all_running:
                        logger.info("✅ All services in cell are running.")
                        break

                except Exception as e:
                    logger.error(f"Waiting for all services in cell to be running: {e}")
                finally:
                    await asyncio.sleep(5)

    with open("instance_config.env", "w") as file:
        file.write(
            f"""
PORTAL_PROD_ACCESS_TOKEN="{access_token}"
PORTAL_PROD_HOST="{host}"
PORTAL_PROD_INSTANCE_ID="{instance_id}"
"""
        )


if __name__ == "__main__":
    asyncio.run(main())
