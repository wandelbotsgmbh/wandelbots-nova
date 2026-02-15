#!/usr/bin/env python3
"""
Description: Retrieve access token, create instance, create default cell,
             and verify that all services are up using NATS subscriptions.
"""

import asyncio
import json
import os
import sys
from datetime import datetime

import requests
from loguru import logger

from nova import Nova
from nova.config import NovaConfig

# TODO use env or command line args for these
CELL_READINESS_TIMEOUT_SECS = 180.0
SERVICES_READINESS_TIMEOUT_SECS = 180.0


async def wait_for_services_ready(nova: Nova, cell: str, timeout: float) -> None:
    """
    Wait for all services in a cell to be running using NATS subscription.

    Args:
        nova: The Nova client instance.
        cell: The cell name to monitor.
        timeout: Maximum time to wait in seconds.
    """
    cell_status_subject = f"nova.v2.cells.{cell}.status"
    all_services_ready = asyncio.Event()

    async def on_cell_status_message(msg):
        if all_services_ready.is_set():
            return
        try:
            data = json.loads(msg.data)
            if not data:
                return

            # Check if all services are running
            all_running = all(
                service.get("status", {}).get("code") == "Running" for service in data
            )
            if all_running:
                service_count = len(data)
                logger.info(f"✅ All {service_count} services in cell '{cell}' are running.")
                all_services_ready.set()
            else:
                not_running = [
                    f"{s['service']}: {s.get('status', {}).get('code')}"
                    for s in data
                    if s.get("status", {}).get("code") != "Running"
                ]
                logger.debug(f"Services not ready: {not_running}")
        except json.JSONDecodeError as e:
            logger.warning(f"Failed to parse status message: {e}")

    sub = await nova.nats.subscribe(subject=cell_status_subject, cb=on_cell_status_message)
    logger.info(f"Subscribed to {cell_status_subject}")
    try:
        await asyncio.wait_for(all_services_ready.wait(), timeout=timeout)
    except asyncio.TimeoutError:
        logger.error(f"Timeout waiting for services in cell '{cell}' to be ready")
        raise TimeoutError(f"Services not ready within {timeout}s")
    finally:
        await sub.unsubscribe()


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
        # Wait for cell to exist (API must be reachable for NATS connection)
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

        # Wait for all services using NATS subscription
        logger.info(
            "Waiting for services readiness via NATS (%fs)...", SERVICES_READINESS_TIMEOUT_SECS
        )
        await wait_for_services_ready(nova, "cell", SERVICES_READINESS_TIMEOUT_SECS)

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
