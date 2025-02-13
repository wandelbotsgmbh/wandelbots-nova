import asyncio
import gc

import rerun as rr
from apscheduler.schedulers.asyncio import AsyncIOScheduler
from apscheduler.triggers.interval import IntervalTrigger

from nova import Nova
from nova_rerun_bridge import NovaRerunBridge
from nova_rerun_bridge.blueprint import get_blueprint
from nova_rerun_bridge.consts import RECORDING_INTERVAL, SCHEDULE_INTERVAL, TIME_INTERVAL_NAME
from nova_rerun_bridge.motion_storage import load_processed_motions, save_processed_motion

# Global run flags
job_running = False
first_run = True
previous_motion_group_list = []


async def process_motions():
    """
    Fetch and process all unprocessed motions.
    """
    global job_running
    global first_run
    global previous_motion_group_list

    # use http://api-gateway:8080 on prod instances
    async with Nova(host="http://api-gateway:8080") as nova:
        motion_api = nova._api_client.motion_api

        try:
            motions = await motion_api.list_motions("cell")
            if motions:
                if first_run:
                    # Mark all existing motions as processed with 0 time
                    # so they won't get re-logged.
                    for mid in motions.motions:
                        save_processed_motion(mid, 0)
                    first_run = False

                processed_motions = load_processed_motions()
                processed_motion_ids = {m[0] for m in processed_motions}

                time_offset = sum(m[1] for m in processed_motions)

                # Filter out already processed motions
                new_motions = [
                    motion_id
                    for motion_id in motions.motions
                    if motion_id not in processed_motion_ids
                ]

                for motion_id in new_motions:
                    async with NovaRerunBridge(
                        nova, spawn=False, recording_id="nova_live"
                    ) as nova_bridge:
                        print(f"Processing motion {motion_id}.", flush=True)
                        rr.set_time_seconds(TIME_INTERVAL_NAME, time_offset)

                        await nova_bridge.log_collision_scenes()

                        await nova_bridge.setup_blueprint()

                        if motion_id in processed_motion_ids:
                            continue

                        trajectory = await motion_api.get_motion_trajectory(
                            "cell", motion_id, int(RECORDING_INTERVAL * 1000)
                        )

                        # Calculate time offset
                        processed_motions = load_processed_motions()
                        time_offset = sum(m[1] for m in processed_motions)
                        trajectory_time = trajectory.trajectory[-1].time
                        print(f"Time offset: {time_offset}", flush=True)

                        motion = await nova._api_client.motion_api.get_planned_motion(
                            nova.cell()._cell_id, motion_id
                        )
                        optimizer_config = await nova._api_client.motion_group_infos_api.get_optimizer_configuration(
                            nova.cell()._cell_id, motion.motion_group
                        )
                        motion_groups = await nova._api_client.motion_group_api.list_motion_groups(
                            nova.cell()._cell_id
                        )
                        motion_motion_group = next(
                            (
                                mg
                                for mg in motion_groups.instances
                                if mg.motion_group == motion.motion_group
                            ),
                            None,
                        )

                        if motion_motion_group is None:
                            raise ValueError(f"Motion group {motion.motion_group} not found")

                        nova_bridge.log_saftey_zones_(
                            motion_group_id=motion.motion_group, optimizer_setup=optimizer_config
                        )
                        await nova_bridge.log_motion(motion_id=motion_id, time_offset=time_offset)

                        # Save the processed motion ID and trajectory time
                        save_processed_motion(motion_id, trajectory_time)

        except Exception as e:
            print(f"Error during job execution: {e}", flush=True)
        finally:
            job_running = False
            await nova._api_client.close()


async def main():
    """Main entry point for the application."""
    motion_groups = []
    async with Nova(host="http://api-gateway:8080") as nova:
        cell = nova.cell()
        controllers = await cell.controllers()
        for controller in controllers:
            for motion_group in await controller.activated_motion_groups():
                motion_groups.append(motion_group.motion_group_id)

    rr.init(application_id="nova", recording_id="nova_live", spawn=False)
    rr.save("data/nova.rrd", default_blueprint=get_blueprint(motion_groups))

    # Setup scheduler
    scheduler = AsyncIOScheduler()
    scheduler.add_job(
        process_motions,
        trigger=IntervalTrigger(seconds=SCHEDULE_INTERVAL),
        id="process_motions_job",
        name=f"Process motions every {SCHEDULE_INTERVAL} seconds",
        replace_existing=True,
    )
    scheduler.start()

    try:
        while True:
            await asyncio.sleep(60)  # Keep the loop running
            gc.collect()
    except (KeyboardInterrupt, SystemExit):
        print("Shutting down gracefully.")
        scheduler.shutdown(wait=False)


if __name__ == "__main__":
    asyncio.run(main())
