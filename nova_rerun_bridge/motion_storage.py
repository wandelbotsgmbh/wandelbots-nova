import json
import os

PROCESSED_MOTIONS_FILE = "processed_motions.json"


def load_processed_motions():
    if os.path.exists(PROCESSED_MOTIONS_FILE):
        with open(PROCESSED_MOTIONS_FILE) as file:
            return set(tuple(item) for item in json.load(file))
    return set()


def save_processed_motion(motion_id, trajectory_time):
    processed_motions = load_processed_motions()
    processed_motions.add((motion_id, trajectory_time))
    with open(PROCESSED_MOTIONS_FILE, "w") as file:
        json.dump(list(processed_motions), file)
