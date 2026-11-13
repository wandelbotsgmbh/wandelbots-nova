"""Run the GR00T cube-place policy on ``ur10e-calibration``, one chunk at a time.

Runs each chunk to standstill, then observes and infers again.
Dry run by default; ``--execute`` moves the robot and the gripper. See README.md.
"""

from gr00t_ur10e import cli

if __name__ == "__main__":
    cli(continuous=False)
