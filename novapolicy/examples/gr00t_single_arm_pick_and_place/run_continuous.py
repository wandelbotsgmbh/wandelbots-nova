"""Run the GR00T cube-place policy on ``ur10e-calibration``, streaming continuously.

Re-infers as fast as the server answers; chunks are merged on a timestep queue.
Dry run by default; ``--execute`` moves the robot and the gripper. See README.md.
"""

from gr00t_ur10e import cli

if __name__ == "__main__":
    cli(continuous=True)
