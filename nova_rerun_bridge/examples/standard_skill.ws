# Read the current pose of the robot
# Note: Please replace <controller_id> with the controller name you've
#   selected and <motion_group_id> with the motion group ID
# Example:
#   - "0@abb" = get_controller("abb")[0]
#   - "1@fanuc" = get_controller("fanuc")[1]
robot = get_controller("ur")[0]

# Change this tcp to the selected tcp in the jogging panel
tcp("Flange")

home = read(robot, "pose")
sync

do with robot:
    # Set the velocity of the robot to 200 mm/s
    velocity(200)

    for i = 0..3:
        move via p2p() to home
        # Move to a pose concatenating the home pose
        move via line() to (50, 20, 30, 0, 0, 0) :: home
        move via line() to (100, 20, 30, 0, 0, 0) :: home
        move via line() to (50, 20, 30, 0, 0, 0) :: home
        move via p2p() to home