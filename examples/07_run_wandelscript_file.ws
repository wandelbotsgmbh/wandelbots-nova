# Read the current pose of the robot.
#
# Note: Please replace <controller_id> with the controller name you've
#   selected and <motion_group_id> with the motion group ID
#
# Example:
#   - "0@abb" = get_controller("abb")[0]
#   - "1@fanuc" = get_controller("fanuc")[1]
#
tcp("Flange")
robot = get_controller("ur")[0]
home = read(robot, "pose")
sync

p = read(arguments, "pose_a")
print(p)

# Set the velocity of the robot to 200 mm/s
velocity(200)

for i = 0..3:
    move via ptp() to home
    # Move to a pose concatenating the home pose
    move via line() to (50, 20, 30, 0, 0, 0) :: home
    move via line() to (100, 20, 30, 0, 0, 0) :: home
    move via line() to (50, 20, 30, 0, 0, 0) :: home
    move via ptp() to home