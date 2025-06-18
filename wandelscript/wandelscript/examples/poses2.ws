# Lets first define a position 'a'. The position is given by three coordinates x, y, z in a frame of reference.
# Lets provide the coordinates in the world frame of reference.

position_of_a_in_world = (1, 2, 3)

# The robot (base) is mounted at a specific pose in the world frame of reference. Lets call this pose.

pose_of_robot_in_world = (30, 20, 10, 0, 0, 0)

# Note this is not a position but a pose. It is a position and an orientation. The orientation is given by the
# rotation vector (0, 0, 0). This means that the robot is oriented in the same direction as the world frame of reference.

# We can compute the position of 'a' in the robot frame of reference by

position_of_a_in_robot = ~pose_of_robot_in_world :: position_of_a_in_world

# The '~' operator is used to invert the pose. The '::' operator is used to transform the position from one frame of reference to another.

# Lets print the result
print(position_of_a_in_world)

# But in many cases we are not just interested to move to a position, but rather to a precise pose

pose_of_a_in_world = (1, 2, 3, 0, 0, 0)

# We can follow the same procedure as before to compute the pose of 'a' in the robot frame of reference
pose_of_a_in_robot = ~pose_of_robot_in_world :: pose_of_a_in_world
