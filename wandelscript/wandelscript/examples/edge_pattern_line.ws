
start_left = (775.2, -473.7, 262.2, 2.916, -0.062, 1.195)  # pose whose xy-plane is parallel to the left surface and the origin is close to start position
end_left = (795.4, 60.7, 272.7, 2.92, -0.062, 1.185)  # pose whose xy-plane is parallel to the left surface and the origin is close to end position
start_right = (799.8, -467.0, 265.4, 2.866, -0.213, -1.158)  # pose whose xy-plane is parallel to the right surface and the origin is close to start position
end_right = (817.1, 54.6, 255.7, 2.866, -0.213, -1.158)  # pose whose xy-plane is parallel to the right surface and the origin is close to end position
radius = 8  # the desired radius of the final edge
spacing = 40  #  the distance between to zig-zag corners
edge_poses = find_edge_from_4_poses(start_left, end_left, start_right, end_right)
start  = to_position(edge_poses[0])
end = to_position(edge_poses[1])
n = int(distance(start, end) / spacing)
plane_orientations = [to_orientation(edge_poses[0]), to_orientation(edge_poses[1])]
center_rotation = interpolate(plane_orientations[0], plane_orientations[1], 0.5)
offset = (0, 0, -radius, 0, 0, 0)
offset_from_axis = center_rotation :: (0, 0, distance_from_corner(edge_poses[0], edge_poses[1], radius), 0, 0, 0) :: ~center_rotation
b = 0

start_poses = []
end_poses = []
steps = []

for i = 0..10:
    steps = steps + [i * 0.1]

for i = 0..<len(steps):
    p = steps[i]
    interpolated_rotation = interpolate(plane_orientations[0], plane_orientations[1], p)
    start_poses = start_poses + [(to_pose(start) :: offset_from_axis :: interpolated_rotation :: offset)]
    end_poses = end_poses + [(to_pose(end) :: offset_from_axis :: interpolated_rotation :: offset)]

move via p2p() to (0, 0, 100, 0, 0, 0) :: to_pose(start) :: offset_from_axis :: plane_orientations[0] :: offset :: (0, 0, -100, 0, 0, 0)
sync
ci = 0
for i = 0..<(int(len(steps) / 2) + 1):
    move via line() to start_poses[ci]
    move via line() to end_poses[ci]

    if (ci + 1) < len(steps):
        move via line() to end_poses[ci + 1]
        move via line() to start_poses[ci + 1]

        if (ci + 2) < len(steps):
            move via line() to start_poses[ci + 2]

    ci = ci + 2

    if i == 1:
        python_print(planned_pose())
        test_pose = planned_pose()

sync