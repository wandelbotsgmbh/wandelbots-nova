start_left = (775.2, -473.7, 262.2, 2.916, -0.062, 1.195)  # pose whose xy-plane is parallel to the left surface and the origin is close to start position
end_left = (795.4, 60.7, 272.7, 2.92, -0.062, 1.185)  # pose whose xy-plane is parallel to the left surface and the origin is close to end position
start_right = (799.8, -467.0, 265.4, 2.866, -0.213, -1.158)  # pose whose xy-plane is parallel to the right surface and the origin is close to start position
end_right = (817.1, 54.6, 255.7, 2.866, -0.213, -1.158)  # pose whose xy-plane is parallel to the right surface and the origin is close to end position
radius = 8  # the desired radius of the final edge
spacing = 40  #  the distance between to zig-zag corners

edge_poses = find_edge_from_4_poses(start_left, end_left, start_right, end_right)
start  = to_position(edge_poses[0])
end  = to_position(edge_poses[1])
n = int(distance(start, end) / spacing)
plane_orientations = [to_orientation(edge_poses[0]), to_orientation(edge_poses[1])]
center_rotation = interpolate(plane_orientations[0], plane_orientations[1], 0.5)

offset = (0, 0, -radius, 0, 0, 0)
offset_from_axis = center_rotation :: (0, 0, distance_from_corner(edge_poses[0], edge_poses[1], radius), 0, 0, 0) :: ~center_rotation
b = 0

move via p2p() to (0, 0, 100, 0, 0, 0) :: to_pose(start) :: offset_from_axis :: plane_orientations[0] :: offset :: (0, 0, -100, 0, 0, 0)
move via line() to to_pose(start) :: offset_from_axis :: plane_orientations[0] :: offset
sync

controller = get_controller("controller")
write(controller, "10020#0009", True)
sync
for i = 0..<int(n / 2):
    c = b
    a = interpolate(to_pose(start), to_pose(end),(2 * i + 1) / n ) :: offset_from_axis :: center_rotation :: offset
    b = interpolate(to_pose(start), to_pose(end),(2 * i + 2) / n ) :: offset_from_axis :: plane_orientations[modulo(i + 1, 2)] :: offset
    python_print(to_position(b))
    move via arc(a) to b
    # FIX: without the following line the last motion is a circular and the first motion cannot be a circular
    move via p2p() to b
    if i == 1:
        test_pose = planned_pose()
sync
write(controller, "10020#0009", False)
sync
