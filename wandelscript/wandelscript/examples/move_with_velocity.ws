with velocity(100):
    velocity_uninitialized_global_inside_context = __ms_tcp_velocity_limit
    move via p2p() to (243, -440, 241, 0, 3, 0)

# TODO: It's a bit more effort to test this because the velocity is set to Infinity
# velocity_global_default = __ms_tcp_velocity_limit

velocity(200)
velocity_global_set = __ms_tcp_velocity_limit
move via p2p() to (150, -355, 389, 0, 3, 0)
move via p2p() to (150, -355, 392, 0, 3, 0)
move via p2p() to (-95, -363, 387) with velocity(300)
velocity_global_after_motion_modifier = __ms_tcp_velocity_limit
with velocity(400):
    velocity_global_inside_context = __ms_tcp_velocity_limit
    move via p2p() to (243, -440, 241)
velocity_global_after_context = __ms_tcp_velocity_limit
a = planned_pose()
