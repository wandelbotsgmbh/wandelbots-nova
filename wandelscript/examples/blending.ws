home = (-200, -600, 250, 0, -pi, 0)

blending(0)
blending_start = __ms_position_zone_radius # =0
velocity(1000)

move via p2p() to home
move via line() to (0, 200, 0, 0, 0, 0) :: home with blending(1)
blending_after_single_line = __ms_position_zone_radius # =0
with blending(2):
    blending_in_context = __ms_position_zone_radius # =2
    move via line() to (200, 200, 0, 0, 0, 0) :: home with blending(500)
    blending_in_context_after_single_line = __ms_position_zone_radius # =2
blending_after_context = __ms_position_zone_radius # =0
move via p2p() to home

def move_line_1(pose):
    move via line() to pose :: home
    return __ms_position_zone_radius

blending_initial_in_function = move_line_1((200, 200, 0, 0, 0, 0)) # =0

with blending(3):
    blending_in_context_in_function = move_line_1((200, 200, 0, 0, 0, 0)) # =3
    move via line() to (200, 0, 0, 0, 0, 0) :: home with blending(500)
    blending_in_context_after_single_line_in_function = __ms_position_zone_radius # =3

blending_end = __ms_position_zone_radius # =0
