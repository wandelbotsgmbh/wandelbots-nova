a = []
interrupt inter1(param, pose) when on_robot_moves():
    a = a + [pose]
home = (832, -452, 289, 0, pi, 0)
activate inter1
move via p2p() to home
move via line() to (0, 0, 100, 0, 0, 0) :: home
move via line() to (0, 50, 0, 0, 0, 0) :: home
sync
l = len(a)
deactivate inter1