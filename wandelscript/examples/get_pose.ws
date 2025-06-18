move via p2p() to (-200, -600, 300, 0, pi, 0)
move via line() to (-250, -600, 300, 0, pi, 0)
a = planned_pose()
python_print(a)
sync

controller = get_controller("controller")
b = read(controller[0], 'pose')
python_print(b)
