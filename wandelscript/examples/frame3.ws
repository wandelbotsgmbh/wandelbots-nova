controller = get_controller("controller")

move [tcp1 | controller[0]] via p2p() to (-200, -600, 250, 0, -pi, 0)
move [tcp2 | controller[1]] via p2p() to (-200, -500, 250, 0, -pi, 0)

a = read(controller[0], tcp1)
b = read(controller[1], tcp2)
