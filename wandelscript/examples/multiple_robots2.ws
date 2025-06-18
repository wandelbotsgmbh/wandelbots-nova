controller = get_controller("controller")
tcp1 = frame("tool100")
tcp2 = frame("TOOL 0")

do with controller[0]:
    move tcp1 via p2p() to (-200, -600, 250, 0, -pi, 0)
and do with controller[1]:
    move tcp2 via p2p() to (-200, -600, 200, 0, -pi, 0)

a = read(controller[0], tcp1)
b = read(controller[1], tcp2)
