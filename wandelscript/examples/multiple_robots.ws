controller = get_controller("controller")
tcp1 = frame("tool100")
tcp2 = frame("TOOL 0")

# case: no context, frame relation
move [tcp1 | controller[0]] via p2p() to (-200, -600, 250, 0, -pi, 0)
move [tcp2 | controller[1]] via p2p() to (-200, -600, 200, 0, -pi, 0)
move [tcp1 | controller[0]] via p2p() to (-200, -600, 250, 0, -pi, 0)
move [tcp2 | controller[1]] via p2p() to (-200, -500, 250, 0, -pi, 0)
sync

# case: with context, tcp
do with controller[0]:
    move tcp1 via p2p() to (-200, -600, 250, 0, -pi, 0)
    move tcp1 via p2p() to (-200, -600, 250, 0, -pi, 0)
and do with controller[1]:
    move tcp2 via p2p() to (-200, -600, 200, 0, -pi, 0)
    move tcp2 via p2p() to (-200, -500, 250, 0, -pi, 0)

# case: no context, tcp -> only works with single robot
# raises wandelscript.exception.WrongRobotError
# move tcp1 via p2p() to (-200, -600, 250, 0, -pi, 0)

a = read(controller[0], 'tcp1')
b = read(controller[1], 'tcp2')
c = read(controller[0], tcp1)
d = read(controller[1], tcp2)
