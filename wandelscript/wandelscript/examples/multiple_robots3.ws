def gripper(controller, open):
  if open:
    write(controller, "digital_out[0]", True)
    write(controller, "digital_out[1]", False)
    write(controller, "digital_out[2]", True)
  else:
    write(controller, "digital_out[0]", True)
    write(controller, "digital_out[1]", True)
    write(controller, "digital_out[2]", False)

controller = get_controller("controller")

do with controller[0]:
    move via p2p() to (-189, -600, 260, 0, -pi, 0)
    gripper(controller, True)
and do with controller[1]:
    move via p2p() to (500, 0, 500, -2, 0, -2)
    move via p2p() to (500, 0, 500, -2, 0, -2) :: (0, 0, 100)
    move via p2p() to (500, 0, 500, -2, 0, -2)
    gripper(controller, False)
