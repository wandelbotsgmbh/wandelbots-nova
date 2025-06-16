controller = get_controller("controller")

write(controller, 'tool_out[0]', True)
move via p2p() to (-200, -600, 300, 0, -3, 0)
write(controller, 'tool_out[0]', False)
move via line() to (-200, -600, 100)
sync
a = read(controller, 'tool_out[0]')
write(controller, 'tool_out[0]', True)
move via line() to (-200, -600, 150)
sync
b = read(controller, 'tool_out[0]')
