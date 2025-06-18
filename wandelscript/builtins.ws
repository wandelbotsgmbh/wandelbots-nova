def wait_for_io(device, key, value):
    while (read(device, key) != value):
        pass

def take_image(robot_cell):
    sync
    return async_take_image(robot_cell)

def take_point_cloud(robot_cell):
    sync
    return async_take_point_cloud(robot_cell)

def set_io_map(io_map):
    io_names = io_map[0]
    io_values = io_map[1]
    for io_counter = 0..<len(io_names):
        write(controller, io_names[io_counter], io_values[io_counter])
