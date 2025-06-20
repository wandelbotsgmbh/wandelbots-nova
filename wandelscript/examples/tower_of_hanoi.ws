io = get_controller("controller")
# Global reference positions
home = (-145.5, -578.3, 77, 0.000, 3.142, 0.000)
approach = (0, 0, -100, 0, 0, 0)
rod_left = (-247.1, -726.4, -94, 0.000, pi, 0.000)
rod_right = (-39.9, -726.4, -94, 0.000, pi, 0.000)
rod_center = interpolate(rod_left, rod_right, 0.5)
rods = [rod_left, rod_center, rod_center]
disk_height = (158.0 - 125) / 4
state = [3, 0, 0]
hanoi_state = { s0: 5, s1: 0, s2: 0 }

velocity(300)

def set_gripper(close):
    sync
    write(io, "tool_out[0]", close == False)
    write(io, "tool_out[1]", close)
    wait(500)

def action(pose, close):
    move via line() to pose :: approach
    move via line() to pose
    set_gripper(close)
    velocity(100)
    move via line() to pose :: approach
    velocity(300)

def move_disk(source, target):
    hanoi_state["s" + string(source)] = hanoi_state["s" + string(source) - 1)]
    action(rods[source] :: (0, 0, - disk_height * hanoi_state["s" + string(source)], 0, 0, 0), True)
    action(rods[target] :: (0, 0, - disk_height * hanoi_state["s" + string(target)], 0, 0, 0), False)
    write(hanoi_state, string(target), hanoi_state["s" + string(target) + 1])
    sync

def tower_of_hanoi(n , source, destination, auxiliary):
    if n == 1:
        move_disk(source, destination)
    else:
        tower_of_hanoi(n - 1, source, auxiliary, destination)
        move_disk(source, destination)
        tower_of_hanoi(n - 1, auxiliary, destination, source)

move via p2p() to home
tower_of_hanoi(5, 0, 1, 2)
state = [hanoi_state["s0"], hanoi_state["s1"], hanoi_state["s2"]]
move via line() to home
