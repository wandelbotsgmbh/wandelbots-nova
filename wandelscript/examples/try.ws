controller = get_controller("controller")

do:
    move via p2p() to (-200, -600, 260, 0, pi, 0)
    move via p2p() to (-200, -600, 300, 0, pi, 0)
    raise "some artificial error"
    move via p2p() to (-200, -600, 250, 0, pi, 0)
sync:
    print("Hello World")
except:
    sync
    stopped_pose = read(controller[0], 'pose')

    move via p2p() to (-200, -600, 350, 0, pi, 0)
    sync
    final_pose = read(controller[0], 'pose')
