[world | red_disk] = (100, 200, 0, 0, 0, 2)
top_of_red_disk = (0, 0, 100, 0, 0, 0) :: red_disk
[world | blue_disk] = (200, 200, 0, pi / 2, 0, 0)
top_of_blue_disk = (0, 0, 100, 0, 0, 0) :: blue_disk
a = [red_disk | blue_disk]
b = [world | top_of_red_disk]
