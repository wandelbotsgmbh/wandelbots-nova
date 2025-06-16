def modifier(pose):
    tmp = __default_orientation
    __default_orientation = pose
    def on_exit():
        __default_orientation = tmp
    return on_exit
__default_orientation = 'last'
a = __default_orientation
with modifier((0, 0, 0, 0, 0, 0)):
    b = __default_orientation
c = a