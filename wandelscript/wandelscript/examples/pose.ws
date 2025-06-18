a = (4, 5, 6, 1, 2, 3)
b = a
c = (7, 8, 9, 0, 0, 0)
d = a :: c

# read pose components
a_x = a[0]
a_rz = a[5]

# using negative indices
a_rx = a[-3]

# read position/orientation vector components)
c_y = c[1]

# "setting" pose/position/orientation components
a_new_y = assoc(a, 1, 0)
c_new_z = assoc(c, 2, c[0])

# also using negative indices
c_new_y = assoc(c, 1, 42)
