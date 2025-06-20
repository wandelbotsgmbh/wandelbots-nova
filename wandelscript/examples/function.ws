def square(a):
    return a * a

a = square(12)

def power2(c, e):
    if e:
        result = c * power2(c, e - 1)
    else:
        result = 1
    return result

b = power2(3, 4)
