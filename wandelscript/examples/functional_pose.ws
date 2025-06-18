a = (1, 2, 3, 0.4, 0.5, 0.6)
b = (4, 2, 5, 0.1, 0.3, 0.5)
c = a :: b

def func_a():
    return a

def func_b():
    return b

func_c1 = func_a :: b
evaled_c1 = func_c1()
func_c2 = a :: func_b
evaled_c2 = func_c2()
func_c3 = func_a :: func_b
evaled_c3 = func_c3()
