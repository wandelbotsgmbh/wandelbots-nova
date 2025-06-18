# This is the global scope
g = "global"

def get_global():
    return g

def set_global(value):
    g = value

def local_modified_by_local_setter():
    outer_var = "outer value"

    def get_outer():
        return outer_var

    def set_outer(value):
        outer_var = value

    set_outer("outer modified")
    return get_outer

def local_modified_by_2nd_level_local_setter():
    outer_var = "outer value"

    def get_outer():
        return outer_var

    def get_setter():
        def set_outer(value):
            outer_var = value
        return set_outer

    set_outer = get_setter()
    set_outer("outer modified")
    return get_outer

def definition_order_inside_closure():
    def get_outer():
        return outer_var

    def set_outer(value):
        outer_var = value

    set_outer("outer modified")
    outer_var = "outer value"
    return get_outer


global_before_modification = get_global()
set_global("global modified")
global_after_modification = get_global()

getter = local_modified_by_local_setter()
g_local_modified_by_local_setter = getter()

getter = local_modified_by_2nd_level_local_setter()
g_local_modified_by_2nd_level_local_setter = getter()

getter = definition_order_inside_closure()
g_definition_order_inside_closure = getter()