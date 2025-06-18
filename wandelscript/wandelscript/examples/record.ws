r_empty = {}

r_datatypes = { keystr: "str", keyint: 1, keyfloat: 1.1, keybool: True, keybool2: False }

r_inline = { a: 5, b: 6, c: { d: 7, e: 8 } }

r_multiline = {
    a: 1,
    b: 2,
    c: {
        d: 3,
        e: 4
    }
}

r_mixedline = {
    a: 1,
    b: 2,
    c: { d: 3, e: 4 }
}

r_trailing_comma = {
    a: 10,
    b: { c: 11, d: 12, },
}

read_keystr_attr_access = r_datatypes.keystr
read_keystr_item_access = r_datatypes["keystr"]
read_keystr_attr_access_deep = r_multiline.c.e
read_keystr_item_access_deep = r_multiline["c"]["d"]
read_keystr_item_access_deep2 = r_multiline.c["e"]

# r_mixedline.a = 4 # Error: Not possible we need to use assoc(...) for now
r_mixedline_written = assoc(r_mixedline, "b", 5)
r_mixedline_written_deep = assoc(r_mixedline, "c", assoc(r_mixedline.c, "d", 6))

def function_that_takes_a_record(rec):
    return rec

r_passed_to_fn = function_that_takes_a_record(r_inline)