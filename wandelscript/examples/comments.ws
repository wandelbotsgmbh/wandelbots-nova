a = 0 # This is an end of line comment
b = 1 # This as well # and it has the comment marker inside
# This is a full line comment which should not reassign b  b = 2

# This is a comment with an empty line before
c = 1
# This is a comment with an empty line after

d = 1

# This is a comment with empty lines before and after

    # We also allow indented comments

def foo():
    # And comments inside of functions
    a = 2
    def bar():
        # In nested functions even
        a = 3
        pass  # End of line comments still work here too
