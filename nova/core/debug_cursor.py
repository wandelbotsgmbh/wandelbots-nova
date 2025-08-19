import pdb

class CursorPdb(pdb.Pdb):
    """pdb with commands bound to a TrajectoryCursor instance."""
    def __init__(self, cursor, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cursor = cursor

    # Movement commands you can type directly in pdb
    def do_forward(self, arg):
        """forward -> start moving forward."""
        self.cursor.forward()
        self.do_continue(arg)

    def do_backward(self, arg):
        """backward -> start moving backward."""
        self.cursor.backward()

    def do_pause(self, arg):
        """pause -> pause movement."""
        self.cursor.pause()

    def do_detach(self, arg):
        """detach -> stop controlling movement (your detach method)."""
        self.cursor.detach()

    def do_pause_at(self, arg):
        """pause_at <float> -> add a breakpoint at <location>."""
        try:
            self.cursor.pause_at(float(arg))
        except Exception as e:
            self.error(str(e))

    def do_forward_to(self, arg):
        """forward_to <float> -> go forward until the given breakpoint location."""
        try:
            self.cursor.forward_to(float(arg))
        except Exception as e:
            self.error(str(e))

    def do_forward_to_next(self, arg):
        """forward_to_next -> go forward to the next breakpoint after current location."""
        try:
            self.cursor.forward_to_next()
        except Exception as e:
            self.error(str(e))
