"""Scene colours, taken from the NOVA design system.

The values are the resolved tokens of the dark ``zero-gravity`` simulation
theme (``nova-design-system``, ``packages/design-tokens/tokens``), with the
token name kept beside each one so a change over there can be followed here.
Roles rather than colours are exported, so what a colour means in the 3D scene
is stated once. Every kind of geometry that can overlap another gets its own
token, because telling them apart is the whole point of drawing them:

- the model's own collision hulls are information about the robot: ``info``
- the tool's collision geometry is something added to the robot: ``success``
- the safety controller's volumes belong to the safety system: ``primary``
- a zone the robot may not enter is a hard restriction: ``error``
- a zone the robot may not leave is a limit, not a forbidden place: ``warning``
- a collision object in the plan is a thing in the cell, not a state: neutral
"""

INFO = (83, 109, 254)
"""``zero-gravity.info.main`` -> ``colors.indigo.A200`` (#536dfe)."""
ERROR = (239, 83, 80)
"""``zero-gravity.error.main`` -> ``colors.red.400`` (#ef5350)."""
WARNING = (255, 171, 64)
"""``zero-gravity.warning.main`` -> ``colors.orange.A200`` (#ffab40)."""
PRIMARY = (142, 86, 252)
"""``zero-gravity.primary.main`` -> ``Nova Violet5`` (#8e56fc)."""
SUCCESS = (38, 166, 154)
"""``zero-gravity.success.main`` -> ``colors.teal.400`` (#26a69a)."""
NEUTRAL = (144, 164, 174)
"""``colors.blueGrey.300`` (#90a4ae)."""

# Alphas: a body one looks *through* has to be faint, an outline solid.
BODY_ALPHA = 45
OUTLINE_ALPHA = 255

ROBOT_COLLISION = (*INFO, BODY_ALPHA)
"""The model's own collision hulls, from the URDF.

Faint on purpose. A collision hull encloses the visual mesh it belongs to, so
whatever is painted over it is painted over the robot: a strong fill turns the
model into a blue robot. The hull is read from its edges instead, and the fill
only says which side of them is inside.
"""
TOOL_COLLISION = (*SUCCESS, BODY_ALPHA)
"""The tool's collision geometry, from the URDF: what the cell defines as
mounted on the flange, which is a different thing from the model's own hulls
and from what the safety controller enforces."""
SAFETY_VOLUME = (*PRIMARY, OUTLINE_ALPHA)
"""The controller's safety volumes for links and tool. Full colour, because
they are drawn as wireframes over the robot rather than as a fill."""
SAFETY_VOLUME_BODY = (*PRIMARY, BODY_ALPHA)
"""The same volumes when they come out of the URDF as meshes: a faint body,
read from its edges, so the robot underneath stays its own colour."""

COLLISION_BODY = {"model": ROBOT_COLLISION, "tool": TOOL_COLLISION, "safety": SAFETY_VOLUME_BODY}
"""What each kind of collision geometry in a URDF is drawn in."""
OBSTACLE = (*NEUTRAL, 110)
"""A collision object a plan was checked against."""
ZONE_KEEP_OUT = (*ERROR, BODY_ALPHA)
"""A zone the robot may not enter, drawn as a see-through body."""
ZONE_KEEP_OUT_OUTLINE = (*ERROR, OUTLINE_ALPHA)
ZONE_KEEP_IN_OUTLINE = (*WARNING, OUTLINE_ALPHA)
"""A zone the robot may not leave, drawn as an outline only: filling it would
put a wall between the camera and the robot inside it."""
