from __future__ import annotations

import numpy as np
from geometricalgebra import cga3d
from scipy.sparse.csgraph import dijkstra

Frame = str


class FrameSystem:
    """This class collects frames and its relationships and computes all other relationships on demand

    Example:
    >>> fs = FrameSystem()
    >>> a = Frame("a")
    >>> b = Frame("b")
    >>> a_in_b = cga3d.Vector.from_pos_and_rot_vector([4, 5, 6, .1, .2, .3])
    >>> fs[a, b] = a_in_b
    >>> fs[a, b].to_pos_and_rot_vector()
    array([4. , 5. , 6. , 0.1, 0.2, 0.3])
    >>> b_in_c = cga3d.Vector.from_pos_and_rot_vector([1, 0, 0, 0, 0, 0])
    >>> c = Frame("c")
    >>> fs[b, c] = b_in_c
    >>> (a_in_b & b_in_c).to_pos_and_rot_vector()
    array([4.9357548 , 5.30293271, 5.81945992, 0.1       , 0.2       ,
           0.3       ])
    >>> fs.eval(a, c).to_pos_and_rot_vector()
    array([4.9357548 , 5.30293271, 5.81945992, 0.1       , 0.2       ,
           0.3       ])
    """

    def copy(self) -> FrameSystem:
        """Copies deeply

        Example:
        >>> a = Frame("a")
        >>> b = Frame("b")
        >>> fs = FrameSystem()
        >>> copy = fs.copy()
        >>> fs[a, b] = cga3d.Vector.from_identity()
        >>> (a, b) in copy._relations
        False
        """
        return FrameSystem(self._relations.copy())

    def __init__(self, relations: dict[tuple[Frame, Frame], cga3d.Vector] | None = None):
        self._relations = relations if relations is not None else {}

    def frames(self) -> set[Frame]:
        return {a for (a, _) in self._relations}.union({b for (_, b) in self._relations})

    def _compute(self, start, end) -> cga3d.Vector:
        frames_list = list(self.frames())
        dist = np.full([len(frames_list), len(frames_list)], np.inf)
        for ra, rb in self._relations:
            a_indices = frames_list.index(ra)
            b_indices = frames_list.index(rb)
            dist[a_indices, b_indices] = 1
            dist[b_indices, a_indices] = 2
        _, predecessors = dijkstra(dist, return_predecessors=True)
        a = frames_list.index(start)
        b = frames_list.index(end)
        chain = [b]
        while chain[-1] != a:
            chain.append(predecessors[a, chain[-1]])
        chain_frames = [frames_list[i] for i in chain]
        v = cga3d.Vector.from_identity()
        for rb, ra in zip(chain_frames, chain_frames[1:]):
            t = (
                self._relations[ra, rb]
                if (ra, rb) in self._relations
                else self._relations[rb, ra].inverse()
            )
            v = t & v
        return v

    def __getitem__(self, key):
        return self._relations[key]

    def eval(self, target, source):
        try:
            return self._relations[target, source]
        except KeyError:
            return self._compute(target, source)

    def __setitem__(self, key, item):
        self._relations[key] = item
