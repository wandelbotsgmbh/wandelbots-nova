"""Python 3.13 compatibility shim for ``geometricalgebra``.

``geometricalgebra`` (last released as 0.1.3, unmaintained) defines its
``algebra`` class attribute by chaining ``@classmethod`` and ``@property``:

    @classmethod
    @property
    def algebra(cls):
        ...

That pattern was deprecated in Python 3.11 and disabled in 3.13: ``classmethod``
stops delegating ``__get__`` to the wrapped ``property``, so ``cls.algebra``
returns the raw ``property`` object instead of invoking it, e.g.
``AttributeError: 'property' object has no attribute 'dims_of_grade'``.

The affected modules (``vector``, ``cga2d``, ``cga3d``, ``cga4d``) trigger this
bug in their own top-level code (e.g. ``cga3d.py`` calls ``Vector.basis()`` right
after the class body), so the fix has to be applied *while the module is being
imported*, before any post-import monkeypatch could run. This installs an
import hook that rewrites the broken decorator pair to an equivalent, working
classproperty descriptor in the module source before it executes.

There is no newer upstream release of ``geometricalgebra`` to depend on
instead (0.1.3 is the last one ever published).
"""

from __future__ import annotations

import importlib.abc
import importlib.machinery
import sys
from typing import Callable


class _ClassProperty:
    """Drop-in replacement for the ``@classmethod`` + ``@property`` pattern."""

    def __init__(self, fget: Callable):
        self._fget = fget

    def __get__(self, _obj, owner):
        return self._fget(owner)


_PATCHED_SUBMODULES = frozenset({"vector", "cga2d", "cga3d", "cga4d"})
_OLD_SNIPPET = "@classmethod  # type: ignore\n    @property\n"
_NEW_SNIPPET = "@_ClassProperty\n"


class _PatchedSourceLoader(importlib.machinery.SourceFileLoader):
    def get_source(self, fullname):
        source = super().get_source(fullname)
        assert source is not None
        patched = source.replace(_OLD_SNIPPET, _NEW_SNIPPET)
        if patched.count(_NEW_SNIPPET) != 1:
            raise ImportError(
                f"geometricalgebra 3.13 compat shim: expected exactly one classproperty "
                f"pattern in {fullname}, found source changed upstream. Refusing to patch."
            )
        return patched

    def get_code(self, fullname):
        # Bypass the .pyc cache: it would be keyed off the unpatched source and skip
        # get_source() entirely, silently reintroducing the bug it's meant to fix.
        source = self.get_source(fullname)
        return compile(source, self.get_filename(fullname), "exec", dont_inherit=True)

    def exec_module(self, module):
        module.__dict__["_ClassProperty"] = _ClassProperty
        super().exec_module(module)


class _GeometricAlgebraFinder(importlib.abc.MetaPathFinder):
    """Rewrites geometricalgebra's broken classmethod+property pattern on import."""

    def find_spec(self, fullname, path, target=None):
        if not fullname.startswith("geometricalgebra."):
            return None
        if fullname.rsplit(".", 1)[-1] not in _PATCHED_SUBMODULES:
            return None
        spec = importlib.machinery.PathFinder.find_spec(fullname, path)
        if spec is None or not isinstance(spec.loader, importlib.machinery.SourceFileLoader):
            return None
        spec.loader = _PatchedSourceLoader(spec.loader.name, spec.loader.path)
        return spec


def apply() -> None:
    """Install the geometricalgebra compat import hook (Python 3.13+ only)."""
    if sys.version_info < (3, 13):
        return  # the original pattern still works on 3.11/3.12
    if any(isinstance(finder, _GeometricAlgebraFinder) for finder in sys.meta_path):
        return  # already installed
    sys.meta_path.insert(0, _GeometricAlgebraFinder())
