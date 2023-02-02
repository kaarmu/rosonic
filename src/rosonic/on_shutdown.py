from functools import partial
from typing import (
    Callable,
    Generic,
    TypeVar,
)

from .node import Node, NodeAttr

__all__ = [
    'on_shutdown',
]

T = TypeVar('T', bound=Node)
class on_shutdown(NodeAttr, Generic[T]):

    _bind: bool
    _fn: Callable[..., None]

    def __init__(self, is_method: bool = True):
        self._bind = is_method # bind func to instance if it is a method

    def __call__(self, fn: Callable[..., None]) -> 'on_shutdown':
        self._fn = fn
        return self

    def on_enter(self, instance: T) -> None:
        pass

    def on_exit(self, instance: T) -> None:
        fn = partial(self._fn, instance) if self._bind else self._fn
        return fn()
