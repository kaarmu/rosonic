from functools import partial, wraps
from typing import Callable, Generic, TypeVar, Union

import rospy

from .node import Node, NodeAttr

__all__ = [
    'rate',
    'repeater',
]


T = TypeVar('T', bound=Node)
A = TypeVar('A')
class rate(NodeAttr, Generic[T, A]):

    _fn: Callable[..., A]
    _hz: int
    _reset: bool
    _bind: bool

    def __init__(
        self,
        hz: int,
        reset: bool = False,
        is_method: bool = True
    ):

        self._hz = hz
        self._reset = reset
        self._bind = is_method # bind func to instance if it is a method

    def __call__(self, fn: Callable[..., A]) -> 'rate':
        self._fn = fn
        return self

    def on_enter(self, instance: T) -> Callable[..., A]:

        rate = rospy.Rate(self._hz, self._reset)
        fn = partial(self._fn, instance) if self._bind else self._fn

        @wraps(self._fn)
        def wrapper(*args, **kwargs):
            try: return fn(*args, **kwargs)
            finally: rate.sleep()

        return wrapper

    def on_exit(self, instance: T) -> None:
        pass


T = TypeVar('T', bound=Node)
class repeater(NodeAttr, Generic[T]):

    _fn: Callable[..., None]
    _period: rospy.Duration
    _reset: bool
    _bind: bool

    def __init__(
        self,
        period: Union[int, float],
        reset: bool = False,
        is_method: bool = False,
    ):

        self._period = rospy.Duration.from_sec(period)
        self._reset = reset
        self._bind = is_method # bind func to instance if it is a method

    def __call__(self, fn: Callable[..., None]) -> 'repeater':
        self._fn = fn
        return self

    def on_enter(self, instance: T) -> rospy.Timer:

        fn = partial(self._fn, instance) if self._bind else self._fn

        return rospy.Timer(
                period=self._period,
                callback=fn,
                oneshot=False,
                reset=self._reset,
            )

    def on_exit(self, instance: T) -> None:
        pass

