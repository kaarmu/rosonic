from functools import partial, wraps
from typing import TypeVar, Union

import rospy

from .fields import Field


__all__ = [
    'Rate',
    'Timer',
]


T = TypeVar('T')


class Rate(Field):

    def __init__(self, hz: int, reset: bool = False, immediate: bool = False):

        self.hz = hz
        self.reset = reset
        self.rate = None

        if immediate:
            self.process_one()

    def __call__(self, fn) -> 'Rate':

        @wraps(fn)
        def wrapper(*args, **kwargs):
            try:
                return fn(*args, **kwargs)
            finally:
                self.timer.sleep()

        self.fn = wrapper

        return self

    def __get__(self, instance, owner):
        return partial(self.fn, instance)

    def process_one(self):
        self.timer = rospy.Rate(self.hz, self.reset)


class Timer(Field):

    def __init__(
        self,
        period: Union[int, float],
        oneshot: bool = False,
        reset: bool = False,
        bind: bool = True,
        immediate: bool = False,
    ):

        self.period = rospy.Duration.from_sec(period)
        self.callback = None
        self.oneshot = oneshot
        self.reset = reset
        self.timer = None
        self.fn = lambda _: None

        self.bind = bind
        self.immediate = immediate

    def __call__(self, fn):
        self.fn = fn
        return self

    def process_one(self, container):

        if self.bind:
            callback = lambda *a, **k: self.fn(container, *a, **k)
        else:
            callback = self.fn

        def start():
            self.timer = rospy.Timer(
                self.period,
                callback,
                self.oneshot,
                self.reset,
            )

        self.start = start

        if self.immediate:
            self.start()

    @classmethod
    def process_all(cls, container):
        super().process_all(container, container)

    @staticmethod
    def start() -> None:
        raise Exception('Timer has not been processed yet')

    def cancel(self):
        assert self.timer is not None, 'Missing timer'
        self.timer.shutdown()

