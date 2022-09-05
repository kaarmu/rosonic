from functools import partial
from typing import Callable, Union

import rospy

from . import fields
from .fields import Field, FieldContainer

_Meth = Callable[[FieldContainer], None]
_Func = Callable[[], None]

class OnShutdown(Field):

    bind: bool
    fn: Union[_Meth, _Func]

    def __init__(self, bind: bool = True):
        self.bind = bind
        self.fn = lambda _: None

    def __call__(self, fn: Union[_Meth, _Func]):
        self.fn = fn
        return self

    @staticmethod
    def process_all(container: FieldContainer):

        # Bind shutdown callbacks to container
        fs = fields.map(
            lambda self: (
                partial(self.fn, container) if self.bind else
                self.fn
            ),
            OnShutdown,
            container
        )

        # register callbacks with rospy
        for f in fs:
            rospy.on_shutdown(f)

