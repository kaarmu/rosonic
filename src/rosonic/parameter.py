from concurrent.futures import Future
from copy import deepcopy
from typing import Any, Callable

import rospy

from . import fields
from .fields import Field, FieldContainer
from .name import Name

class ParameterError(Exception):
    pass

class Parameter(Field):

    name: Name

    _fut: Future

    def __init__(self, name, value=None, optional=False, **kwargs):
        self.name = Name(name, **kwargs)
        self.value = value
        self.is_optional = optional or value is not None
        self._fut = Future()

    def __repr__(self) -> str:
        return 'Parameter({})'.format(str(self.name))

    def __get__(self, instance, owner):
        return self._fut.result()

    def process(self):

        if rospy.has_param(str(self.name)):
            value = rospy.get_param(str(self.name))
        elif self.is_optional:
            value = self.value
        else:
            raise ParameterError('Parameter server does not recognize "%s"' % str(self.name))

        self._fut.set_result(value)

    @staticmethod
    def process_all(container: FieldContainer):
        fields.map(
            Parameter.process,
            Parameter,
            container,
        )

    def copy(self) -> 'Parameter':
        return deepcopy(self)

    def then(self, fn: Callable[[Any], None]) -> 'Parameter':
        self._fut.add_done_callback(lambda fut: fn(fut.result()))
        return self

