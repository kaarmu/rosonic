from typing import (
    Generic,
    Type,
    TypeVar,
)

import rospy

from .node import Node, NodeAttr
from .name import Name

__all__ = [
    'ParameterError',
    'Parameter',
]

class ParameterError(Exception):
    pass

T = TypeVar('T', bound=Node)
V = TypeVar('V')
class Parameter(NodeAttr, Generic[T, V]):

    name: Name
    is_optional: bool

    _optional: V

    def __init__(self, name: str, optional: V = ..., /, **kwargs):
        self.name = Name(name, **kwargs)
        self.is_optional = optional is not Ellipsis
        self._optional = optional

    def __repr__(self) -> str:
        return 'Parameter({})'.format(str(self.name))

    def on_enter(self, instance: T) -> V:
        name = str(self.name)
        if rospy.has_param(name):
            value = rospy.get_param(name)
        elif self.is_optional:
            value = self._optional
        else:
            raise ParameterError(f'Parameter server does not recognize "{name}"')
        return value

    def on_exit(self, instance: T) -> None:
        pass

