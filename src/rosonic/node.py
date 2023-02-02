from abc import ABC, abstractmethod
from contextlib import ExitStack
from functools import partial
from typing import (
    Any,
    Callable,
    Dict,
    Generic,
    Tuple,
    Type,
    TypeVar,
    Union,
)

import rospy

from .task import Task

__all__ = [
    'NodeException',
    'NodeShutdown',
    'Node',
    'NodeAttr',
    'run',
]


class rospy_mixin:

    is_shutdown = staticmethod(rospy.is_shutdown)
    logdebug = staticmethod(rospy.logdebug)
    logerr = staticmethod(rospy.logerr)
    logfatal = staticmethod(rospy.logfatal)
    loginfo = staticmethod(rospy.loginfo)
    logwarn = staticmethod(rospy.logwarn)
    sleep = staticmethod(rospy.sleep)


class NodeException(Exception):

    _fmt: str
    _args: Tuple

    def __init__(self, fmt: str, *args: str):
        self._fmt = fmt
        self._args = args

    def __call__(self, *args: str) -> 'NodeException':
        self._args += args
        return self

    def __str__(self) -> str:
        return self._fmt % self.args

class NodeShutdown(NodeException): pass

class Node(Task, rospy_mixin):

    anonymous: bool = False

    __node_attrs__: Dict[str, Any]

    def __new__(cls):

        rospy.init_node(cls.__name__,
                        anonymous=cls.anonymous,
                        disable_signals=True)

        self = super().__new__(cls)

        self.__node_attrs__ = {}

        return self

    def __enter__(self) -> Callable[[], None]:
        return self.main

    def __exit__(self, *exc) -> bool:
        # call on_shutdown callbacks
        return exc[0] is NodeShutdown

    def main(self):
        while True:
            self.spin()

    def spin(self):
        rospy.spin()

    @staticmethod
    def shutdown(reason: str, *args: Any):
        rospy.signal_shutdown(reason % args)
        raise NodeShutdown(reason % args)


EXC_NODE_ATTR_MISSING = NodeException('Node attribute %s has not been loaded')

T = TypeVar('T', bound=Node)
A = TypeVar('A')
class NodeAttr(Generic[T, A], ABC):

    _name: str

    @abstractmethod
    def __init__(self) -> None:
        ...

    @abstractmethod
    def on_enter(self, instance: T) -> A:
        ...

    @abstractmethod
    def on_exit(self, inst: T) -> None:
        ...

    def get(self, instance: T) -> A:
        return instance.__node_attrs__[self._name]

    def set(self, instance: T, value: A) -> None:
        instance.__node_attrs__[self._name] = value

    def __set_name__(self, owner: Type[T], name: str):
        self._name = name

    def __get__(self, instance: T, owner: Union[Type[T], None] = None) -> A:
        if owner is None:
            raise EXC_NODE_ATTR_MISSING(self._name)
        else:
            return self.get(instance)

    def __set__(self, instance: T, value: A) -> None:
        self.set(instance, value)

    def __enter__(self) -> Callable[[T], Tuple[Callable, Tuple]]:

        # setup function calls on_enter, sets the attr value and then
        # returns an on_exit function that should be added as a stack callback
        def setup(inst: T) -> Tuple[Callable, Tuple]:
            self.__set__(inst, self.on_enter(inst))
            return self.on_exit, (inst,)

        return setup

    def __exit__(self, *exc) -> None:
        return


T = TypeVar('T', bound=Node)
def run(cls: Type[T], *args, **kwargs) -> None:
    with ExitStack() as stack:

        node = cls(*args, **kwargs)
        task = stack.enter_context(node)

        for attr in vars(cls).values():
            if isinstance(attr, NodeAttr):
                cb, a = stack.enter_context(attr)(node)
                stack.callback(cb, *a)

        task()

