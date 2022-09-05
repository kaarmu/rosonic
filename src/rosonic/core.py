from threading import Thread
from abc import ABC, abstractmethod
from typing import Any, Union

import rospy

from . import fields
from .fields import FieldContainer
from .name import Name
from .time import Rate, Timer
from .argument import Argument
from .parameter import Parameter
from .on_shutdown import OnShutdown


__all__ = [
    'GenericProgram',
    'ROSProgram',
    'Node',
    'SubNode',
]


class GenericProgram(ABC):
    """The default structure of a rosonic program."""

    def run(self, *args, **kwargs):

        self.init(*args, **kwargs)

        try:
            self.main()
        finally:
            self.deinit()

    #
    # user-space methods (intended to be overloaded)
    #

    @abstractmethod
    def init(self, *args: Any, **kwargs: Any) -> None:
        ...

    @abstractmethod
    def main(self) -> None:
        ...

    @abstractmethod
    def deinit(self) -> None:
        ...


@fields.register(OnShutdown, Timer, Rate)
class ROSProgram(GenericProgram, FieldContainer):

    is_shutdown = staticmethod(rospy.is_shutdown)
    logdebug = staticmethod(rospy.logdebug)
    logerr = staticmethod(rospy.logerr)
    logfatal = staticmethod(rospy.logfatal)
    loginfo = staticmethod(rospy.loginfo)
    logwarn = staticmethod(rospy.logwarn)
    sleep = staticmethod(rospy.sleep)

    @staticmethod
    def shutdown(reason: str, *args: Any):
        rospy.signal_shutdown(str(reason) % args)

    def __init__(self) -> None:

        ## Process fields

        fields.load(self)

        Rate.process_all(self)
        Timer.process_all(self)

    def main(self):
        while self.keep_alive():
            self.spin()

    def keep_alive(self):
        return not self.is_shutdown()

    def spin(self):
        rospy.spin()

    def deinit(self):
        OnShutdown.process_all(self)


@fields.register(Argument, Parameter)
class Node(ROSProgram):

    def __init__(
        self,
        process_params: bool = True,
        process_args: bool = True,
        args_description: Union[str, None] = None,
        anonymous: bool = False,
        **kwargs: Any,
    ) -> None:

        ## Initialize ROS node

        # default node name is name of class
        # (if not overridden by e.g. launch file)
        rospy.init_node(
            type(self).__name__,
            anonymous=anonymous,
        )

        ## Initialize program

        super().__init__(**kwargs)

        ## Process fields

        if process_args:
            Argument.process_all(
                self,
                prog=Name().base,                # get the real node name
                description=args_description,
            )

        if process_params:
            Parameter.process_all(self)


@fields.register(Parameter)
class SubNode(ROSProgram):

    def __init__(
        self,
        process_params: bool = True,
        **kwargs: Any,
    ) -> None:

        ## Initialize program

        super().__init__(**kwargs)

        ## Process fields

        if process_params:
            Parameter.process_all(self)

    def run(self, *args, **kwargs):
        self._thread = Thread(target=super().run, args=args, kwargs=kwargs)
        self._thread.start()
        return self

