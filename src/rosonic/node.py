import threading

import rospy

from . import fields
from .name import Name
from .time import Rate, Timer
from .argument import Argument
from .parameter import Parameter
from .on_shutdown import OnShutdown


@fields.register(Timer, Rate)
class Program(object):
    """
    The default structure of a rosonic program.

    rosonic sees a program like a process that can either run on the main
    thread or separate. The main thread should of course use the `Node` class,
    however, `SubProgram` will be run on a separate thread.
    """

    # For convenience...
    logdebug = staticmethod(rospy.logdebug)
    logwarn = staticmethod(rospy.logwarn)
    loginfo = staticmethod(rospy.loginfo)
    logerr = staticmethod(rospy.logerr)
    logfatal = staticmethod(rospy.logfatal)
    is_shutdown = staticmethod(rospy.is_shutdown)

    def __new__(cls, *args, **kwargs):

        ## Process registered fields

        fields.load(cls)

        Rate.process_all(cls)
        Timer.process_all(cls)

        ## Create Program object

        return super(Program, cls).__new__(cls, *args, **kwargs)

    def main(self):
        while self.keep_alive():
            self.spin()

    def keep_alive(self):
        return not self.is_shutdown()

    def spin(self):
        rospy.spin()

    @staticmethod
    def sleep(time):
        """Perform rospy.sleep for time amount of seconds"""
        rospy.sleep(rospy.Duration.from_sec(time))


@fields.register(OnShutdown, Argument, Parameter)
class Node(Program):

    def __new__(cls, process_params=True, process_args=True, description=None, **kwargs):

        ## Initialize ROS node

        rospy.init_node(cls.__name__, **kwargs)

        cls.name = Name()

        ## Process registered fields

        fields.load(cls)

        if process_args:
            Argument.process_all(cls, prog=cls.name.base, description=description)

        if process_params:
            Parameter.process_all(cls)

        ## Run node

        self = super(Node, cls).__new__(cls)

        try:
            self.__init__()
            self.main()
        finally:
            OnShutdown.process_all(self)

        return self


    #
    # Wrapped rospy concepts
    #

    def log(self, msg, *args, **kwargs):
        """Log from this node consistently"""
        level = getattr(kwargs, 'level', 'info')
        logfn = getattr(rospy, 'log' + level)
        assert logfn is not None, 'Invalid logging level %s' % level
        logfn('(%s) %s', self.name.base, str(msg) % args)

    def logevent(self, msg, *args, **kwargs):
        """Log events from this node consistently"""
        self.log('=> %s', str(msg) % args, **kwargs)

    @staticmethod
    def shutdown(reason, *args):
        """Trigger shutdown"""
        rospy.signal_shutdown(str(reason) % args)


class SubProgram(Program):
    """
    This is a program that runs on a thread.
    """

    def __new__(cls, *args, **kwargs):
        # cls._thread = threading.Thread(target=cls.main)
        cls._stop_event = threading.Event()
        self = super(SubProgram, cls).__new__(cls)
        self.__init__(*args, **kwargs)
        self._thread = threading.Thread(target=self.main)
        return self

    def start(self):
        """Start the threaded component. """
        self._thread.start()
        return self

    def keep_alive(self):
        return not (self.is_shutdown() or self._stop_event.is_set())

    def stop(self):
        return self._stop_event.set()

    def join(self, *args, **kwargs):
        return self._thread.join(*args, **kwargs)

    def stop_and_join(self):
        self.stop()
        self.join()
