import threading

import rospy

from . import fields
from .name import Name
from .time import Rate, Timer
from .argument import Argument
from .parameter import Parameter
from .on_shutdown import OnShutdown


class Program(object):
    """
    The default structure of a rosonic program.

    rosonic sees a program like a process that can either run on the main
    thread or separate. The main thread should of course use the `Node` class,
    however, other program-like processes can run on other threads. They can
    utilize the ThreadedComponent that integrates well with rosonic fields.
    """

    def __new__(cls, *args, **kwargs):
        self = super(Program, cls).__new__(cls)
        self.__init__(*args, **kwargs)
        self.main()
        return self

    def __init__(self):
        pass

    def main(self):
        while self.keep_alive():
            self.spin()

    def keep_alive(self):
        return True

    def spin(self):
        rospy.spin()


@fields.register(Argument, Parameter, OnShutdown, Timer, Rate)
class Node(Program):

    # For convenience...
    logdebug = staticmethod(rospy.logdebug)
    logwarn = staticmethod(rospy.logwarn)
    loginfo = staticmethod(rospy.loginfo)
    logerr = staticmethod(rospy.logerr)
    logfatal = staticmethod(rospy.logfatal)
    is_shutdown = staticmethod(rospy.is_shutdown)

    def __new__(cls, process_params=True, process_args=True, description=None, **kwargs):

        ## Initialize ROS node

        rospy.init_node(cls.__name__, **kwargs)

        cls.name = Name()

        ## Process registered fields

        fields.load(cls)

        OnShutdown.process_all(cls)
        Rate.process_all(cls)
        Timer.process_all(cls)

        if process_args:
            Argument.process_all(cls, prog=cls.name.base, description=description)

        if process_params:
            Parameter.process_all(cls)

        ## Run node

        try:
            return super(Node, cls).__new__(cls)
        finally:
            cls.shutdown('Node "%s" finished', cls.name.base)

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
    def sleep(time):
        """Perform rospy.sleep for time amount of seconds"""
        rospy.sleep(rospy.Duration.from_sec(time))

    @staticmethod
    def shutdown(reason, *args):
        """Trigger shutdown"""
        rospy.signal_shutdown(str(reason) % args)


class Thread(Program):
    """
    This is a program that runs on a thread.
    """

    def __new__(cls, *args, **kwargs):

        cls.thread = threading.Thread(target=cls.run, args=args, kwargs=kwargs)
        cls.event = threading.Event()

        # useful methods...
        cls.join = cls.thread.join
        cls.release = cls.event.set
        cls.wait = cls.event.wait

        return cls

    @classmethod
    def run(cls, *args, **kwargs):
        # Complete construction (includes calling __init__) and run program
        return super(Thread, cls).__new__(cls, *args, **kwargs)

    @classmethod
    def start(cls, timeout=0):
        """
        Start the threaded component.

        Args:
            timeout (float): A threaded component has the possibility to block
                the caller until the component is ready. If `timeout > 0` then
                the caller will be blocked at most `timeout` seconds. If
                `timeout < 0` then the caller will be blocked indefinetely. If
                `timeout == 0` then the caller will not be blocked at all.
        """

        cls.thread.start()

        if timeout:
            cls.wait(timeout if timeout > 0 else None)

