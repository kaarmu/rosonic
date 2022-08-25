import rospy

from .name import Name
from .argument import Argument
from .parameter import Parameter
from .on_shutdown import OnShutdown
from .container import Container


class something:
    pass

@Container.register(OnShutdown, Argument, Parameter)
class Node(Container):

    node_name = ''
    rate = 1e9

    # For convenience...
    logdebug = staticmethod(rospy.logdebug)
    logwarn = staticmethod(rospy.logwarn)
    loginfo = staticmethod(rospy.loginfo)
    logerr = staticmethod(rospy.logerr)
    logfatal = staticmethod(rospy.logfatal)
    is_shutdown = staticmethod(rospy.is_shutdown)

    def __new__(cls, process_params=True, process_args=True, description=None, **kwargs):

        ## Initialize ROS node

        # Pick class name as default node name, then check if basename
        cls.node_name = cls.node_name or cls.__name__
        assert Name(cls.node_name).is_basename, 'Node must be named with a ROS basename'

        rospy.init_node(cls.node_name, **kwargs)

        ## Create and run node

        self = super(Node, cls).__new__(cls)

        # Load all `Argument`
        if process_args:
            Argument.process_arguments(self, prog=cls.node_name, description=description)

        # Load all `Parameter`
        if process_params:
            Parameter.process_parameters(self)

        # Register all `OnShutdown`
        OnShutdown.process_onshutdown(self)

        # Initialize rate
        if isinstance(self.rate, (int, float)):
            self.rate = rospy.Rate(self.rate)

        # Initialize node (user defined)
        self.__init__()

        # Run main loop
        self.main()

    def main(self):
        while self.keep_alive():
            self.spin()
            self.rate.sleep()
        self.shutdown('Node "%s" finished', self.node_name)

    def keep_alive(self):
        """Predicate for running main loop"""
        return not self.is_shutdown()

    def spin(self):
        rospy.spin()

    #
    # Wrapped rospy concepts
    #

    @classmethod
    def log(cls, msg, *args, **kwargs):
        """Log from this node consistently"""
        level = getattr(kwargs, 'level', 'info')
        logfn = getattr(rospy, 'log' + level)
        assert logfn is not None, 'Invalid logging level %s' % level
        logfn('(%s) %s', cls.node_name, msg % args)

    @classmethod
    def logevent(cls, msg, *args, **kwargs):
        """Log events from this node consistently"""
        cls.log('=> %s', msg % args, **kwargs)

    @staticmethod
    def sleep(time):
        """Perform rospy.sleep for time amount of seconds"""
        rospy.sleep(rospy.Duration.from_sec(time))

    @staticmethod
    def shutdown(reason, *args):
        """Trigger shutdown"""
        rospy.signal_shutdown(reason % args)
