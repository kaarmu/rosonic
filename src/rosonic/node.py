import rospy

from .name import Name
from .argument import Argument
from .parameter import Parameter
from .shutdown import OnShutdown
from .container import Container

# Hack: A container can contain containers, for possibility of recursive iteration
@Container(Argument)
@Container(Parameter)
@Container(OnShutdown)
class Node(object):

    name = ''

    # For convenience...
    logdebug = staticmethod(rospy.logdebug)
    logwarn = staticmethod(rospy.logwarn)
    loginfo = staticmethod(rospy.loginfo)
    logerr = staticmethod(rospy.logerr)
    logfatal = staticmethod(rospy.logfatal)
    is_shutdown = staticmethod(rospy.is_shutdown)

    def __new__(cls, load_params=True, load_args=True, description=None, **kwargs):

        ## Initialize ROS node

        # Pick class name as default node name, then check if basename
        self.name = cls.name or cls.__name__
        assert Name(self.name).is_basename, 'Node must be named with a ROS basename'

        rospy.init_node(self.name, **kwargs)

        ## Load parameters/arguments

        if load_args:
            Argument.parse_from(cls, prog=cls.name, description=description)

        if load_params:
            Parameter.load_from(cls)

        ## Create and run node

        self = super(Node, cls).__new__(cls)

        self.__init__()

        try:
            return self.main()
        finally:
            Container.map(
                lambda x: x.func(self),
                OnShutdown,
                cls
            )

    def main(self):
        while self.keep_alive():
            self.spin()
        self.shutdown()

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
        logfn('(%s) %s', cls.name, msg % args)

    @classmethod
    def logevent(cls, msg, *args, **kwargs):
        """Log events from this node consistently"""
        cls.log('=> %s', msg % args, **kwargs)

    @staticmethod
    def sleep(time):
        """Perform rospy.sleep for time amount of seconds"""
        rospy.sleep(rospy.Duration.from_sec(time))

    @staticmethod
    def shutdown():
        """Trigger shutdown"""
        raise rospy.ROSInterruptException()
