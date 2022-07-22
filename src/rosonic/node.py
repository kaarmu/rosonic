import rospy

from .container import Container


class Node(Container):

    name = None
    rate = None

    # For convenience...
    logdebug = rospy.logdebug
    logwarn = rospy.logwarn
    loginfo = rospy.loginfo
    logerr = rospy.logerr
    logfatal = rospy.logfatal

    @classmethod
    def run(cls, load_params=True, **kwargs):
        """Create and run node"""

        # Pick class name as default node name
        cls.name = cls.name or cls.__name__

        # Create a Rate object from int
        if isinstance(cls.rate, int):
            cls.rate = rospy.Rate(cls.rate)

        rospy.init_node(cls.name, **kwargs)

        if load_params:
            cls.load_params()

        return cls().main()

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
        logfn = getattr(cls, 'log' + level, cls.loginfo)
        logfn('(%s) %s', msg % args)

    @classmethod
    def logevent(cls, msg, *args, **kwargs):
        """Log events from this node consistently"""
        cls.log('=> %s', msg % args, **kwargs)

    @staticmethod
    def sleep(time):
        """Perform rospy.sleep for time amount of seconds"""
        rospy.sleep(rospy.Duration.from_sec(time))

    @staticmethod
    def is_shutdown():
        """Return if the node is shutdown or not"""
        return rospy.is_shutdown()

    @staticmethod
    def shutdown():
        """Trigger shutdown"""
        raise rospy.ROSInterruptException()
