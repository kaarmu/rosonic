from functools import wraps

import rospy


class Rate(object):

    def __init__(self, hz):
        self.rate = rospy.Rate(hz)

    def __call__(self, f):
        @wraps(f)
        def wrapper(*args, **kwargs):
            ret = f(*args, **kwargs)
            self.rate.sleep()
            return ret
        return wrapper


class Timer(object):

    def __init__(self, period, oneshot=False, autostart=True):
        self.period = rospy.Duration.from_sec(period)
        self.callback = None
        self.oneshot = oneshot
        self.autostart = autostart
        self.timer = None

    def start(self):
        assert self.callback is not None, 'Missing callback'
        self.timer = rospy.Timer(self.period, self.callback, self.oneshot)

    def shutdown(self):
        assert self.timer is not None, 'Missing timer'
        self.timer.shutdown()

    def __call__(self, f):
        self.callback = f
        if self.autostart:
            self.start()
        return self

