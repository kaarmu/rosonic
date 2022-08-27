from functools import partial, wraps

import rospy

from . import fields

class Rate(object):

    def __init__(self, hz, reset=False):
        self.hz = hz
        self.reset = reset
        self.rate = None

    def process(self):
        self.timer = rospy.Rate(self.hz, self.reset)

    @staticmethod
    def process_all(container):
        fields.map(
            Rate.process,
            Rate,
            container,
        )

    def __call__(self, f):

        @wraps(f)
        def wrapper(*args, **kwargs):
            try:
                return f(*args, **kwargs)
            finally:
                self.timer.sleep()

        self.f = wrapper

        return self

    def __get__(self, instance, owner):
        return partial(self.f, instance)


class Timer(object):

    def __init__(self, period, oneshot=False, reset=False, autostart=True):
        self.period = rospy.Duration.from_sec(period)
        self.callback = None
        self.oneshot = oneshot
        self.reset = reset
        self.autostart = autostart
        self.timer = None

    def process(self):
        if self.autostart:
            self.start()

    @staticmethod
    def process_all(container):
        fields.map(
            Timer.process,
            Timer,
            container,
        )

    def start(self):
        assert self.callback is not None, 'Missing callback'
        self.timer = rospy.Timer(self.period, self.callback, self.oneshot, self.reset)

    def cancel(self):
        assert self.timer is not None, 'Missing timer'
        self.timer.shutdown()

    def __call__(self, f):
        self.callback = f
        return self

