from functools import partial, wraps

import rospy

from . import fields

class Rate(object):

    def __init__(self, hz, reset=False, immediate=False):
        self.hz = hz
        self.reset = reset
        self.rate = None

        if immediate:
            self.process()

    def __call__(self, f):

        @wraps(f)
        def wrapper(*args, **kwargs):
            try:
                return f(*args, **kwargs)
            finally:
                self.timer.sleep()

        self.inner_func = wrapper

        return self

    def __get__(self, instance, owner):
        return partial(self.inner_func, instance)

    @staticmethod
    def process_all(container):
        fields.map(
            Rate.process,
            Rate,
            container,
        )

    def process(self):
        self.timer = rospy.Rate(self.hz, self.reset)


class Timer(object):

    def __init__(self, period, oneshot=False, reset=False, immediate=False):
        self.period = rospy.Duration.from_sec(period)
        self.callback = None
        self.oneshot = oneshot
        self.reset = reset
        self.timer = None

        if immediate:
            self.process()

    def __call__(self, f):
        self.callback = f
        return self

    @staticmethod
    def process_all(container):
        fields.map(
            Timer.process,
            Timer,
            container,
        )

    def process(self):
        self.start()

    def start(self):
        assert self.callback is not None, 'Missing callback'
        self.timer = rospy.Timer(self.period, self.callback, self.oneshot, self.reset)

    def cancel(self):
        assert self.timer is not None, 'Missing timer'
        self.timer.shutdown()
