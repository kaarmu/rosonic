from functools import partial

import rospy

from .container import Container


class OnShutdown(object):

    def __init__(self, bind=True):
        self.bind = bind
        self.f = lambda _: None

    def __call__(self, f):
        self.f = f
        return self

    @staticmethod
    def process_onshutdown(container):

        # Bind shutdown callbacks to container
        fs = Container.map(
            lambda self: (
                partial(self.f, container) if self.bind else 
                self.f
            ),
            OnShutdown,
            container
        )

        # register callbacks with rospy
        for f in fs:
            rospy.on_shutdown(f)

