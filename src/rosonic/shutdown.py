

class OnShutdown(object):

    def __call__(self, f):
        self.f = f
        return self

