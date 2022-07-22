from .parameter import Parameter


class Container(object):

    @classmethod
    def iter_params(cls, deep=True):
        for _, x in vars(cls).items():
            if isinstance(x, Parameter):
                yield x
            if deep and isinstance(x, Container):
                for xx in x.iter_params(deep=deep):
                    yield xx

    @classmethod
    def load_params(cls):
        for x in cls.iter_params():
            x.load()

