
BAGS_FIELD = '__container_bags__'


class Container(object):

    def __new__(cls):

        bags = getattr(cls, BAGS_FIELD, {})

        # Check if special bag for recursion exist
        if Container not in bags:
            Container.register(Container)(cls)

        # Load bags
        for t, bag in bags.items():
            for _, x in vars(cls).items():
                if isinstance(x, t):
                    bags[t].append(x)

        return super(Container, cls).__new__(cls)

    @staticmethod
    def register(*types):
        """Register a new bag for type `t` in `cls`."""

        def decorator(cls):
            for t in types:
                bags = getattr(cls, BAGS_FIELD, {})
                bags[t] = []
                setattr(cls, BAGS_FIELD, bags)
            return cls

        return decorator

    @staticmethod
    def any(t, self, recurse=True):
        """
        Check if `self` contains any fields of type `t`.

        Mnemonic:
            Are there any `t` in `self`?

        If `recurse` is true then perform `Container.any(t, x)`
        for any `x` of type `Container`.

        Returns `True` or `False`.
        """
        bags = getattr(self, BAGS_FIELD, {})

        # If there exist anything in bag the for `t`, then
        # `self` has a field of type `t`.
        if t in bags and bags[t]:
            return True

        # `Container.map` will return a list of bools that is results
        # from `Container.any`.
        if recurse:
            return any(Container.map(
                lambda x: Container.any(t, x),
                Container,
                self,
                recurse=False,
            ))

        return False

    @staticmethod
    def map(f, t, self, recurse=True):
        """
        For fields in `self` of type `t`, apply function `f`.

        Mnemonic:
            Map `f` on all `t` inside `self`.

        If `recurse` is true then perform `Container.any(f, t, x)`
        for any `x` of type `Container`.

        Returns list of results from `f`.
        """

        values = []
        bags = getattr(self, BAGS_FIELD, {})

        if t in bags:
            # normal map of f over fields of type t
            values.extend(map(f, bags[t]))

        if recurse:
            Container.map(
                lambda x: values.extend(Container.map(f, t, x, recurse=recurse)),
                Container,
                self,
                recurse=False,
            )

        return values


#
# Example
#

if __name__ == '__main__':

    class P(object):

        def __init__(self, v):
            self.v = v

        def __get__(self, i, o):
            return self.v

        def load(self):
            print('  P(%d)' % self.v)
            self.v += 1


    @Container.register(P)
    class N(Container):

        a = P(1)
        b = P(2)

        @classmethod
        def load(cls):
            Container.map(P.load, P, cls)


    print('On class')
    print('--------')

    print('N.a', N.a)
    print('N.b', N.b)
    print()

    N.load()

    print()
    print('N.a', N.a)
    print('N.b', N.b)

    print()

    print('On instance')
    print('-----------')

    n = N()

    print('n.a', n.a)
    print('n.b', n.b)
    print()

    n.load()

    print()
    print('n.a', n.a)
    print('n.b', n.b)


