
BAGS_FIELD = '__container_bags__'


class Container(object):

    def __init__(self, t):
        self.t = t

    def __call__(self, cls):
        """Register a new bag for type `self.t` in `cls`."""

        # Create bag
        bags = getattr(cls, BAGS_FIELD, {})
        bags[self.t] = []

        # Load bag
        for _, x in vars(cls).items():
            if isinstance(x, self.t):
                bags[self.t].append(x)

        setattr(cls, BAGS_FIELD, bags)

        # Check if special bag for recursion exist
        if Container not in bags:
            Container(Container)(cls)

        return cls

    @staticmethod
    def map(f, t, self, recurse=True):
        """
        For fields in `self` of type `t`, apply function `f`.

        Always think the sentence:
            Map `f` on all `t` inside `self`

        If `recurse` is true then perform `Container.map`
        applied on fields of type `Container`.

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


    @Container(P)
    class N(object):

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


