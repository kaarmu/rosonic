
BAGS_FIELD = '__container_bags__'

_any = any
_map = map

__all__ = [
    'is_container',
    'load',
    'register',
    'unregister',
    'any',
    'map',
]

################################
## Managing a field container ##
################################

def is_container(t):
    """
    A container is a type instance which has the bags attribute.

    Args:
        t (type)
    """
    return isinstance(t, type) and hasattr(t, BAGS_FIELD)

def load(container, recurse=True):
    """
    Load the field container.

    A dictionary of "bags" will be fetched from the container. The dictionary
    holds a bag for each field type registered for the container. If the type
    `type` is not found in the dictionary, it will be registered to enable
    recursion. Loading the bags is as simple as looking at all the fields in
    the container and adding the object to the corresponding bag.

    Args:
        container (type)
    """

    bags = getattr(container, BAGS_FIELD, {})

    # Register type to enable recursion
    if recurse and type not in bags:
        register(type)(container)

    # Load bags
    for t, bag in bags.items():
        for _, x in vars(container).items():
            if isinstance(x, t):
                bag.add(x)

    # Load bags recursively
    if recurse:
        map(
            lambda t: is_container(t) and load(t),
            type,
            container,
        )

def register(*types):
    """
    Register new field type(s) and create corresponding bags.

    This function returns a decorator. The decorator will register each given
    argument as a new field type.

    Args:
        types (type)

    Example:

        @register(Parameter)
        class my_node(Node):
            ...

    """

    def decorator(cls):
        for t in types:
            bags = getattr(cls, BAGS_FIELD, {})
            bags[t] = set()
            setattr(cls, BAGS_FIELD, bags)
        return cls

    return decorator


def unregister(*types):
    """
    Unregister field types from the container by removing the corresponding bags.

    Args:
        types (type)
    """

    def decorator(cls):
        for t in types:
            bags = getattr(cls, BAGS_FIELD, {})
            bags.pop(t, None)
        return cls

    return decorator

def any(t, container, recurse=True):
    """
    Returns true if `container` has any fields of type `t`.

    Mnemonic: Are there any `t` in `container`?

    If `recurse` is true then perform `any(t, x)` for any `x` of type `type`.

    Args:
        t (type)
        container (type)
    """
    bags = getattr(container, BAGS_FIELD, {})

    # If there exist anything in bag the for `t`, then
    # `self` has a field of type `t`.
    if t in bags and bags[t]:
        return True

    # `map` will return a list of bools
    if recurse:
        return _any(map(
            lambda x: any(t, x),
            type,
            container,
            recurse=False,
        ))

    return False


def map(f, t, container, recurse=True):
    """
    For fields in `container` of type `t`, apply function `f`.

    Mnemonic: Map `f` on all `t` in `container`.

    If `recurse` is true then perform `any(f, t, x)` for any `x` of type
    `type`.

    Returns list of results from `f`.

    Args:
        f (Callable)
        t (type)
        container (type)
    """

    values = []
    bags = getattr(container, BAGS_FIELD, {})

    if t in bags:
        # normal map of f over fields of type t
        values.extend(_map(f, bags[t]))

    if recurse:
        map(
            lambda x: values.extend(map(f, t, x, recurse=recurse)),
            type,
            container,
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

        def increment(self):
            self.v += 1


    @register(P)
    class N(object):

        a = P(1)
        b = P(2)

        @classmethod
        def load(cls):
            map(P.increment, P, cls)

    load(N)

    print('N.a', N.a)
    print('N.b', N.b)

    N.load()
    print()

    print('N.a', N.a)
    print('N.b', N.b)

