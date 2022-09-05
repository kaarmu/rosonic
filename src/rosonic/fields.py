from abc import ABC, abstractmethod
from typing import (
    Any,
    Callable,
    Dict,
    Generic,
    Iterable,
    Iterator,
    List,
    Set,
    Type,
    TypeVar,
)

__all__ = [
    'Field',
    'FieldContainer',
    'register',
    'unregister',
    'any',
    'map',
    'filter',
]


_any = any
_map = map
_filter = filter


C = TypeVar('C', bound='FieldContainer')
F = TypeVar('F', bound='Field')
R = TypeVar('R')
T = TypeVar('T')


class Field(Generic[R], ABC):

    @abstractmethod
    def process_one(self, *args: Any, **kwargs: Any) -> R:
        ...

    @classmethod
    def process_all(cls, container: Any, *args: Any, **kwargs: Any) -> List[R]:
        fn = lambda self: cls.process_one(self, *args, **kwargs)
        return list(map(fn, cls, container))


class FieldContainer(Generic[F]):

    __container_bags__: Dict[Type[F], Set[F]] = {}

    def __new__(cls, *args, **kwargs):
        """
        Load the field container.

        A dictionary of "bags" will be fetched from the container. The dictionary
        holds a bag for each field type registered for the container. If the type
        `type` is not found in the dictionary, it will be registered to enable
        recursion. Loading the bags is as simple as looking at all the fields in
        the container and adding the object to the corresponding bag.
        """

        # Load bags
        for t, bag in cls.__container_bags__.items():
            for x in _iter_attr(cls, t):
                bag.add(x)

        return super(FieldContainer, cls).__new__(cls, *args, **kwargs)


def _iter_attr(obj: Any, t: Type[T]) -> Iterable[T]:
    """
    Filter all attributes of an object by type.
    """
    for attr in vars(obj).values():
        if isinstance(attr, t):
            yield attr


def _recursive_map(
    fn: Callable[[FieldContainer], R],
    container: FieldContainer
) -> Iterable[R]:
    """
    Map function on all FieldContainer inside a FieldContainer.

    Due to limitations in typing, this is a special case of map.
    """
    yield from _map(fn, _iter_attr(container, FieldContainer))


def register(*types: type) -> Callable[[Type[C]], Type[C]]:
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
            cls.__container_bags__[t] = set()
        return cls

    return decorator


def unregister(*types: type) -> Callable[[C], C]:
    """
    Unregister field types from the container by removing the corresponding bags.

    Args:
        types (type)
    """

    def decorator(cls):
        for t in types:
            cls.__container_bags__.pop(t, None)
        return cls

    return decorator


def any(t: Type[Field], container: FieldContainer, recurse: bool = True) -> bool:
    """
    Returns true if `container` has any fields of type `t`.

    Mnemonic: Are there any `t` in `container`?

    If `recurse` is true then perform `any(t, x)` for any `x` of type `type`.
    """

    bags = container.__container_bags__

    # If there exist anything in bag the for `t`, then
    # `self` has a field of type `t`.
    if t in bags and bags[t]:
        return True

    if recurse:
        return _any(_recursive_map(lambda x: any(t, x), container))
    else:
        return False


class map(Iterable, Generic[F, R]):
    """
    For fields in `container` of type `t`, apply function `f`.

    Mnemonic: Map `f` on all `t` in `container`.

    If `recurse` is true then perform `any(f, t, x)` for any `x` of type
    `type`.

    Returns list of results from `f`.
    """


    def __init__(
        self,
        fn: Callable[[F], R],
        field_type: Type[F],
        container: FieldContainer,
        recurse: bool = True,
    ):

        self.fn = fn
        self.field_type = field_type
        self.container = container
        self.recurse = recurse

    def __iter__(self) -> Iterator[R]:

        bags = self.container.__container_bags__

        if self.field_type in bags:
            # normal map of _fn over fields of type _type
            yield from _map(self.fn, bags[self.field_type])

        if self.recurse:
            # create a map generator for each FieldContainer inside
            # _container and yield from those

            def gen(container: FieldContainer) -> Iterable[R]:
                yield from map(
                    self.fn,
                    self.field_type,
                    container,
                    recurse=self.recurse,
                )

            for g in _recursive_map(gen, self.container):
                yield from g


class filter(Iterable, Generic[F, R]):

    def __init__(
        self,
        fn: Callable[[F], R],
        field_type: Type[F],
        container: FieldContainer,
        recurse: bool = True,
    ):

        self.fn = fn
        self.field_type = field_type
        self.container = container
        self.recurse = recurse

    def __iter__(self) -> Iterator[R]:

        bags = self.container.__container_bags__

        if self.field_type in bags:
            # normal map of _fn over fields of type _type
            yield from _filter(self.fn, bags[self.field_type])

        if self.recurse:
            # create a map generator for each FieldContainer inside
            # _container and yield from those

            def gen(container: FieldContainer) -> Iterable[R]:
                yield from filter(
                    self.fn,
                    self.field_type,
                    container,
                    recurse=self.recurse,
                )

            for g in _recursive_map(gen, self.container):
                yield from g

