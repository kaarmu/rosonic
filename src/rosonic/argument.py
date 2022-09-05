import sys
from argparse import ArgumentParser
from typing import Any, Dict, Tuple

import rospy

from . import fields
from .fields import Field, FieldContainer

ARGPARSE_FIELD = '__argparse__'


class Argument(Field):

    dest: str
    args: Tuple[Any]
    kwargs: Dict[str, Any]

    def __init__(self, *args: Any, **kwargs: Any):
        # save the arguments for add_argument
        self.args = args
        self.kwargs = kwargs

    def __get__(self, instance, owner):
        namespace = getattr(owner, ARGPARSE_FIELD)
        return getattr(namespace, self.dest, None)

    def __repr__(self) -> str:
        return 'Argument({}, {}, {})'.format(self.dest, self.args, self.kwargs)

    def process_one(self, parser: ArgumentParser):
        store_action = parser.add_argument(*self.args, **self.kwargs)
        self.dest = store_action.dest

    @classmethod
    def process_all(cls, container: FieldContainer, *args, **kwargs):
        if fields.any(Argument, container):
            # Create parser
            parser = ArgumentParser(*args, **kwargs)
            fields.map(
                lambda self: self.process_one(parser),
                Argument,
                container,
            )
            # Parse argv, making sure to strip any special ROS args
            argv = rospy.myargv(argv=sys.argv)
            setattr(container, ARGPARSE_FIELD, parser.parse_args(argv))

