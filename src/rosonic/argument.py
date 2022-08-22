from argparse import ArgumentParser
import sys

import rospy

from .container import Container

ARGPARSE_FIELD = '__argparse__'


class Argument(object):

    dest = ''

    def __init__(self, *args, **kwargs):
        # save the arguments for add_argument
        self.args = args
        self.kwargs = kwargs

    def __get__(self, instance, owner):
        namespace = getattr(owner, ARGPARSE_FIELD)
        return getattr(namespace, self.dest, None)

    def __repr__(self):
        return 'Argument({}, {}, {})'.format(self.dest, self.args, self.kwargs)

    @staticmethod
    def process_arguments(container, *args, **kwargs):
        if Container.any(Argument, container):
            # Create parser
            parser = ArgumentParser(*args, **kwargs)
            Container.map(
                lambda self: self.add_argument(parser),
                Argument,
                container,
            )
            # Parse argv, making sure to strip any special ROS args
            argv = rospy.myargv(argv=sys.argv)
            setattr(container, ARGPARSE_FIELD, parser.parse_args(argv))

    def add_argument(self, parser):
        store_action = parser.add_argument(*self.args, **self.kwargs)
        self.dest = store_action.dest


#
# Example
#

if __name__ == '__main__':

    @Container(Argument)
    class Node(object):
        foo = Argument('foo', nargs=3)
        bar = Argument('bar', nargs=2)

    Argument.parse_from(Node, prog='Node')

    node = Node()
    print('Node.foo', node.foo)
    print('Node.bar', node.bar)

