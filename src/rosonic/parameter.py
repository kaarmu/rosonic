from copy import deepcopy

import rospy

from .name import Name
from .container import Container

class ParameterError(Exception):
    pass

class Parameter(object):

    def __init__(self, name, value=None, optional=True, **kwargs):

        self.is_loaded = False
        self.is_optional = optional or value is not None
        self.func_stack = []

        self.name = Name(name, **kwargs)
        self.value = value

    def __repr__(self):
        return 'Parameter({}, {})'.format(str(self.name), self.value)

    def __get__(self, instance, owner):
        return self.value if self.is_loaded else self

    @staticmethod
    def process_parameters(container):
        Container.map(
            Parameter.load,
            Parameter,
            container,
        )

    def load(self):

        if self.is_optional:
            self.value = rospy.get_param(str(self.name), self.value)
        elif rospy.has_param(str(self.name)):
            self.value = rospy.get_param(str(self.name))
        else:
            raise ParameterError('Parameter server does not recognize "%s"' % str(self.name))

        for func in self.func_stack:
            self.value = func(self.value)

        self.is_loaded = True

    def copy(self):
        return deepcopy(self)

    def then(self, func):
        self.func_stack.append(func)
        return self

