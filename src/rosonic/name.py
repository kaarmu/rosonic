from typing import List, Union, Sequence

import rospy

__all__ = [
    'Name',
]

class Name:

    name: str
    namespace: List[str]
    is_global: bool
    is_private: bool
    is_basename: bool

    def __init__(
        self,
        name: Union[str, None] = None,
        *,
        namespace: Union[Sequence[str], str, None] = None,
        is_private: bool = False,
        is_global: bool = False,
    ):

        if name is None:
            name = rospy.get_name()

        assert isinstance(name, str), 'Name must be a string'
        assert name, 'Empty string is not a valid name'

        self.is_private = name.startswith('~') or is_private
        self.is_global = name.startswith('/') or is_global
        self.is_basename = '/' not in name and not self.is_private
        self.namespace = []
        self.name = ''

        if namespace is None:
            namespace = []
        else:
            assert not self.is_global, 'Cannot combine name %s with a namespace argument' % name
            assert not self.is_private, 'Cannot combine name %s with a namespace argument' % name
            self.is_global = True
            self.is_basename = False

        # str is a Sequence but for semantics...
        if not isinstance(namespace, (Sequence, str)):
            raise TypeError('Invalid type on argument namespace, expected str, list or tuple')

        if isinstance(namespace, str):
            if namespace.endswith('/'):
                namespace = namespace[:-1]
            self.namespace += namespace.split('/')
        else:
            self.namespace += namespace

        assert all(map(bool, self.namespace)), 'namespace argument contains an empty name (double-slashes)'

        i = int(self.is_private or self.is_global)
        assert '~' not in name[i:], 'Invalid ROS name ' + name
        assert not name.endswith('/'), 'Name %s cannot end with /' % name
        parts = name[i:].split('/')

        initial_alpha = lambda name: name[0].isalpha()
        subseq_alnums = lambda name: name[1:].replace('_', '').isalnum()
        name_test = lambda name: initial_alpha(name) and subseq_alnums(name)
        assert all(map(bool, parts)), 'name argument contains an empty name (double-slashes)'
        assert all(map(name_test, parts)),  'Invalid ROS name ' + name

        self.namespace.extend(parts[:-1])
        self.base = parts[-1]

    def __repr__(self) -> str:
        return 'Name({})'.format(str(self))

    def __str__(self) -> str:
        s = ''
        if self.is_private:
            s += '~'
        if self.is_global:
            s += '/'
        s += '/'.join(self.namespace + [self.base])
        return s

    def replace_base(self, other: Union['Name', str]) -> 'Name':
        """Replace base of self with other."""
        if isinstance(other, str):
            other = Name(other)
        assert not (other.is_private or other.is_global), 'Cannot combine name=%s with other=%s' % (self, other)
        self.namespace += other.namespace
        self.base = other.base
        return self
