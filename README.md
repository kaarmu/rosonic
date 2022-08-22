# rosonic

This is the library to make you as fast as Sonic in developing python-based ROS
packages. The name does not actually come from Sonic, but "ROS (made pyth)onic"
since the main motivation is making ROS nodes, well, more pythonic. Doing this
allows for a more comfortable workflow, the code becomes more "natural" and
easier to understand.

To alleviate some of the issues described in [Motivation](docs/motivation.md)
`rosonic` tries to gently "force" the user to follow a design pattern. Namely, 
`rosonic` conceptualizes a ROS node as a class in a simple, pythonic way with 
reasonable default behaviours. 


## Requirements

`rosonic` has been developed for ROS Melodic and above, i.e Python 2.7, 
Python 3.6 and above. However, at writing moment only Python 3.6 has been
tested. As soon as I'm transitioning my other projects to Noetic then I will
drop any support for Python 2.7.


## Installation

This package is available on PyPI and can be installed as a normal pip package.

```
pip install rosonic
```


## Usage

### Creating a Node

The `rosonic` barebones, do-nothing ROS node now looks like this

```python
#! /usr/bin/env python

from rosonic import Node

class my_node(Node):
    pass

if __name__ == '__main__':
    my_node()
```

The default behaviour of a `rosonic.Node` will be to call `rospy.spin`. This is
useful for the "Subscriber" and "Pub-Sub" patterns in 
[Motivation](docs/motivation.md), then we only need to implement initialization
and callbacks. For the initialization, just create a normal `__init__` for the
class that doesn't take any arguments. Any resources, e.g. `pub`, can be set as
object fields (`self.pub`) and callbacks can be implemented as methods.

```python
class my_node(Node):

    def __init__(self):

        self.pub = rospy.Publisher(...)
        rospy.Subscriber(..., self.callback)

    def callback(self):

        self.pub.publish(...)
```

When calling `my_node()` you (in order) initialize the ROS node with
the class name, load parameters, create the `my_node` instance and call
`my_node.main`. The default `main` method will call the `spin` which in turn,
by default, calls `rospy.spin`.

```python
def main(self):
    while self.keep_alive():
        self.spin()
    self.shutdown()

def spin(self):
    rospy.spin()
```

The added bonus of these defaults is that it is very easy to do the "Publisher"
pattern as well. You specify a `rate`, write `__init__` as before and overload
the `spin` method. If, however, you want completely different still then you
need to overload `main`, which is fairly simple in itself!

```python
from rosonic import Node, Rate

class my_node(Node):

    rate = 10

    def __init__(self):
        ...

    def spin(self):
        ...
```

### Parameters as class attributes

> It isn't the mountain ahead that wears you out - it's the grain of sand in 
> your shoe

What's the easiest way to declare a parameter? Maybe like this...

```python
#! /usr/bin/env python

from rosonic import Node, Parameter

class my_node(Node):

    IMAGE1 = Parameter('~image1', 'image1')
    IMAGE2 = Parameter('~image2', 'image2')
    USE_FILTER = Parameter('~use_filter', True)

    ...

if __name__ == '__main__':
    my_node()
```

That's the `rosonic` way at least! You don't have to care about asserting 
existence or managing scoping rules. Since they are class fields you have them
available all the time. If `Parameter` is not given a default value nor is the
`optional` argument set, then `Parameter` checks for existence at load time.


## A very slim implementation

`rosonic` tries to be very slim. There is some complexity regarding class 
fields. However, it takes 5 minutes to look through the ROS relevant parts. So,
see the implementation as the real documentation!
