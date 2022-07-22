# rosonic

This is the library to make you as fast as Sonic in developing python-based ROS
packages. The name does not actually come from Sonic, but ROS-(pyth)onic as the
main motivation is making ROS nodes more pythonic.

## Motivation

When creating a node in ROS, typically it will look something like this

```python
#! /usr/bin/env python

import rospy


## Publisher design pattern

def main():

    # initialization
    pub = rospy.Publisher(...)
    rate = rospy.Rate(...)

    # main loop
    while not rospy.is_shutdown():
        pub.publish(...)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('my_node')

    try:
        main()
    except rospy.ROSInterruptException:
        pass


## Subscriber design pattern

def callback(msg):
    ...

def main():

    # initialization
    Subscriber(..., callback)

    # main loop
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('my_node')
    main()

```

Neither of these are very complicated. However, this is mainly since they *follow
a clear design pattern*. When developing, especially in research-oriented projects,
following design patterns are often second priority and complexity gets out of hand.
This is especially true when you start using ROS parameters, have multiple
publishers and subscribers, and also services.

For new users there is also the overhead of learning what a ROS node is, how it
behaves and how they relate to other ROS concepts, e.g. ROS parameters. It may
be easy to explain that ROS starts a separate process that runs your python script
as a standalone program, the script declares itself to be ROS node with
`rospy.init_node`, opens communication channels for any of your topics, and
then it executes your logic. However, that is probably unclear for any
developers/researcher that gets thrown into ROS for the first time.

As a minor example, how should you design a subscribe-publish node? To clarify,
you node should subscribe to some topic, augment the incoming data then, on a new
topic, publish it. Should you initialize in `main`? How does `callback` then have
your `pub`? Maybe you declare `pub` globally, but that's a Python anti-pattern! Of
course you can solve this neatly, but there is probably no clear solution that
everyone agrees on. Below is *one* solution where `pub` is sent as an argument to
`callback`. The initialization is moved to a dedicated function and like previously
the node ends on `rospy.spin()`.

```python
#! /usr/bin/env python

import rospy

def callback(msg, pub):
    pub.publish(...)

def init():
    pub = rospy.Publisher(...)
    rospy.Subscriber(..., callback, pub)

if __name__ == '__main__':
    rospy.init_node('my_node')
    init()
    rospy.spin()

```

To alleviate some of these issues, primarily what ROS nodes are and how they
interact with `launch`-files through ROS parameters, `rosonic` tries to gently
"force" the user to follow a design pattern. Namely, the package conceptualizes
a ROS node as a class in a simple, pythonic way with reasonable default behaviours.
The resulting developer experience will be similar to the examples above; define
your callbacks and resources during initialization and then overload the `main`
logic.

> **Enough about design patterns!** I don't care about writing nice code, why should
> I care about `rosonic`?

Now, let me tell you! Surely the most important problems to solve are those that
cause mild inconvenience. `rosonic` will help you once again before you can say
"ROS parameters"! Let's look at a typical scenario...

```python
#! /usr/bin/env python

import rospy

use_filter = True

def main():
    global use_filter

    # initialize

    has_image2 = rospy.has_param('~image2')
    assert has_image2

    image1 = rospy.get_param('~image1', 'image1')
    image2 = rospy.get_param('~image2')
    use_filter = rospy.get_param('~use_filter', True)

    pub = rospy.Publisher(image1, ...)
    Subscriber(image2, ...)

    # main loop

    rospy.spin()

def callback(msg):

    if use_filter:
        ...

    ...


if __name__ == '__main__':
    main()
```


There are multiple problems with passing topic names through parameters, a very
typical use case. Firstly, the distinction between `'~image1'` and `'image1'` is
not very clear (a problem not solved by `rosonic`). Many times `'~image1'`
may be written as `'image1'` as well! Secondly, asserting required parameters,
such as `'~image2'`, exists is clunky. Thirdly, passing parameter values to
different functions/scopes (same problem as `pub` before). As you will see,
`rosonic` solves these two latter problems in quite an elegant way!


## Usage

### Creating a Node

Using `rosonic`, the barebones, do-nothing ROS node now looks like this

```python
#! /usr/bin/env python

from rospy import Publisher
from rosonic import Node

class my_node(Node):
    pass

if __name__ == '__main__':
    my_node.run()
```

The default behaviour of a `rosonic.Node` will be to call `rospy.spin`. This is
useful for the "subscribe" and "subscribe-publish" nodes, then we only need to
implement initialization and callbacks. For the initialization, just create a normal
`__init__` for the class that doesn't take any arguments. Any
resources, e.g. `pub`, can be set as object fields (`self.pub`) and callbacks
can be implemented as methods.

```python
class my_node(Node):

    def __init__(self):

        self.pub = rospy.Publisher(...)
        rospy.Subscriber(..., self.callback)

    def callback(self):

        self.pub.publish(...)
```

When calling `my_node.run` you (in order) initialize the ROS node with
the class name, load parameters, create the `my_node` instance, calls
`my_node.main`. The `main` method will call `spin` which, by default,
calls `rospy.spin`.

```python
def main(self):
    """Runs default behaviour."""
    while self.keep_alive():
        self.spin()
    self.shutdown()

def spin(self):
    rospy.spin()
```

The added bonus of these defaults is that it is very easy to create the "publish"
nodes. You need to use the `Rate` decorator, write `__init__` as before and implement
the `spin` pass of `main`'s loop. If, however, you want completely different
behaviour then you'll need to overload `main`, which is fairly simple in itself!

```python
from rosonic import Node, Rate

class my_node(Node):

    def __init__(self):
        ...

    @Rate(...)
    def spin(self):
        ...
```

*(Nice to have)* For convenience, `rospy`'s logging functions are added as
fields to `Node`. That means you can call `self.loginfo`. However, even better
may be `self.logevent` which also writes the nodes name! Super useful in your
`__init__` when creating a subscriber/publisher to a new topic.

### Parameters as class attributes

> It isn't the mountain ahead that wears you out - it's the grain of sand in your shoe

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
    my_node.run()
```

That is the `rosonic` way eitherway, so to say. You don't have to care about
existence checks or scoping rules. Since they are class fields you have them
available all the time. If `Parameter` is given a default value or the
`required` argument is switched then it will pass on, otherwise `Parameter`
checks for existence at load time.


### A very slim implementation

`rosonic` is very slim. It takes 5 minutes to look through the entire
implementation. So, final advice, let the implementation be the
documentation!

## Installation

Simply clone this repository to your workspace's `src` directory.

