# Get started

## Requirements

`rosonic` has been developed for ROS Melodic and above, i.e Python 2.7,
Python 3.6 and above. However, at writing moment only Python 3.6 has been
tested. As soon as I'm transitioning my other projects to Noetic then I will
drop any support for Python 2.7.

## Installation

This package is available on PyPI and can be installed as a normal pip package.

```shell
pip install rosonic
```

## Usage

There will be two nodes for counting numbers in this example. The first simply
publishes a `uint32` that increases each iteration until it wraps around
and starts over. The other node subscribes to the `uint32` and publishes a
`string` with binary, and possibly hexadecimal, representation. Let's start
by breaking down the code for the first node.

### Example 1

```python
#! /usr/bin/env python

from rosonic import Node, Parameter
from rospy import Publisher
from std_msgs.msg import UInt32


class uint_counter(Node):

    START = Parameter('~start', 0)

    rate = 4

    def __init__(self):
        self.value = self.START
        self.pub_counter = Publisher('counter', UInt32, queue_size=1)

    def spin(self):
        self.pub_counter.publish(self.value)
        self.value += 1
        self.value %= 32


if __name__ == '__main__':

    uint_counter(anonymous=True)
```

### Example 1 -- Explained

We import `Node` and `Parameter` from `rosonic`. You should still import other
`rospy` tools as you need them. There are neither a special `Publisher` or
`Subscriber` in `rosonic` yet so we need to use `rospy`'s for now.

```python
from rosonic import Node, Parameter
from rospy import Publisher
from std_msgs.msg import UInt32
```

We setup a class to declare the node. The name of the class will be the default
name of the node (just as when you call `rospy.init_node('<name>')`. The
special field `rate` will be turned into a `rospy.Rate` used to control how
often `spin` will be called.

```python
class uint_counter(Node):

    START = Parameter('~start', 0)

    rate = 4
```

One of the benefits of viewing nodes as classes is the clear separation of
initialization and the rest of its lifetime. During initialization we want
to create resources like publishers, subscribers, services, action-servers
and more. It is here we use parameters and setup the runtime behvaiour. In
`rosonic` we naturally utilize the `__init__` method with no arguments but
`self`.

```python
class uint_counter(Node):

    ...

    def __init__(self):
        self.value = self.START
        self.pub_counter = Publisher('counter', UInt32, queue_size=1)
```

The innovation of `rosonic` nodes is very simple, but constitutes of three
different methods. `__init__` that was previously mentioned, `main` and `spin`.
In most cases you will **not** want to change `main` whose default behaviour
will loop over a call `spin`. `spin`, on the other hand, is *meant* to be
overloaded. In this example we publish and update the `uint32` value.

```python
class uint_counter(Node):

    ...

    def spin(self):
        self.pub_counter.publish(self.value)
        self.value += 1
        self.value %= 32
```

Finally we are ready to start our node! As if it were a normal function we can
call `uint_counter` (with key-word arguments passed to `rospy.init_node`) and
run the code. This code snippet is required for every `rosonic`-style node.

```python
if __name__ == '__main__':

    uint_counter(anonymous=True)
```

### Example 2

```python
#! /usr/bin/env python

from rosonic import Node, Parameter
from rospy import Publisher, Subscriber
from std_msgs.msg import UInt32, String


class str_counter(Node):

    HEX = Parameter('~hex', False)

    def __init__(self):

        self.pub_bin_counter = Publisher(
            'bin',
            String,
            queue_size=1,
        )

        self.pub_hex_counter = Publisher(
            'hex',
            String,
            queue_size=1,
        )

        self.sub_uint_counter = Subscriber(
            'counter',
            UInt32,
            self.uint_counter_cb,
        )

    def uint_counter_cb(self, msg):

        data = msg.data
        hex_msg = String('hex=' + hex(data))
        bin_msg = String('bin=' + bin(data))

        if self.HEX:
            self.pub_hex_counter.publish(hex_msg)

        self.pub_bin_counter.publish(bin_msg)


if __name__ == '__main__':

    str_counter(anonymous=True)
```

### Example 2 -- Explained

Just like in Example 1 we start by declaring a `Node` class, any parameters
and its initialization. Notice that we have a subscriber this time!

```python
class str_counter(Node):

    HEX = Parameter('~hex', False)

    def __init__(self):

        self.pub_bin_counter = Publisher(
            'bin',
            String,
            queue_size=1,
        )

        self.pub_hex_counter = Publisher(
            'hex',
            String,
            queue_size=1,
        )

        self.sub_uint_counter = Subscriber(
            'counter',
            UInt32,
            self.uint_counter_cb,
        )
```

There is no problem using a method as the callback. Having `self` in the
callback allow us to use resources belonging to the class. It is therefore no
need to have any global variables and we can rest assured that `data`,
`hex_str` and `bin_str` are all contained in this scope.

```python
class str_counter(Node):

    ...

    def uint_counter_cb(self, msg):

        data = msg.data
        hex_msg = String('hex=' + hex(data))
        bin_msg = String('bin=' + bin(data))

        if self.HEX:
            self.pub_hex_counter.publish(hex_msg)

        self.pub_bin_counter.publish(bin_msg)
```

Then we start the node, just as we did last time! This time we did not overload
`spin` which is also fine. Let's look at Example 3 to investigate this.

```python
if __name__ == '__main__':

    str_counter(anonymous=True)
```

### Example 3

```python
#! /usr/bin/env python

from rosonic import Node


class dummy(Node):
    pass


if __name__ == '__main__':

    dummy(anonymous=True)
```

### Example 3 -- Explained

The purpose of `str_counter` was to react to `uint_counter` and only use its
one callback. Normally we would call `rospy.spin` at the end in such nodes.
However, this happens automatically in `rosonic`. Remember `__init__`, `main`
and `spin`? Well, by default `spin` calls `rospy.spin`! Lets look at the
default behaviour of our `dummy` node.

When `dummy` is called it will (as with any Python class) run `__init__` for
initialization. This is up to you to decide if it's necessary or not.

```python
if __name__ == '__main__':

    dummy(anonymous=True)
```

The next step, called internally by `rosonic`, is `main`. It will look
something like a while-loop that keeps calling `spin`. The loop is conditioned
by `keep_alive` (that you can also overload if necessary) that by default
returns `not rospy.is_shutdown()`.

```python
class Node(...):

    ...

    def main(self):
        while self.keep_alive():
            self.spin()
            self.rate.sleep()
        self.shutdown()
```

Then there is `spin` that calls `rospy.spin`. This essentially means that
after `__init__` is done, if neither `main` or `spin` is overloaded, then the
node will fall into `rospy.spin` until the node is shutdown.

```python
class Node(...):

    ...

    def spin(self):
        rospy.spin()
```

### What's next?

If you want to learn more about `rosonic`'s features I recommend you to read
the source code. You are also free to contribute with ideas through issue
tickets or PR's.
