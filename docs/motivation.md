# Motivation

Following are some problems outlined with ROS and `rospy` that motivates the
use of `rosonic`. However, no `rosonic` code will be found here.

## ROS Nodes

When creating a node in ROS, it will typically look something like any of the
examples below.

```python
#! /usr/bin/env python

import rospy


## Publisher pattern ##

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


## Subscriber pattern ##

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


## Pub-Sub pattern ##

def callback(msg, args):
    (pub1, pub2) = args

    pub1.publish(...)
    
    pub2.publish(...)

def main():

    # initialization
    pub1 = Publisher(...)
    pub2 = Publisher(...)
    Subscriber(..., callback, (pub1, pub2))

    # main loop
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('my_node')
    main()

```

Neither of these are very complicated. However, this is mainly since they
*follow a clear design pattern*. When developing, especially in research-
oriented projects,following design patterns are often second priority and
complexity tend to get out of hand. This is especially true when you start
using other ROS concepts more and more.

For new users there is also the overhead of learning what a ROS node is, how it
behaves and how they relate to other ROS concepts. It may be easy to explain
that ROS starts a separate process that runs your python script as a standalone
program, the script declares itself to be ROS node with `rospy.init_node`, 
opens communication channels for any of your topics, and then it executes your
logic. However, that is probably unclear for any developers/researcher that
gets thrown into ROS for the first time.

As a minor example, how should you design a subscribe-publish node? To clarify,
you node should subscribe to some topic, augment the incoming data then, on a 
new topic, publish it. Should you initialize in `main`? How does `callback` 
then have your `pub`? Maybe you declare `pub` globally, but that's a Python
anti-pattern! Of course you can solve this neatly and in many ways, but there 
is nothing in `rospy` that motivates you to follow best-practice and/or a good
design patter. Below is *one* solution where `pub` is sent as an argument to
`callback` but you can imagine how that can get out of hand with more publisher
and subscribers. The initialization is moved to a dedicated function and, like
previously, the node ends on `rospy.spin()`.

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

## ROS Parameters

> **Enough about design patterns!** I don't care about writing nice code, why 
> should I care about `rosonic`?

Now, let me tell you! Surely the most important problems to solve are those
that cause mild inconvenience. `rosonic` will help you once again before you
can say "ROS parameters"! Let's look at a typical scenario...

```python
#! /usr/bin/env python

import rospy

use_filter = True

def main():
    global use_filter

    # initialize

    has_image2 = rospy.has_param('~image2')
    assert has_image2, 'Parameter "image2" is missing!'

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
typical use case. Firstly, the distinction between `'~image1'` and `'image1'` 
is not very clear (a problem not solved by `rosonic`). Many times `'~image1'`
may have the default value `'image1'` as well! Secondly, asserting that 
required parameters, such as `'~image2'`, exists is clunky. Thirdly, parameter
values must be passed to different functions/scopes or declared global (same 
problem as `pub` before).
