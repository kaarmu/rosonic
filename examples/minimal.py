#! /usr/bin/env python3

from rosonic import *

class node(Node):

    @rate(20)
    def spin(self): self.loginfo('live')

    @on_shutdown()
    def fool(self): self.loginfo('fool')

if __name__ == '__main__':
    run(node)
