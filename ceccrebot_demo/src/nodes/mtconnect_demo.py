#!/usr/bin/env python
from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import sys
import os
sys.path.append(os.path.join(os.getenv('HOME'), 'Workspaces/ceccrebot/src/ros_mtconnect_2'))

import rospy
import mtconnect_bridge
import simulator.src

class MTConnectDemo:
    def __init__(self):
        self._bridge = mtconnect_bridge.Bridge()
        self._robot = simulator.src.Robot()

        #Connect simulated bot to bridge
        self._robot.superstate.on_enter('base:activated:loading', self.loading_work)

    def loading_work(self):
        self._bridge.do_work('move', '{dest: cnc}')

    def spin(self):
        #TODO: how does this spin inside the mtconnect simulator?
        pass

def main():
    rospy.init_node('mtconnect_demo')
    MTConnectDemo().spin()

if __name__ == '__main__':
    main()
