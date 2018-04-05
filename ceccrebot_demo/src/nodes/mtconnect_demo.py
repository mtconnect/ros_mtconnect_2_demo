#!/usr/bin/env python
from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import sys
import os
sys.path.append(os.path.join(os.getenv('HOME'), 'Workspaces/ceccrebot/src/ros_mtconnect_2'))

import rospy
import mtconnect_bridge
from simulator.src.binding import Robot

class MTConnectDemo:
    def __init__(self):
        self._bridge = mtconnect_bridge.Bridge()
        self._robot = Robot()

        #Connect simulated bot to bridge
        self._robot.superstate.on_enter('base:operational:loading', self.loading_work)
        self._robot.superstate.on_enter('base:operational:unloading', self.unloading_work)

    def loading_work(self):
        self._bridge.do_work('move', 'conveyor')
        self._bridge.wait_for_work('move')
        self._bridge.do_work('move', 'cnc')
        self._bridge.wait_for_work('move')
        self._bridge.do_work('move', 'home')
        self._bridge.wait_for_work('move')
        self._robot.superstate.event('robot', 'robot', 'SubTask_MaterialLoad', 'COMPLETE')

    def unloading_work(self):
        self._bridge.do_work('move', 'place')
        self._bridge.wait_for_work('move')
        self._robot.superstate.event('robot', 'robot', 'SubTask_MaterialUnload', 'COMPLETE')

    def spin(self):
        #Let the threads started in the simulator run
        rospy.spin()

def main():
    rospy.init_node('mtconnect_demo')
    MTConnectDemo().spin()

if __name__ == '__main__':
    main()
