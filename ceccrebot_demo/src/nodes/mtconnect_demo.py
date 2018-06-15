#!/usr/bin/env python
from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import sys
import os

import rospy
import mtconnect_bridge
#from simulator.src import Robot

class MTConnectDemo:
    def __init__(self):
        self._bridge = mtconnect_bridge.Bridge()
        #self._robot = Robot(host, port)

        #Connect simulated bot to bridge
        #self.after_move_in = self._robot.set_state_trigger('base:operational:loading:moving_in', self.move_to_cnc)
        #self.after_move_out = self._robot.set_state_trigger('base:operational:loading:moving_out', self.move_out_of_cnc)

    def move_to_cnc(self):
        rospy.loginfo("Demo moving to 'input_conveyor'")
        self._bridge.do_work('move', 'input_conveyor')
        rospy.loginfo("Demo moving to 'cnc'")
        self._bridge.do_work('move', 'cnc')
        rospy.loginfo("Demo moving to 'all_zeros'")
        self._bridge.do_work('move', 'all_zeros')
        #self.after_move_in()

    def move_out_of_cnc(self):
        rospy.loginfo("Demo moving to 'all_zeros'")
        self._bridge.do_work('move', 'all_zeros')
        #self.after_move_out()

    def spin(self):
        #Demo
        self.move_to_cnc()
        rospy.sleep(3.0)
        self.move_out_of_cnc()

        #Let the threads started in the simulator run
        rospy.spin()

def main():
    rospy.init_node('mtconnect_demo')
    MTConnectDemo().spin()

if __name__ == '__main__':
    main()
