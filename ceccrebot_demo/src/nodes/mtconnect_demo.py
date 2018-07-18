#!/usr/bin/env python
from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import sys
import os

import rospy
import mtconnect_bridge
#from simulator.src import Robot

class RobotTaskInterface:
    def __init__(self, bridge):
        self._bridge = bridge

    def move_sequence(self, targets):
        for target in targets:
            rospy.loginfo("Moving to '{}'".format(target))
            self._bridge.do_work('move', target)
        return True

    def move_in(self, data):
        move_targets = [
            'cnc_door',
            'cnc_pregrasp',
            'cnc_grasp',
        ]
        return self.move_sequence(move_targets)

    def move_out(self, data):
        move_targets = [
            'cnc_postgrasp',
            'cnc_door'
        ]
        return self.move_sequence(move_targets)

    def grab(self, data):
        self._bridge.do_work('gripper', 'close')
        return True

    def release(self, data):
        self._bridge.do_work('gripper', 'open')
        return True

def run_simulated_demo(task_interface):
    task_interface.move_in('cnc')
    rospy.sleep(3.0)
    task_interface.move_out('cnc')

def main():
    rospy.init_node('mtconnect_demo')
    sim = rospy.get_param('~sim')

    bridge = mtconnect_bridge.Bridge()
    task_interface = RobotTaskInterface(self._bridge)

    if sim:
        run_simulated_demo()
    else:
        robot = Robot(host, port, task_interface)
        rospy.spin()

if __name__ == '__main__':
    main()
