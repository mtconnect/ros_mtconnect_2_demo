#!/usr/bin/env python
from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import sys
import os

import rospy
import mtconnect_bridge
#from simulator.src import Robot

class RobotInterface:
    def __init__(self):
        self._bridge = mtconnect_bridge.Bridge()

    def move_sequence(self, targets):
        for target in targets:
            rospy.loginfo("Moving to '{}'".format(target))
            self._bridge.do_work('move', target)
        return True

    def move_home(self):
        self._bridge.do_work('move', 'home')
        return True

    def move_in(self, device):
        move_targets = [
            'waypoint',
            'pregrasp',
            'grasp',
        ]
        device_targets = [device + '_' + target for target in move_targets]
        return self.move_sequence(device_targets)

    def move_out(self, device):
        move_targets = [
            'postgrasp',
            'waypoint',
        ]
        device_targets = [device + '_' + target for target in move_targets]
        return self.move_sequence(device_targets) and self.move_home()

    def grab(self, data):
        self._bridge.do_work('gripper', 'close')
        return True

    def release(self, data):
        self._bridge.do_work('gripper', 'open')
        return True

def run_simulated_demo(robot):
    robot.move_home()

    #robot.move_in('cnc')
    #robot.grab('')
    #robot.move_out('cnc')

    rospy.sleep(3.0)

    #robot.move_in('cmm')
    #robot.grab('')
    #robot.move_out('cmm')

    #rospy.sleep(3.0)

    robot.move_in('table')
    robot.release('')
    robot.move_out('table')

def init():
    rospy.init_node('mtconnect_demo')
    return RobotInterface()

def main():
    robot_interface = init()

    sim = rospy.get_param('~sim')
    if sim:
        run_simulated_demo(robot_interface)
    else:
        robot_state_machine = Robot(host, port, robot_interface)
        rospy.spin() #We're now at the mercy of the state machine

if __name__ == '__main__':
    main()
