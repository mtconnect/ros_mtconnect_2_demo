#!/usr/bin/env python
from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import sys
import os

import rospy
import mtconnect_bridge
from simulator.src.robot import Robot

def map_device_name(dev):
    if dev == 'cnc1':
        return 'table1'
    elif dev == 'conv1':
        return 'table1'
    elif dev == 'cmm1':
        return 'table2'
    elif dev == 'b1':
        return 'table3'
    elif dev == 't1':
        return 'table4'

class RobotInterface:
    POINTS = {
        'cnc': {
            'in': ['waypoint', 'pregrasp', 'grasp'],
            'out': ['pregrasp', 'waypoint'],
        },
        'cmm': {
            'in': ['waypoint', 'pregrasp', 'grasp'],
            'out': ['pregrasp', 'waypoint'],
        },
        'table1': {
            'in': ['pregrasp', 'grasp'],
            'out': ['pregrasp'],
        },
        'table2': {
            'in': ['pregrasp', 'grasp'],
            'out': ['pregrasp'],
        },
        'table3': {
            'in': ['pregrasp', 'grasp'],
            'out': ['pregrasp'],
        },
        'table4': {
            'in': ['pregrasp', 'grasp'],
            'out': ['pregrasp'],
        },
    }

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

    def move_in(self, device, dest):
        print('Robot move_in to dev: {}, dest: {}'.format(device, dest))
        loc = map_device_name(device)
        move_targets = self.POINTS[loc]['in']
        device_targets = [loc + '_' + target for target in move_targets]
        return self.move_sequence(device_targets)

    def move_out(self, device, dest):
        print('Robot move_in to dev: {}, dest: {}'.format(device, dest))
        loc = map_device_name(device)
        move_targets = self.POINTS[loc]['out']
        device_targets = [loc + '_' + target for target in move_targets]
        return self.move_sequence(device_targets) and self.move_home()

    def grab(self, data):
        self._bridge.do_work('gripper', 'close')
        return True

    def release(self, data):
        self._bridge.do_work('gripper', 'open')
        return True

def run_simulated_demo(robot):
    robot.move_home()

    robot.move_in('cnc1', '')
    robot.grab('')
    robot.move_out('cnc1', '')

    rospy.sleep(3.0)

    robot.move_in('cmm1', '')
    robot.release('')
    robot.move_out('cmm1', '')

    rospy.sleep(3.0)

    robot.move_in('t1', '')
    robot.grab('')
    robot.move_out('t1', '')

    rospy.sleep(3.0)

    robot.move_in('b1', '')
    robot.release('')
    robot.move_out('b1', '')

def init():
    rospy.init_node('mtconnect_demo')
    return RobotInterface()

def main():
    robot_interface = init()

    sim = rospy.get_param('~sim')
    if sim:
        run_simulated_demo(robot_interface)
    else:
        rospy.loginfo("In dir: " + os.getcwd())
        os.chdir(os.path.join(os.getenv("HOME"), "Workspaces/ceccrebot/src/ceccrebot/simulator/src"))
        rospy.loginfo("In dir: " + os.getcwd())

        host = rospy.get_param('~host')
        port = rospy.get_param('~port')
        robot_state_machine = Robot(host, port, robot_interface)
        robot_state_machine.superstate.enable()
        rospy.spin() #We're now at the mercy of the state machine

if __name__ == '__main__':
    main()
