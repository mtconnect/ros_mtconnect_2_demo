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
        return 'cnc'
    elif dev == 'conv1':
        return 'conv'
    elif dev == 'cmm1':
        return 'cmm'
    elif dev == 'b1':
        return 'buff'
    elif dev == 't1':
        return 'tool'
    elif dev == 'cnc_t1':
        return 'cnctool'

def map_device_destination(dev, dest):
    if dev != 'conv':
        dest = 'default'

    return dest


class RobotInterface:
    POINTS = {
        'cnc': {
            'default': {
                'in': ['waypoint', 'door', 'pregrasp', 'grasp'],
                'out': ['pregrasp', 'waypoint'],
            },
        },
        'cmm': {
             'default': {
                 'in': ['waypoint', 'pregrasp', 'grasp'],
                 'out': ['pregrasp', 'waypoint'],
             },
        },
        'conv': {
            'good': {
                'in': ['pregrasp', 'grasp'],
                'out': ['pregrasp'],
            },
            'bad': {
                'in': ['pregrasp', 'grasp'],
                'out': ['pregrasp'],
            },
            'rework': {
                'in': ['pregrasp', 'grasp'],
                'out': ['pregrasp'],
            },
            'default': {
                'in': ['pregrasp', 'grasp'],
                'out': ['pregrasp'],
            },
        },
        'buff': {
            'default': {
                'in': ['pregrasp', 'grasp'],
                'out': ['pregrasp'],
            },
        },
        'tool': {
            'default': {
                'in': ['pregrasp', 'grasp'],
                'out': ['pregrasp'],
            },
        },
        'cnctool': {
            'default': {
                'in': ['pregrasp', 'grasp'],
                'out': ['pregrasp'],
            },
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
        dest = map_device_destination(loc, dest)
        move_targets = self.POINTS[loc][dest]['in']
        device_targets = [loc + '_' + dest + '_' +  target for target in move_targets]
        return self.move_sequence(device_targets)

    def move_out(self, device, dest):
        print('Robot move_in to dev: {}, dest: {}'.format(device, dest))
        loc = map_device_name(device)
        dest = map_device_destination(loc, dest)
        move_targets = self.POINTS[loc][dest]['out']
        device_targets = [loc + '_' + dest + '_' + target for target in move_targets]
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
        os.chdir(os.path.join(os.getenv("HOME"), "catkin_ws/src/ceccrebot/simulator/src"))
        rospy.loginfo("In dir: " + os.getcwd())

        host = rospy.get_param('~host')
        port = rospy.get_param('~port')
        robot_state_machine = Robot(host, port, robot_interface)
        robot_state_machine.superstate.enable()
        rospy.spin() #We're now at the mercy of the state machine

if __name__ == '__main__':
    main()
