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
        return 'tabletool'
    elif dev == 'cnc1_t1':
        return 'cnctool'

def map_device_destination(dev, dest):
    if dev != 'conv':
        dest = 'default'
    return dest

def map_device_payload(dev):
    if dev == 'cnctool':
        return 'tool'
    elif dev == 'tool':
        return 'tool'
    else:
        return 'part'

class RobotInterface:
    POINTS = {
        'cnc': {
            'default': {
                'in': ['waypoint', 'door', 'door2', 'pregrasp'],
                'out': ['pregrasp', 'door2', 'door', 'waypoint'],
            },
        },
        'cmm': {
             'default': {
                 'in': ['waypoint', 'waypoint2', 'waypoint3', 'pregrasp'],
                 'out': ['pregrasp', 'waypoint3', 'waypoint2', 'waypoint'],
             },
        },
        'conv': {
            'good': {
                'in': ['pregrasp'],
                'out': ['pregrasp'],
            },
            'bad': {
                'in': ['pregrasp'],
                'out': ['pregrasp'],
            },
            'rework': {
                'in': ['pregrasp'],
                'out': ['pregrasp'],
            },
            'default': {
                'in': ['pregrasp'],
                'out': ['pregrasp'],
            },
        },
        'buff': {
            'default': {
                'in': ['pregrasp'],
                'out': ['pregrasp'],
            },
        },
        'tabletool': {
            'default': {
                'in': ['pregrasp'],
                'out': ['pregrasp'],
            },
        },
        'cnctool': {
            'default': {
                'in': ['pregrasp'],
                'out': ['pregrasp'],
            },
        },
    }

    def __init__(self):
        self._bridge = mtconnect_bridge.Bridge()

    def try_work(self, action, data):
        rospy.loginfo("Doing work: {} - {}".format(str(action), str(data)))
        try:
            self._bridge.do_work(action, data)
        except mtconnect_bridge.MTConnectBridgeException as e:
            rospy.logerr("    failed: " + str(e))
            return False
        rospy.loginfo("    success")
        return True

    def move_sequence(self, targets):
        for target in targets:
            if not self.try_work('move', target)
                return False
        return True

    def move_home(self):
        if not self.try_work('move', 'home'):
            return False
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

    def grab(self, device, dest):
        devname = map_device_name(device)
        dest = map_device_destination(devname, dest)
        payload = map_device_payload(devname)

        prefix = devname + '_' + dest + '_'
        data = "close {}".format(payload)

        if not (self.try_work('move', prefix + 'pick') and self.try_work('gripper', data)):
            return False
        return True

    def release(self, device, dest):
        devname = map_device_name(device)
        dest = map_device_destination(devname, dest)
        payload = map_device_payload(devname)

        prefix = devname + '_' + dest + '_'
        data = "open {}".format(payload)

        if not (self.try_work('move', prefix + 'place') and self.try_work('gripper', data)):
            return False
        return True

def run_simulated_demo(robot):
    robot.move_home()

    robot.move_in('cnc1', '')
    robot.grab('cnc1')
    robot.move_out('cnc1', '')

    robot.move_in('t1', '')
    robot.grab('t1')
    rospy.sleep(3.0)
    robot.move_out('t1', '')

    robot.move_in('cmm1', '')
    robot.release('cmm1')
    rospy.sleep(3.0)
    robot.move_out('cmm1', '')

    rospy.sleep(3.0)

    robot.move_in('t1', '')
    robot.grab('t1')
    rospy.sleep(3.0)
    robot.move_out('t1', '')

    rospy.sleep(3.0)
    robot.move_out('cmm1', '')

    robot.move_in('t1', '')
    robot.release('t1')
    rospy.sleep(3.0)
    robot.move_out('t1', '')

    robot.move_in('cnc1_t1', '')
    robot.release('cnc1_t1')
    rospy.sleep(3.0)
    robot.move_out('cnc1_t1', '')

def init():
    rospy.init_node('mtconnect_demo')
    return RobotInterface()

def main():
    robot_interface = init()

    sim = rospy.get_param('~sim')
    if sim:
        run_simulated_demo(robot_interface)
    else:
        os.chdir(os.path.join(os.getenv("HOME"), "catkin_ws/src/ceccrebot/simulator/src"))

        host = rospy.get_param('~host')
        port = rospy.get_param('~port')
        robot_state_machine = Robot(host, port, robot_interface)
        robot_state_machine.superstate.enable()
        rospy.spin() #We're now at the mercy of the state machine

if __name__ == '__main__':
    main()
