#!/usr/bin/env python
from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import rospy
import actionlib
from  mtconnect_bridge.msg import DeviceWorkAction, DeviceWorkGoal

TIMEOUT = 5.0 #seconds
WORK_LIST = [
    {'type': 'move', 'data': 'conv_in'},
    {'type': 'move', 'data': 'cnc_wpt'},
    {'type': 'move', 'data': 'cnc'},
    {'type': 'move', 'data': 'cnc_wpt'},
    {'type': 'move', 'data': 'cmm_wpt'},
    {'type': 'move', 'data': 'cmm'},
    {'type': 'move', 'data': 'cmm_wpt'},
    {'type': 'move', 'data': 'conv_out'},
    {'type': 'move', 'data': 'allzeros'},
]

class TestCNCConveyor:
    def __init__(self):
        self.mtconnect_client = actionlib.SimpleActionClient("work", DeviceWorkAction)
        self.work_goals = [DeviceWorkGoal(type = item['type'], data = item['data']) for item in WORK_LIST]

    def execute(self):
        rospy.loginfo("Waiting for MTConnect device")
        self.mtconnect_client.wait_for_server()

        for item in self.work_goals:
            rospy.loginfo("Sending work: {}".format(item.type))
            self.mtconnect_client.send_goal(item)
            self.mtconnect_client.wait_for_result(rospy.Duration(TIMEOUT))
            rospy.loginfo("    work complete")

def main():
    rospy.init_node("test_cnc_conveyor")
    TestCNCConveyor().execute()

if __name__ == "__main__":
    main()
