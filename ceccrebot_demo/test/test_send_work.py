#!/usr/bin/env python
from __future__ import absolute_import, division, print_function, unicode_literals
__metaclass__ = type

import sys
import rospy
import actionlib
from mtconnect_bridge.msg import DeviceWorkAction, DeviceWorkGoal

TIMEOUT=30.0

def run(client):
    move_items = [
        #'high_center',
        #'cnc_door',
        #'cnc_pregrasp',
        #'cnc_grasp',
        #'cnc_pregrasp',
        #'cnc_door',
        #'high_center',
        #'table1_pregrasp',
        #'table1_grasp',
        #'table1_pregrasp',
        #'home',
        #'cmm_waypoint',
        #'cmm_pregrasp',
        #'cmm_grasp',
        #'cmm_pregrasp',
        #'cmm_waypoint',
        #'acc_test_approach',
        #'acc_test_point',
        'home',
        #'high_center',
    ]
    for item in move_items:
        rospy.loginfo("sending: " + item)
        goal = DeviceWorkGoal(type='move', data=item)
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(TIMEOUT))
        rospy.loginfo("  result: {}".format(str(client.get_state())))
        client.cancel_all_goals()
        rospy.sleep(3.0)

def run_one(client):
    item = DeviceWorkGoal(type=sys.argv[1], data=sys.argv[2])

    rospy.loginfo("Sending work: {}".format(item.type))
    client.send_goal(item)
    client.wait_for_result(rospy.Duration(TIMEOUT))
    rospy.loginfo("    work complete")

def main():
    rospy.init_node("test_send_work")

    mtconnect_client = actionlib.SimpleActionClient("ur/work", DeviceWorkAction)
    rospy.loginfo("Waiting for MTConnect device")
    mtconnect_client.wait_for_server()
    rospy.loginfo("MTConnect device available")

    run(mtconnect_client)

if __name__ == "__main__":
    main()
