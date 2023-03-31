#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# =====================================================================================================
# Name: calibration_robot.py
# Author: Hélio Ochoa
# Version: 0.0
# Description: A .py file to calibrate the joint torques
# =====================================================================================================

# import roslib; roslib.load_manifest('kinova_thesis')
import rospy
import sys
import actionlib

import kinova_msgs.msg
from kinova_msgs.srv import *
import geometry_msgs.msg
import std_msgs.msg


def joint_position_client(angle_set, prefix):
    action_address = '/' + prefix + '_driver/joints_action/joint_angles'
    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.ArmJointAnglesAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmJointAnglesGoal()
    goal.angles.joint1 = angle_set[0]
    goal.angles.joint2 = angle_set[1]
    goal.angles.joint3 = angle_set[2]
    goal.angles.joint4 = angle_set[3]
    goal.angles.joint5 = angle_set[4]
    goal.angles.joint6 = angle_set[5]

    client.send_goal(goal)

    client.wait_for_result(rospy.Duration(100.0))

    # Prints out the result of executing the action
    return client.get_result()

# def cartesian_position_client(position, orientation, prefix):

#     action_address = '/' + prefix + 'driver/pose_action/tool_pose'
#     client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
#     client.wait_for_server()


#     goal = kinova_msgs.msg.ArmPoseGoal()
#     goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + 'link_base'))
#     goal.pose.pose.position=geometry_msgs.msg.Point(
#         x=position[0], y=position[1], z=position[2])
#     goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
#         x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

#     client.send_goal(goal)

#     if client.wait_for_result(rospy.Duration(10.0)):
#         return client.get_result()
#     else:
#         client.cancel_all_goals()
#         print('        the cartesian action timed-out')
#         return None

def cartesian_pose_client(position, orientation, prefix):
    """Send a cartesian goal to the action server."""
    action_address = '/' + prefix + 'driver/pose_action/tool_pose'
    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + 'link_base'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    # print('goal.pose in client 1: {}'.format(goal.pose.pose)) # debug

    client.send_goal(goal)

    if client.wait_for_result(rospy.Duration(10.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the cartesian action timed-out')
        return None

def main():
    rospy.init_node('calibration_robot')
    prefix = 'j2s6s300'

    angle_set = [180, 270, 90, 270, 270, 270]  # Reset Position
    joint_position_client(angle_set, prefix)

    # pose_set = [0.0,0.40,0.50, 0.707, -0.707, 0.0, 0.0] # [position (x, y, z)  orientation (x, y, z, w)]
    # result=cartesian_pose_client(pose_set[:3], pose_set[3:], prefix)
    rospy.sleep(3)

if __name__ == "__main__":

    try:
        main()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")