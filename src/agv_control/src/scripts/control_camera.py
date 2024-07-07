#!/usr/bin/python3
# -*- coding:utf-8 -*-
import rospy
import actionlib
import subprocess
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult

def move_base_result_callback(result):
    rospy.loginfo("Move base goal status received.")
    if result.status == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Navigation successful, launching stereo_camera.launch")
        try:
            subprocess.Popen(["roslaunch", "agv_control", "stereo_camera.launch"])
        except Exception as e:
            rospy.logerr("Failed to launch stereo_camera.launch: %s", str(e))
    else:
        rospy.logwarn("Navigation failed with status: %d", result.status)

def move_base_result_listener():
    rospy.init_node('move_base_result_listener', anonymous=True)
    rospy.Subscriber('/move_base/result', MoveBaseAction, move_base_result_callback)
    rospy.loginfo("Subscribed to /move_base/result")
    rospy.spin()

if __name__ == '__main__':
    try:
        move_base_result_listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("move_base_result_listener node terminated.")
