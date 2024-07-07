#!/usr/bin/python3
# -*- coding:utf-8 -*-
import rospy
from std_msgs.msg import Int32MultiArray
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from geometry_msgs.msg import Pose, Point, Quaternion
from agv_control.msg import mymsg
import subprocess

# 预定义目标位置
positions = {
    0: Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
    1: Pose(Point(3.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
    2: Pose(Point(4.799468994140625, 2.09700345993042, 0.0), Quaternion(0.0, 0.0, -0.7148333910454387, 0.6992948041037333)),
    3: Pose(Point(5.113161563873291, -1.3252441883087158, 0.0), Quaternion(0.0, 0.0, 0.7045460178352373, 0.7096583042228911)),
    4: Pose(Point(9.38815975189209, 1.542029857635498, 0.0), Quaternion(0.0, 0.0, -0.7352998851063133, 0.6777418970099477)),
    5: Pose(Point(9.580562591552734, -1.217545986175537, 0.0), Quaternion(0.0, 0.0, 0.7126849490706716, 0.7014842573915214)),
    6: Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
}

# 全局变量
current_sequence = []
goal_reached = True
arm_state_now=0
arm_state_last=0

def sequence_callback(data):
    global current_sequence
    print(data)
    current_sequence = data.data

def result_callback(msg):
    global goal_reached
    if msg.status.status == 3:  # SUCCEEDED
        rospy.loginfo("Goal reached successfully!")
        goal_reached = True
    elif msg.status.status in [4, 5, 9]:  # ABORTED, REJECTED, or LOST
        rospy.loginfo("Goal could not be reached.")
        goal_reached = True

def armstate_callback(msg):
    global arm_state_now
    arm_state_now=msg.num

def send_goal(pose):
    global goal_publisher
    goal = MoveBaseActionGoal()
    goal.goal.target_pose.header.frame_id = "map"
    goal.goal.target_pose.header.stamp = rospy.Time.now()
    goal.goal.target_pose.pose = pose

    rospy.loginfo("Sending goal: {}".format(pose))
    goal_publisher.publish(goal)

def main():
    global goal_publisher, goal_reached, current_sequence,arm_state_now,arm_state_last

    rospy.init_node('move_base_sequence')
    goal_publisher = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=10)
    rospy.Subscriber('/move_base/result', MoveBaseActionResult, result_callback)
    rospy.Subscriber('/web_data', Int32MultiArray, sequence_callback)
    rospy.Subscriber('/mass_and_vol', mymsg, armstate_callback)
    publisher = rospy.Publisher('/target_arm_position', Point, queue_size=10)

    point1 = Point()
    point1.x = 0
    point1.y = 0
    point1.z = 0

    point2 = Point()
    point2.x = 1
    point2.y = 1
    point2.z = 1

    rate = rospy.Rate(1)  # 1 
    while not rospy.is_shutdown():
        if current_sequence:
            for number in current_sequence:
                if number in positions:
                    send_goal(positions[number])
                    goal_reached = False
                    while not goal_reached:
                        rospy.sleep(1)
                    if number == 0 :
                        subprocess.run(["python3", "/home/agv/agv/src/agv_control/src/scripts/strero_camera.py"])
                        #publisher.publish(point)
                        while arm_state_now==arm_state_last:
                           rospy.sleep(1)
                        arm_state_last=arm_state_now
                        #rospy.sleep(3)
                    if number >1 and number <6 :
                        publisher.publish(point1)
                        while arm_state_now==arm_state_last:
                            rospy.sleep(1)
                        #rospy.sleep(45)
                        arm_state_last=arm_state_now
                    if number == 6:
                        publisher.publish(point2)
                        while arm_state_now==arm_state_last:
                            rospy.sleep(1)
                        #rospy.sleep(45)
                        arm_state_last=arm_state_now
            current_sequence = []
        rate.sleep()

if __name__ == '__main__':
    main()
