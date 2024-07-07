#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped

class TwistPublisher:
    def __init__(self):
        rospy.init_node('twist_from_cmd_vel_publisher')
        self.twist_pub = rospy.Publisher('/twist_car', TwistWithCovarianceStamped, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)

    def cmd_callback(self, msg):
        twist = TwistWithCovarianceStamped()
        twist.header.stamp = rospy.Time.now()
        twist.header.frame_id = "front_base"
        twist.twist.twist = msg
        twist.twist.twist.linear.z = 0
        twist.twist.twist.angular.x = 0
        twist.twist.twist.angular.y = 0
        twist.twist.covariance = [1e-3, 0, 0, 0, 0, 0,
                                0, 1e-3, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 1e3]
        self.twist_pub.publish(twist)

if __name__ == '__main__':
    try:
        TwistPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass