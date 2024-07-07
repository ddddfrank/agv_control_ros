import rospy
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

# 存储四个固定位置的坐标和朝向
locations = {
    'A': {'x': 1.0, 'y': 2.0, 'w': 1.0},
    'B': {'x': 3.0, 'y': 4.0, 'w': 1.0},
    'C': {'x': 5.0, 'y': 6.0, 'w': 1.0},
    'D': {'x': 7.0, 'y': 8.0, 'w': 1.0},
}

def send_goal(location):
    # 创建一个SimpleActionClient，'move_base'是我们想要连接的服务器的名称
    client = SimpleActionClient('move_base', MoveBaseAction)

    # 等待服务器变得可用
    client.wait_for_server()

    # 创建一个新的目标
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'  # 目标的坐标系
    goal.target_pose.header.stamp = rospy.Time.now()  # 目标的时间戳
    goal.target_pose.pose.position.x = location['x']  # 目标的x位置（米）
    goal.target_pose.pose.position.y = location['y']  # 目标的y位置（米）
    goal.target_pose.pose.orientation.w = location['w']  # 目标的方向（四元数）

    # 发送目标
    client.send_goal(goal)

    # 等待结果
    client.wait_for_result()

def handle_request(msg):
    # 从字典中获取对应的位置和朝向
    location = locations.get(msg.data)

    if location is not None:
        # 发送目标
        send_goal(location)
    else:
        rospy.loginfo('Unknown location: %s', msg.data)

def main():
    # 初始化ROS节点
    rospy.init_node('send_goal')

    # 订阅一个话题，来接收从网页发送的请求
    rospy.Subscriber('location_request', String, handle_request)

    # 进入循环，等待消息到来
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
