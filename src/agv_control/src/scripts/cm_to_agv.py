#!/usr/bin/python3
# -*- coding:utf-8 -*-
import rospy
from geometry_msgs.msg import Twist,Point,TwistWithCovarianceStamped
from agv_control.msg import mymsg
import serial
import struct

# 创建串口对象
ser = serial.Serial('/dev/ttyUSB0', 9600)


# 定义回调函数
def vel_callback(data):
    # 提取速度信息
    linear_x = data.linear.y
    linear_y = data.linear.x
    angular_z = data.angular.z

    # 将速度信息转换为CAN总线通用协议格式
    # 这里假设你已经有一个函数可以做这个转换
    msg = convert_to_can_protocol(linear_x, linear_y, angular_z,1)
    # 通过串口发送信息
    ser.write(bytes([0xcc]))
    ser.write(msg)
    ser.write(bytes([0xaa]))
    ser.write(bytes([0xee]))

def convert_to_can_protocol(linear_x, linear_y, angular_z,signal):
    global num
    Data=[0]*8
    # 将速度信息编码到Data数组中
    # 这里我们将每个速度值乘以10000（将m/s转换为0.1mm/s），然后转换为整数
    # 然后将这些整数转换为小端字节序，并存储在Data数组中
    if signal==1:
        Data[0:2] = struct.pack('<h', int(linear_x * 10000))
        Data[2:4] = struct.pack('<h', int(linear_y * 10000))
        Data[4:6] = struct.pack('<h', int(angular_z * 10000))
        Data[6:8] = struct.pack('<h', 0)
    if signal==0:
        Data[0:2] = struct.pack('<h', int(linear_x * 10))
        Data[2:4] = struct.pack('<h', int(linear_y * 10))
        Data[4:6] = struct.pack('<h', int(angular_z * 10))
        Data[6]=num
        Data[7]=1
    if signal==2:
        Data[0:2] = struct.pack('<h', int(linear_x * 10))
        Data[2:4] = struct.pack('<h', int(linear_y * 10))
        Data[4:6] = struct.pack('<h', int(angular_z * 10))
        num = num+1
        Data[6]=num
        Data[7]=2
        #data[7]=0,识别抓，=1抓放，=2识别放，3放置
    if signal==3:
        Data[0:2] = struct.pack('<h', int(linear_x * 10))
        Data[2:4] = struct.pack('<h', int(linear_y * 10))
        Data[4:6] = struct.pack('<h', int(angular_z * 10))
        Data[6]=num
        Data[7]=3
    return bytes(Data)

def position_callback(data):
    # 提取信息
    x = data.z
    y = -data.x
    z = -data.y-5
    #print(x,y,z)
    if(x == 0 and y == 0):
        msg = convert_to_can_protocol(x, y, z,0)
        print("zhuangfang")
    elif(x ==1 and y == -1):
        msg = convert_to_can_protocol(x, y, z,3)
        print("fang")
    else:
        msg = convert_to_can_protocol(x, y, z,2)
        print("shibie")
    #print(msg)
    # 通过串口发送信息
    ser.write(bytes([0xcc]))
    ser.write(msg)
    ser.write(bytes([0xbb]))
    ser.write(bytes([0xee]))

def read_serial(event):
    global vel_pub,msg_pub
    while ser.in_waiting >= 14:
        header = ser.read(1)
        if header == b'\xA5':
            data = ser.read(11)
            checksum = ser.read(1)
            footer = ser.read(1)
            if footer == b'\x5A':
                # 计算校验和
                vel_x = struct.unpack('<h', data[5:7])[0] / 10000.0
                if vel_x<1.5 and vel_x>-1.5:
                    num=data[0]
                    voltage=struct.unpack('<h', data[1:3])[0]
                    mass = struct.unpack('<h', data[3:5])[0]
                    #辅助信息
                    msg=mymsg()
                    msg.num=num
                    msg.mass=mass
                    msg.voltage=voltage
                    #速度信息
                    linear_x = struct.unpack('<h', data[5:7])[0] / 10000.0
                    linear_y = struct.unpack('<h', data[7:9])[0] / 10000.0
                    angular_z = struct.unpack('<h', data[9:11])[0] / 10000.0

                    twist_msg = TwistWithCovarianceStamped()
                    twist_msg.header.stamp = rospy.Time.now()
                    twist_msg.header.frame_id = "front_base"
                    twist_msg.twist.twist.linear.x = linear_x
                    twist_msg.twist.twist.linear.y = linear_y
                    twist_msg.twist.twist.angular.z = angular_z

                        # 添加协方差矩阵（示例值，可以根据实际情况调整）
                    covariance_matrix = [
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0
                    ]

                    twist_msg.twist.covariance = covariance_matrix
                    msg_pub.publish(msg)
                    vel_pub.publish(twist_msg)

def main():
    global vel_pub,msg_pub,num
    rospy.init_node('agv_velocity_subscriber')

    rospy.Subscriber('/cmd_vel', Twist, vel_callback)
    rospy.Subscriber('/target_arm_position', Point, position_callback)
    num=0
    # 创建发布者
    vel_pub = rospy.Publisher('/car_vel_real', TwistWithCovarianceStamped, queue_size=10)

    msg_pub = rospy.Publisher('/mass_and_vol', mymsg, queue_size=10)
    # 启动串口读取定时器
    rospy.Timer(rospy.Duration(0.05), read_serial)

    rospy.spin()

if __name__ == '__main__':
    main()