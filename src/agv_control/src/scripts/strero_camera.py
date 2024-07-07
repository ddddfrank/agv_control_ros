#!/usr/bin/python3
# -*- coding:utf-8 -*-
import cv2
from ultralytics import YOLO
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import rospy
#from geometry_msgs.msg import Twist


def publish_average_coordinates(pub,coordinate):

    avg_depth = sum(depth for _, _, depth in coordinate) / len(coordinate)
    std_dev_depth = (sum((depth - avg_depth) ** 2 for _, _, depth in coordinate) / len(coordinate)) ** 0.5

    filtered_coordinates = [(X, Y, depth) for X, Y, depth in coordinate if abs(depth - avg_depth) <= 2 * std_dev_depth]

    if not len(filtered_coordinates) == 0:

        avg_X = sum(X for X, _, _ in filtered_coordinates) / len(filtered_coordinates)
        avg_Y = sum(Y for _, Y, _ in filtered_coordinates) / len(filtered_coordinates)
        avg_depth = sum(depth for _, _, depth in filtered_coordinates) / len(filtered_coordinates)

        point = Point()
        point.x = avg_X
        point.y = avg_Y
        point.z = avg_depth
        print("X:{},Y:{},D:{}",point.x,point.y,point.z)
        pub.publish(point)


cam_matrix_left = np.array([[715.710370163797,0,506.876648577778], 
                                         [0, 714.909211775145, 352.102156488718],
                                         [0, 0, 1]])
cam_matrix_right = np.array([[716.839634148517, 0,627.932433316406], 
                                          [0, 717.564024967029, 367.249525422071],
                                          [0, 0, 1]])
distortion_l = np.array([[0.089235590420666,-0.088252406451707,-0.001902103712354,-0.013036705077329,-0.000430630903048]])
distortion_r = np.array([[0.053920139206221,0.065210975052539,0.002049355832562,-0.004567318738406,-0.210629809329054]])
R = np.array([[0.999721892126114,0.000363973495712614,-0.023579777926776],
                          [-0.00045791403958677,0.999991980224757, -0.0039786682322091],
                          [0.023578140692472,0.00398835924460916,0.999714041249808]])
T = np.array([[-60.3073604483071], [0.139841698191725], [0.982852601774268]])
baseline = 59
cap = cv2.VideoCapture(0) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
model=YOLO("/home/agv/agv/src/agv_control/src/scripts/agv_arm.onnx",task='detect')
R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(cam_matrix_left, distortion_l, cam_matrix_right, distortion_r, (640, 480), R, T)
left_map1, left_map2 = cv2.initUndistortRectifyMap(cam_matrix_left, distortion_l, R1, P1, (640,480), cv2.CV_16SC2)
right_map1, right_map2 = cv2.initUndistortRectifyMap(cam_matrix_right, distortion_r, R2, P2, (640,480), cv2.CV_16SC2)
# 创建平移矩阵
M = np.float32([[1, 0, -80], [0, 1, 0]])
coordinates = []
rospy.init_node('stereo_camera', anonymous=True)
publisher = rospy.Publisher('/target_arm_position', Point, queue_size=10)
pub_right = rospy.Publisher('/image_right', CompressedImage, queue_size=1)
pub_left = rospy.Publisher('/image_left', CompressedImage, queue_size=1)
bridge = CvBridge()
#vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#vel1=Twist()
#vel1.angular.z=0.1
#vel_pub(vel1)
#vel0=Twist()
#vel0.angular.z=0
#flag=0
while True:
    # 读取一帧图像
    ret, frame = cap.read()
    x_left, y_left, x_right, y_right = None, None, None, None
    left=frame[:,:640]
    left = cv2.remap(left, left_map1, left_map2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_TRANSPARENT)
    # 使用平移矩阵对图像进行平移
    left = cv2.warpAffine(left, M, (left.shape[1], left.shape[0]))
    left=cv2.resize(left,(640,640))
    lresults = model(source=left, device='cpu', retina_masks=True, imgsz=640, conf=0.5, iou=0.5,verbose=False)
    lbbox = lresults[0].boxes.xyxy
    if not len(lbbox) == 0:
        #if flag == 0:
        #   rospy.sleep(1)
        #    vel_pub(vel0)
        #三falg=1
        lbbox_np = lbbox.numpy()
        x1, y1, x2, y2 = lbbox_np[0]
        x_left, y_left = (x1 + x2) / 2+80, (y1 + y2) / 2 
        #x_left, y_left = (x1 + x2) / 2, (y1 + y2) / 2 
        cv2.rectangle(left, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
    left=cv2.resize(left,(640,480))
    cv2.imshow('left', left)
    
    right=frame[:,640:]
    #right = cv2.remap(right, right_map1, right_map2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_TRANSPARENT)
    right=cv2.resize(right,(640,640))
    rresults = model(source=right, device='cpu', retina_masks=True, imgsz=640, conf=0.5, iou=0.5,verbose=False)
    rbbox = rresults[0].boxes.xyxy
    if not len(rbbox) == 0:
        rbbox_np = rbbox.numpy()
        x1, y1, x2, y2 = rbbox_np[0]
        x_right, y_right = (x1 + x2) / 2, (y1 + y2) / 2
        cv2.rectangle(right, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
    right=cv2.resize(right,(640,480))
    cv2.imshow('right', right)

    right_msg=bridge.cv2_to_compressed_imgmsg(right,dst_format='jpg')
    left_msg=bridge.cv2_to_compressed_imgmsg(left,dst_format='jpg')
    pub_right.publish(right_msg)
    pub_left.publish(left_msg)

    if not None in [x_left, y_left, x_right, y_right]:
        # Compute disparity
        disparity = abs(x_left - x_right)/480*640
        # Check if disparity is zero
        if not disparity == 0:
            depth = baseline * cam_matrix_left[0, 0] / disparity

            # Compute X and Y coordinates
            X = (x_left -80 - P1[0, 2]) * depth / P1[0, 0]
            #XR = (x_left -80 - P1[0, 2]) * depth / P1[0, 0]
            #XL = (x_right - cam_matrix_right[0, 2]) * depth / cam_matrix_right[0, 0]
            #X=(XR+XL)/2
            Y = (y_left - cam_matrix_left[1, 2]) * depth / cam_matrix_left[1, 1]
            coordinates.append((X,Y,depth))
            if len(coordinates)>=20:
                publish_average_coordinates(publisher,coordinates)
                #coordinates.clear()
                break
    # 如果按下q键，就退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头和销毁所有窗口
cap.release()
cv2.destroyAllWindows()


