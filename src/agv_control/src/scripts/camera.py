#!/usr/bin/python3
# -*- coding:utf-8 -*-
import cv2
from ultralytics import YOLO
import numpy as np
from geometry_msgs.msg import Point
import rospy


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


cam_matrix_left = np.array([[716.4487569,0,510.266838], 
                                         [0, 715.844336375359, 354.346793027759],
                                         [0, 0, 1]])
cam_matrix_right = np.array([[717.834479544753, 0,629.400144546435], 
                                          [0, 718.364574760964, 368.842582928362],
                                          [0, 0, 1]])
distortion_l = np.array([[0.08742315,-0.073043768,-0.001820694,-0.010104924, -0.021419388]])
distortion_r = np.array([[0.053627504,0.100513118, 0.001797253,-0.002577317, -0.288523235]])
R = np.array([[0.999776623,0.000325268,-0.021132874],
                          [-0.000393399,0.999994739,-0.003219839],
                          [0.021131716,0.003227434,0.999771491]])
T = np.array([[-60.1948168610904], [0.206726585972538], [0.793068192458128]])
baseline = 59
cap = cv2.VideoCapture(0) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
model=YOLO("/home/agv/agv/src/agv_control/src/agv_arm.onnx",task='detect')
R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(cam_matrix_left, distortion_l, cam_matrix_right, distortion_r, (640, 480), R, T)
left_map1, left_map2 = cv2.initUndistortRectifyMap(cam_matrix_left, distortion_l, R1, P1, (640,480), cv2.CV_16SC2)
right_map1, right_map2 = cv2.initUndistortRectifyMap(cam_matrix_right, distortion_r, R2, P2, (640,480), cv2.CV_16SC2)
# 创建平移矩阵
M = np.float32([[1, 0, -80], [0, 1, 0]])
coordinates = []
rospy.init_node('stereo_camera', anonymous=True)
publisher = rospy.Publisher('/target_arm_position', Point, queue_size=10)

while True:
    # 读取一帧图像
    ret, frame = cap.read()
    x_left, y_left, x_right, y_right = None, None, None, None
    left=frame[:,:640]
    #left = cv2.remap(left, left_map1, left_map2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_TRANSPARENT)
    # 使用平移矩阵对图像进行平移
    #left = cv2.warpAffine(left, M, (left.shape[1], left.shape[0]))
    left=cv2.resize(left,(640,640))
    lresults = model(source=left, device='cpu', retina_masks=True, imgsz=640, conf=0.5, iou=0.5,verbose=False)
    lbbox = lresults[0].boxes.xyxy
    if not len(lbbox) == 0:
        lbbox_np = lbbox.numpy()
        x1, y1, x2, y2 = lbbox_np[0]
        x_left, y_left = (x1 + x2) / 2+80, (y1 + y2) / 2 
        cv2.rectangle(left, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
    left=cv2.resize(left,(640,480))
    cv2.imshow('left', left)
    
    right=frame[:,640:]
    right=cv2.resize(right,(640,640))
    #left = cv2.remap(right, right_map1, right_map2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_TRANSPARENT)
    rresults = model(source=right, device='cpu', retina_masks=True, imgsz=640, conf=0.5, iou=0.5,verbose=False)
    rbbox = rresults[0].boxes.xyxy
    if not len(rbbox) == 0:
        rbbox_np = rbbox.numpy()
        x1, y1, x2, y2 = rbbox_np[0]
        x_right, y_right = (x1 + x2) / 2, (y1 + y2) / 2
        cv2.rectangle(right, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
    right=cv2.resize(right,(640,480))
    cv2.imshow('right', right)

    if not None in [x_left, y_left, x_right, y_right]:
        # Compute disparity
        disparity = abs(x_left - x_right)/480*640
        # Check if disparity is zero
        if not disparity == 0:
            depth = baseline * cam_matrix_left[0, 0] / disparity

            # Compute X and Y coordinates
            X = (x_left - cam_matrix_left[0, 2]) * depth / cam_matrix_left[0, 0]
            Y = (y_left - cam_matrix_left[1, 2]) * depth / cam_matrix_left[1, 1]
            coordinates.append((X,Y,depth))
            if len(coordinates)>=20:
                publish_average_coordinates(publisher,coordinates)
                break
    # 如果按下q键，就退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头和销毁所有窗口
cap.release()
cv2.destroyAllWindows()

