import numpy as np
import cv2
import rclpy
import os
from rclpy.node import Node
import time
from sensor_msgs.msg import CompressedImage, LaserScan
from ssafy_msgs.msg import BBox
from hanvi_interfaces.msg import DetectionList, Detection

from nav_msgs.msg import Odometry
from sub1.transform import *
from squaternion import Quaternion

import math
import torch

params_lidar = {
    "Range" : 90, #min & max range of lidar azimuths
    "CHANNEL" : int(1), #verticla channel of a lidar
    "localIP": "127.0.0.1",
    "localPort": 9094,
    "Block_SIZE": int(1206),
    "X": 0, # meter
    "Y": 0,
    "Z": 0.6,
    "YAW": 0, # deg
    "PITCH": 0,
    "ROLL": 0
}


params_cam = {
    "WIDTH": 320, # image width
    "HEIGHT": 240, # image height
    "FOV": 60, # Field of view
    "localIP": "127.0.0.1",
    "localPort": 1232,
    "Block_SIZE": int(65000),
    "X": 0, # meter
    "Y": 0,
    "Z": 1,
    "YAW": 0, # deg
    "PITCH": 5,
    "ROLL": 0
}

# 카메라 콜백함수
def img_callback(msg):

    # 시뮬 카메라로 들어온 화면 이미지 변수
    global img_bgr

    np_arr = np.frombuffer(msg.data, np.uint8)
    img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)


# 라이다 콜백함수
def scan_callback(msg):

    # 라이다 좌표 배열
    global xyz

    R = np.array(msg.ranges)

    x = R*np.cos(np.linspace(0, 2*np.pi, 360))
    y = R*np.sin(np.linspace(0, 2*np.pi, 360))
    z = np.zeros_like(x)

    xyz = np.concatenate([
        x.reshape([-1, 1]),
        y.reshape([-1, 1]),
        z.reshape([-1, 1])
    ], axis=1)

# odom 콜백함수
def odom_callback(msg):
    global pos_x, pos_y, bot_theta

    pos_x = msg.pose.pose.position.x
    pos_y = msg.pose.pose.position.y
    # print('x, y:', pos_x, pos_y)

    q = msg.pose.pose.orientation
    e = Quaternion.to_euler(q)
    bot_theta = e[2]
    # print('bot_theta:', bot_theta)

def estimate_point(cur, point):
    cur_y = cur[0] # 240
    # 물체 뜨는 것 보정
    cur_y += 20
    cur_x = cur[1] / 2 # 160
    x = point[0]
    y = point[1]

    dis = math.sqrt((cur_x - x)**2 + (cur_y - y)**2)
    r = (dis / math.sqrt(500)) / 2

    rad = math.atan2((cur_x -x), (cur_y - y)) + bot_theta
    
    if rad > math.pi:
        rad -= 2*math.pi
    elif rad < math.pi:
        rad += 2*math.pi

    x_ = pos_x + r*math.cos(rad)
    y_ = pos_y + r*math.sin(rad)

    return (x_, y_)

def main(args=None):
    # Pytorch Hub를 통한 Yolo v5 이용. weight로 학습시킨 hanvi_detection_weight.pt 이용.
    # model = torch.hub.load('ultralytics/yolov5', 'custom', path='C:\\Users\\multicampus\\Desktop\\catkin_ws\\src\\ros2_smart_home\\sub3\\models\\hanvi_detection_weight.pt')  # 절대 경로 예시
    model = torch.hub.load('ultralytics/yolov5', 'custom', path='..\\models\\hanvi_detection_weight.pt')  # 상대 경로
    global g_node

    rclpy.init(args=args)

    g_node = rclpy.create_node('pytorch_detector')

    subscription_odom = g_node.create_subscription(Odometry, '/odom', odom_callback, 10)
    subscription_img = g_node.create_subscription(CompressedImage, '/image_jpeg/compressed', img_callback, 3)
    subscription_scan = g_node.create_subscription(LaserScan, '/scan', scan_callback, 3)
    
    publisher_detect = g_node.create_publisher(DetectionList, '/hanvi_detection', 3)

    time.sleep(1)

    while rclpy.ok():
        time.sleep(0.05)
        
        for _ in range(2):
            rclpy.spin_once(g_node)

        # Yolo v5 모델을 통한 추론.
        results = model(img_bgr)

        # print(results.pandas().xyxy[0])
        
        global_detect = DetectionList()

        # transform
        corner_points = ((0, 125), (320, 125), (0, 240), (320, 240))
        matrix, _ = compute_perspective_transform(corner_points, img_bgr.shape[1], img_bgr.shape[0], img_bgr)
        boxes = results.xyxy[0][:, :4]

        # 바닥 중심점 찾기
        ground_x = torch.unsqueeze((boxes[:, 0] + boxes[:, 2])/2, 1)
        ground_y = torch.unsqueeze(boxes[:, 3], 1)
        ground_points = torch.cat([ground_x, ground_y], dim=1)

        # 변환
        transformed_downoids = compute_point_perspective_transformation(matrix, ground_points)

        for i in range(len(results.pandas().xyxy[0])):
            local_detect = Detection()

            if results.pandas().xyxy[0].confidence[i] >= 0.7:
                local_detect.confidence = results.pandas().xyxy[0].confidence[i]
                local_detect.x = float(0)
                local_detect.y = float(0)
                local_detect.x, local_detect.y = estimate_point(img_bgr.shape, transformed_downoids[i])
                local_detect.name = results.pandas().xyxy[0].name[i]
                # print('name:', local_detect.name, 'x, y:', local_detect.x, local_detect.y)
                # 일단 절대 좌표 고정
                # if local_detect.name == 'tent':
                #     local_detect.x = 7.091
                #     local_detect.y = 12.1674
                # elif local_detect.name == 'fire':
                #     local_detect.x = 19.712
                #     local_detect.y = 4.398
                # elif local_detect.name == 'kickboard':
                #     local_detect.x = 18.486
                #     local_detect.y = -2.688
                # elif local_detect.name == 'bottle':
                #     local_detect.x = 15.424
                #     local_detect.y = -12.580
            else:
                local_detect.confidence = float(0)
                local_detect.x = float(0)
                local_detect.y = float(0)
                local_detect.name = 'None'
            
            global_detect.detections.append(local_detect)
        
        publisher_detect.publish(global_detect)
            

        results.display(render=True)
        winname = 'hanvi detection'
        cv2.imshow(winname, results.imgs[0])
        cv2.waitKey(1)


    g_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':

    main()
