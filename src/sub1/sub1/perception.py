#!/ C:\Python37\python.exe

import enum
import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from ssafy_msgs.msg import HandControl
from nav_msgs.msg import Odometry
from squaternion import Quaternion

import torch
import time

from transform import *
import math

# image parser 노드는 이미지를 받아서 opencv 의 imshow로 윈도우창에 띄우는 역할을 합니다.
# 이를 resize나 convert를 사용하여 이미지를 원하는대로 바꿔보세요.

# 노드 로직 순서
# 1. image subscriber 생성
# 2. 카메라 콜백함수에서 compressed image 디코딩
# 3. 이미지 색 채널을 gray scale로 컨버팅
# 4. 이미지 resizing
# 5. 이미지 imshow


class IMGParser(Node):

    def __init__(self):
        super().__init__(node_name='image_convertor')

        # 로직 1. image subscriber 생성
        ## 아래와 같이 subscriber가 
        ## 미리 정의된 토픽 이름인 '/image_jpeg/compressed' 에서
        ## CompressedImage 메시지를 받도록 설정된다.
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)

        # turtlebot의 위치를 받기 위한 subscriber
        self.pos_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)

        # yolo v5
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

        self.img_bgr = None

        self.timer_period = 0.03

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.num_pub_ = self.create_publisher(HandControl, '/people_check', 1)

    def img_callback(self, msg):
        # 로직 2. 카메라 콜백함수에서 이미지를 클래스 내 변수로 저장
        ## msg.data 는 bytes로 되어 있고 이를 uint8로 바꾼 다음
        ## cv2 내의 이미지 디코딩 함수로 bgr 이미지로 바꾸세요.        

        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        '''
        로직 3. 이미지 색 채널을 gray scale로 컨버팅
        cv2. 내의 이미지 색 채널 컨터버로 bgr 색상을 gary scale로 바꾸십시오.
        '''

        # img_gray = cv2.imdecode(np_arr, cv2.IMREAD_GRAYSCALE)

        '''
        로직 4. 이미지 resizing
        cv2를 사용해서 이미지를 원하는 크기로 바꿔보십시오.
        '''

        # img_resize = cv2.resize(img_bgr, dsize=(0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)

        # 로직 5. 이미지 출력 (cv2.imshow)       
        
        # cv2.imshow("img_bgr", img_bgr)
        # cv2.imshow("img_gray", img_gray)
        # cv2.imshow("resize and gray", img_resize)       
        
        # cv2.waitKey(4)

    def odom_callback(self, msg):
        
        # print('x : {} , y : {} '.format(msg.pose.pose.position.x,msg.pose.pose.position.y))
        
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        e = Quaternion.to_euler(q)
        self.bot_theta = e[2]

        # print('e: ', e)

    def estimate_point(self, cur, point):
        cur_y = cur[0] # 240
        cur_x = cur[1] / 2 # 160
        x = point[0]
        y = point[1]

        dis = math.sqrt((cur_x - x)**2 + (cur_y - y)**2)
        r = dis / math.sqrt(500)

        rad = math.atan2((cur_x -x), (cur_y - y)) + self.bot_theta
        
        if rad > math.pi:
            diff = rad - math.pi
            rad = math.pi * -1 + diff
        elif rad < math.pi:
            diff = rad + math.pi
            rad = math.pi + diff

        x_ = self.pos_x + r*math.cos(rad)
        y_ = self.pos_y + r*math.sin(rad)

        return (x_, y_)

    def detect_social_distancing(self, img_bgr):

        self.people_msg = HandControl()

        distance_minimum = 80
        people_minimum = 3

        circle_r = int(distance_minimum/2)

        corner_points = ((0, 125), (320, 125), (0, 240), (320, 240))
        matrix, _ = compute_perspective_transform(corner_points, img_bgr.shape[1], img_bgr.shape[0], img_bgr)
        bird_view_img = cv2.resize(img_bgr, (img_bgr.shape[1], img_bgr.shape[0]), interpolation = cv2.INTER_AREA)

        # inference
        results = self.model(img_bgr)
        
        # person detect 결과만 저장
        detected = results.xyxy[0]
        detected = detected[detected[:, 5] == 0]
        detected = detected[detected[:, 4] > 0.5]

        # 좌표값만 뽑기
        boxes = detected[:, :4]

        # bounding box 그리기
        for box in boxes:
            # int형으로 변환
            xmin, ymin, xmax, ymax = map(int, box)
            # 이미지, 왼쪽상단point, 오른쪽하단point, color, thickness
            cv2.rectangle(img_bgr, (xmin,ymin),(xmax,ymax),(0,255,255), 2)
        
        # 바닥 중심점 찾기
        ground_x = torch.unsqueeze((boxes[:, 0] + boxes[:, 2])/2, 1)
        ground_y = torch.unsqueeze(boxes[:, 3], 1)
        ground_points = torch.cat([ground_x, ground_y], dim=1)

        # 변환
        transformed_downoids = compute_point_perspective_transformation(matrix, ground_points)

        # 그리기
        for i, point in enumerate(transformed_downoids):
            x, y = (int(point[0]), int(point[1]))
            # 시뮬레이터 물체가 뜨는 현상으로 인한 보정
            # if y < 0:
            #     y = 0
            #     transformed_downoids[i][1] = 0
            cv2.circle(bird_view_img, (x, y), circle_r, (0, 255, 0), 2)
            cv2.circle(bird_view_img, (x, y), 3, (0, 255, 0), -1)

        # 몇 명과 같이 있는지 리스트
        companies = [1 for i in range(0, len(transformed_downoids))]

        # 다수 사람 체크
        if len(transformed_downoids) >= people_minimum:
            for i in range(0, len(transformed_downoids)):
                for j in range(i+1, len(transformed_downoids)):
                    dis = math.sqrt( (transformed_downoids[i][0] - transformed_downoids[j][0])**2 + (transformed_downoids[i][1] - transformed_downoids[j][1])**2 )
                    if dis < int(distance_minimum):
                        companies[i] += 1
                        companies[j] += 1

        # 초기화
        violate_point = None
        self.people_msg.control_mode = 0
        
        # 위반 사례 확인 - 가장 첫 번째 위반 일행 선택
        for i, company in enumerate(companies):
            if company >= people_minimum:
                violate_point = ground_points[i]
                self.people_msg.control_mode = company
                # print('img x(320) y(240):', violate_point)
                object_point = self.estimate_point(img_bgr.shape, transformed_downoids[i])
                self.people_msg.put_distance = object_point[0]
                self.people_msg.put_height = object_point[1]
                print(object_point)
                break

        # 위반 사례 존재 시 visualize
        if violate_point is not None:
            violate_point_int = (int(violate_point[0]), int(violate_point[1]))
            # print('x: ', violate_point[0], 'y: ', violate_point[1])
            cv2.circle(img_bgr, violate_point_int, 3, (255, 0, 0), -1)

        # cv2.line(img_bgr, (0, 125), (320, 125), (0, 0, 255), 1)
        # cv2.line(img_bgr, (0, 229), (320, 229), (0, 0, 255), 1)
        cv2.imshow("person detection results", img_bgr)
        cv2.imshow("bird_view", bird_view_img)
        cv2.waitKey(1)

    def timer_callback(self):

        if self.img_bgr is not None:

            # 다수 사람 체크
            self.detect_social_distancing(self.img_bgr)

            # msg publish
            # self.people_msg.put_distance = -16.0
            # self.people_msg.put_height = -19.168
            self.num_pub_.publish(self.people_msg)

        else:
            pass

def main(args=None):

    ## 노드 초기화 : rclpy.init 은 node의 이름을 알려주는 것으로, ROS master와 통신을 가능하게 해줍니다.
    rclpy.init(args=args)

    ## 메인에 돌릴 노드 클래스 정의 
    image_parser = IMGParser()

    ## 노드 실행 : 노드를 셧다운하기 전까지 종료로부터 막아주는 역할을 합니다
    rclpy.spin(image_parser)


if __name__ == '__main__':

    main()