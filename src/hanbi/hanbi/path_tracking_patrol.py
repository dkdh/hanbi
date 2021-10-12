import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Point32,PoseStamped
# HandControl 추가
from ssafy_msgs.msg import TurtlebotStatus, HandControl
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
# 추가
from sensor_msgs.msg import LaserScan, PointCloud
from math import pi,cos,sin,sqrt,atan2
import numpy as np
# 추가
import threading
import time
from hanvi_interfaces.msg import DetectionList, Detection

class followTheCarrot(Node):

    def __init__(self):
        super().__init__('path_tracking')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.hand_control = self.create_publisher(HandControl, '/hand_control', 10)

        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.status_sub = self.create_subscription(TurtlebotStatus,'/turtlebot_status',self.status_callback,10)
        self.path_sub = self.create_subscription(Path,'/local_path',self.path_callback,10)
        # 추가
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback,10)
        self.goal_pub = self.create_publisher(PoseStamped,'/goal_pose',10)
        self.a_star_local_sub = self.create_subscription(Path, '/a_star_local_path', self.a_star_local_callback,10)

        self.hanvi_sub = self.create_subscription(DetectionList, '/hanvi_detection', self.hanvi_callback,10)
        self.people_check_sub = self.create_subscription(HandControl, '/people_check', self.people_check_callback, 10)

        self.tts_pub = self.create_publisher(PoseStamped,'/tts_signal',10)

        time_period=0.05 
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.is_odom = False
        self.is_path = False
        self.is_status = False
        # 추가
        self.is_lidar = False
        self.collision = False
        
        self.is_a_star = False
        self.a_star_flag = False
        self.is_people_check = False

        self.odom_msg = Odometry()            
        self.robot_yaw = 0.0
        self.path_msg = Path()
        self.cmd_msg = Twist()
        self.a_star_msg = Path()

        self.lfd = 0.1
        self.min_lfd = 0.1
        self.max_lfd = 3.0

        # 제어 메시지 변수 생성 
        self.hand_control_msg = HandControl()  
        self.goal_pose_msg = PoseStamped()
        self.hanvi_msg = DetectionList()
        self.people_check_msg = HandControl()
        self.tts_msg = PoseStamped()

        self.is_hanvi = False

        self.return_position = [0,0]

        self.thread_flag_1 = False
        self.thread_flag_2 = False

        self.violation_flag = False

        self.previous_object = ""

    def timer_callback(self):
        if self.is_status and self.is_odom ==True and self.is_path==True and self.is_lidar==True:
            # 순찰 경로보다 쓰레기통에 다녀오는 경로가 우선하기 때문에 앞에 적겠다.
            if self.is_a_star == True and self.a_star_flag == True:
                # 복귀할 때 자연스럽게 합류하도록
                print("A STAR 경로")
                print(len(self.a_star_msg.poses))
                if len(self.a_star_msg.poses) > 1:
                    print(11111111)
                    self.is_look_forward_point= False
                    
                    robot_pose_x=self.odom_msg.pose.pose.position.x
                    robot_pose_y=self.odom_msg.pose.pose.position.y

                    lateral_error= sqrt(pow(self.a_star_msg.poses[0].pose.position.x-robot_pose_x,2)+pow(self.a_star_msg.poses[0].pose.position.y-robot_pose_y,2))

                    self.lfd = (self.status_msg.twist.linear.x+lateral_error)*0.5
                    
                    if self.lfd < self.min_lfd :
                        self.lfd = self.min_lfd
                    if self.lfd > self.max_lfd:
                        self.lfd = self.max_lfd

                    min_dis=float('inf')
                    for num, waypoint in enumerate(self.a_star_msg.poses):

                        self.current_point=waypoint.pose.position
                        dis=sqrt(pow(self.a_star_msg.poses[0].pose.position.x-self.current_point.x,2)+pow(self.a_star_msg.poses[0].pose.position.y-self.current_point.y,2))
                        if abs(dis-self.lfd) < min_dis:
                            min_dis=abs(dis-self.lfd)
                            self.forward_point=self.current_point
                            self.is_look_forward_point=True

                    # 원상 복귀
                    self.stop_flag = False
                    
                    if self.is_look_forward_point :
                        print(22222)
                        
                        global_forward_point=[self.forward_point.x ,self.forward_point.y,1]

                        trans_matrix = np.array([
                            [cos(self.robot_yaw), -sin(self.robot_yaw), robot_pose_x],
                            [sin(self.robot_yaw), cos(self.robot_yaw), robot_pose_y],
                            [0, 0, 1],
                        ])
                        
                        det_trans_matrix=np.linalg.inv(trans_matrix)
                        local_forward_point=det_trans_matrix.dot(global_forward_point)
                        theta=-atan2(local_forward_point[1],local_forward_point[0])
                        
                        out_vel = 0.8
                        out_rad_vel = 0.8*theta

                        self.cmd_msg.linear.x=out_vel
                        self.cmd_msg.angular.z=out_rad_vel
                        self.cmd_pub.publish(self.cmd_msg)
                        return

                        print(2.5)
                
                # 밑으로는 못 내려간다.
                elif self.stop_flag == True:
                    return
                
                elif self.return_position != [0, 0]:
                    print(3333)
                    if self.previous_object == "trash" or self.previous_object == "bag": 
                        print(4444)
                        
                        if self.thread_flag_2 == False:
                            self.thread_flag_2 = True

                            self.hand_control_msg.control_mode = 3
                            
                            print("내려놓기")
                            for i in range(1000):
                                self.hand_control.publish(self.hand_control_msg)
                            # time.sleep(0.5)
                            self.thread_flag_2 == False


                            self.cmd_msg.linear.x = 0.0
                            self.cmd_msg.angular.z=0.0
                            self.cmd_pub.publish(self.cmd_msg)
                            self.return_position = [0,0]
                    
                    elif self.previous_object == "violation":
                        if self.thread_flag_2 == False:
                            self.thread_flag_2 = True

                            self.hand_control_msg.control_mode = 3
                            
                            if self.violation_flag == True:
                                for i in range(100):
                                    self.cmd_msg.linear.x = 0.0
                                    self.cmd_msg.angular.z=0.0
                                    self.cmd_pub.publish(self.cmd_msg)
                                self.tts_msg.header.frame_id = 'people'
                                self.tts_pub.publish(self.tts_msg)
                                time.sleep(0.5)
                                for i in range(100):
                                    self.cmd_msg.linear.x = 0.0
                                    self.cmd_msg.angular.z=0.0
                                    self.cmd_pub.publish(self.cmd_msg)
                                time.sleep(0.5)
                                for i in range(100):
                                    self.cmd_msg.linear.x = 0.0
                                    self.cmd_msg.angular.z=0.0
                                    self.cmd_pub.publish(self.cmd_msg)
                                self.violation_flag = False
                                time.sleep(5.5)

                            self.thread_flag_2 == False

                            self.cmd_msg.linear.x = 0.0
                            self.cmd_msg.angular.z=0.0
                            self.cmd_pub.publish(self.cmd_msg)
                            self.return_position = [0,0]

                else:
                    self.a_star_flag = False
                    print("경로 복귀")
                    return

            # path_msg에는 local_path 정보가 담긴다.
            if len(self.path_msg.poses)> 1:
                # print("기본 경로")
                self.is_look_forward_point= False
                
                # 로봇의 현재 위치를 나타내는 변수
                robot_pose_x=self.odom_msg.pose.pose.position.x
                robot_pose_y=self.odom_msg.pose.pose.position.y

                # 로봇과 로컬 패스의 가장 가까운 경로점 사이의 거리
                # 이 거리가 멀리 떨어져있다면 전방 주시 거리를 늘리려고
                lateral_error= sqrt(pow(self.path_msg.poses[0].pose.position.x-robot_pose_x,2)+pow(self.path_msg.poses[0].pose.position.y-robot_pose_y,2))
                # 로직 4. 로봇이 주어진 경로점과 떨어진 거리(lateral_error)와 로봇의 선속도를 이용해 전방주시거리 설정
                
                # 로봇의 선속도, lateral_error를 고려하여 lfd 결정
                self.lfd = (self.status_msg.twist.linear.x+lateral_error)*0.5
                
                if self.lfd < self.min_lfd :
                    self.lfd = self.min_lfd
                if self.lfd > self.max_lfd:
                    self.lfd = self.max_lfd

                min_dis=float('inf')
                # 로직 5. 전방 주시 포인트 설정
                for num, waypoint in enumerate(self.path_msg.poses):

                    self.current_point=waypoint.pose.position
                    # 전방주시거리에 가장 가깝게 있는 경로점 선택
                    dis=sqrt(pow(self.path_msg.poses[0].pose.position.x-self.current_point.x,2)+pow(self.path_msg.poses[0].pose.position.y-self.current_point.y,2))
                    if abs(dis-self.lfd) < min_dis:
                        min_dis=abs(dis-self.lfd)
                        # 경로점을 넣어준다
                        self.forward_point=self.current_point
                        self.is_look_forward_point=True
                if self.is_look_forward_point :
                    global_forward_point=[self.forward_point.x ,self.forward_point.y,1]

                    # 로직 6. 전방 주시 포인트와 로봇 헤딩과의 각도 계산

                    # (테스트) 맵에서 로봇의 위치(robot_pose_x,robot_pose_y)가 (5,5)이고, 헤딩(self.robot_yaw) 1.57 rad 일 때, 선택한 전방포인트(global_forward_point)가 (3,7)일 때
                    # 변환행렬을 구해서 전방포인트를 로봇 기준좌표계로 변환을 하면 local_forward_point가 구해지고, atan2를 이용해 선택한 점과의 각도를 구하면
                    # theta는 0.7853 rad 이 나옵니다.


                    # trans_matrix는 로봇좌표계에서 기준좌표계(Map)로 좌표변환을 하기위한 변환 행렬입니다.
                    trans_matrix = np.array([
                        [cos(self.robot_yaw), -sin(self.robot_yaw), robot_pose_x],
                        [sin(self.robot_yaw), cos(self.robot_yaw), robot_pose_y],
                        [0, 0, 1],
                    ])
                    
                    # det_tran_matrix는 trans_matrix의 역행렬로, 기준좌표계(Map)에서 로봇좌표계로 좌표변환을 하기위한 변환 행렬입니다.  
                    det_trans_matrix=np.linalg.inv(trans_matrix)
                    # local_forward_point 는 global_forward_point를 로봇좌표계로 옮겨온 결과를 저장하는 변수입니다.
                    local_forward_point=det_trans_matrix.dot(global_forward_point)
                    # theta는 로봇과 전방 주시 포인트와의 각도입니다.
                    theta=-atan2(local_forward_point[1],local_forward_point[0])
                    
                    # 로직 7. 선속도, 각속도 정하기
                    # 선속도는 1m/s로 고정함

                    # theta가 제어할 각 속도에 들어간다.
                    # 2를 곱했다. 클수록 더 빠르게 경로에 수렴한다.
                    out_vel = 0.9
                    out_rad_vel = theta

                    self.cmd_msg.linear.x=out_vel
                    self.cmd_msg.angular.z=out_rad_vel
                    
                    if self.hanvi_msg.detections:
                        if self.hanvi_msg.detections[0].name == "tent":
                            self.tts_msg.header.frame_id = 'tent'
                            self.tts_pub.publish(self.tts_msg)

                        if self.hanvi_msg.detections[0].name == "bottle":
                            self.tts_msg.header.frame_id = 'bottle'
                            self.tts_pub.publish(self.tts_msg)

                    if self.collision == True:
                        print("무언가 감지")
                        if self.hanvi_msg.detections:
                            if self.previous_object != self.hanvi_msg.detections[0].name:
                                self.previous_object = self.hanvi_msg.detections[0].name
                                self.thread_flag_1 = False
                                self.thread_flag_2 = False

                            # 쓰레기 버리기
                            if self.hanvi_msg.detections[0].name == "trash":
                                print("쓰레기 감지")
                                if self.thread_flag_1 == False:
                                    self.thread_flag_1 = True
                                    
                                    # 멈추기
                                    self.cmd_msg.linear.x = 0.0
                                    self.cmd_msg.angular.z=0.0
                                    self.cmd_pub.publish(self.cmd_msg)

                                    # 경로 만들기
                                    self.goal_pose_msg.header.frame_id = 'map'
                                    # 쓰레기통 위치
                                    self.goal_pose_msg.pose.position.x = -6.89
                                    self.goal_pose_msg.pose.position.y = 6.23
                                    self.goal_pub.publish(self.goal_pose_msg)
                                    # 돌아올 곳
                                    self.return_position = [self.path_msg.poses[-1].pose.position.x, self.path_msg.poses[-1].pose.position.y]

                                    self.hand_control_msg.control_mode = 2
                                    for i in range(1000):
                                        self.hand_control.publish(self.hand_control_msg)
                                    time.sleep(0.5)
                                    for i in range(1000):
                                        self.hand_control.publish(self.hand_control_msg)
                                    time.sleep(0.5)
                                    for i in range(1000):
                                        self.hand_control.publish(self.hand_control_msg)
                                    # 바로 preview
                                    self.hand_control_msg.control_mode = 1
                                    self.hand_control_msg.put_distance = 1.7
                                    self.hand_control_msg.put_height = -2.0
                                    for i in range(1000):
                                        self.hand_control.publish(self.hand_control_msg)
                                    print("픽업 함수 완료")
                                    print("스톱 플래그 온")
                                    self.stop_flag = True
                                    self.a_star_flag = True

                                else:
                                    return
                            # 분실물
                            elif self.hanvi_msg.detections[0].name == "bag":
                                print("가방 감지")
                                if self.thread_flag_1 == False:
                                    self.thread_flag_1 = True
                                    
                                    # 멈추기
                                    self.cmd_msg.linear.x = 0.0
                                    self.cmd_msg.angular.z=0.0
                                    self.cmd_pub.publish(self.cmd_msg)

                                    # 경로 만들기
                                    self.goal_pose_msg.header.frame_id = 'map'
                                    # 관리소 위치
                                    # 6.05 -19.73
                                    self.goal_pose_msg.pose.position.x = 5.62
                                    self.goal_pose_msg.pose.position.y = -17.95
                                    self.goal_pub.publish(self.goal_pose_msg)
                                    # 돌아올 곳
                                    self.return_position = [self.path_msg.poses[-1].pose.position.x, self.path_msg.poses[-1].pose.position.y]

                                    self.hand_control_msg.control_mode = 2
                                    for i in range(1000):
                                        self.hand_control.publish(self.hand_control_msg)
                                    time.sleep(0.5)
                                    for i in range(1000):
                                        self.hand_control.publish(self.hand_control_msg)
                                    time.sleep(0.5)
                                    for i in range(1000):
                                        self.hand_control.publish(self.hand_control_msg)
                                    # 바로 preview
                                    self.hand_control_msg.control_mode = 1
                                    self.hand_control_msg.put_distance = 1.7
                                    self.hand_control_msg.put_height = 1.2
                                    for i in range(1000):
                                        self.hand_control.publish(self.hand_control_msg)
                                    print("픽업 함수 완료")
                                    print("스톱 플래그 온")
                                    self.stop_flag = True
                                    self.a_star_flag = True


                    elif self.is_people_check == True and self.people_check_msg.control_mode == 3:
                        print("인원제한 위반")
                        if sqrt(pow(self.people_check_msg.put_distance-self.odom_msg.pose.pose.position.x,2)+pow(self.people_check_msg.put_height-self.odom_msg.pose.pose.position.y,2)) < 15.0:
                            if self.previous_object != "violation":
                                self.previous_object = "violation"
                            
                                # 경로 만들기
                                self.goal_pose_msg.header.frame_id = 'map'

                                self.goal_pose_msg.pose.position.x = (1.8*self.people_check_msg.put_distance + self.odom_msg.pose.pose.position.x) / 2.8
                                self.goal_pose_msg.pose.position.y = (1.8*self.people_check_msg.put_height + self.odom_msg.pose.pose.position.y) / 2.8
                                self.goal_pub.publish(self.goal_pose_msg)
                                # 돌아올 곳
                                self.return_position = [self.path_msg.poses[-1].pose.position.x, self.path_msg.poses[-1].pose.position.y]

                                self.violation_flag = True
                                self.a_star_flag = True
            
            else :
                print("no found forward point")
                self.cmd_msg.linear.x=0.0
                self.cmd_msg.angular.z=0.0
                self.a_star_flag = True

            self.cmd_pub.publish(self.cmd_msg)

            

    def odom_callback(self, msg):
        self.is_odom=True
        self.odom_msg=msg
        # 로직 3. Quaternion 을 euler angle 로 변환
        q=Quaternion(msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z)
        _,_,self.robot_yaw=q.to_euler()

    
    def path_callback(self, msg):
        self.is_path=True
        self.path_msg=msg


    def status_callback(self,msg):
        self.is_status=True
        self.status_msg=msg

    # 추가
    def lidar_callback(self, msg):
        self.lidar_msg = msg
        # 충돌 체크를 하려면 경로와 위치를 알고 있어야 한다.
        if self.is_path == True and self.is_odom == True:
            
            # 직교좌표계 형태로 데이터타입을 가지는 포인트 클라우드 생성
            pcd_msg = PointCloud()
            pcd_msg.header.frame_id ='map'

            pose_x = self.odom_msg.pose.pose.position.x
            pose_y = self.odom_msg.pose.pose.position.y
            theta = self.robot_yaw

            t = np.array([
                            [cos(theta), -sin(theta), pose_x],
                            [sin(theta),cos(theta), pose_y],
                            [0,0,1]
                        ])
            for angle, r in enumerate(msg.ranges):
                # angle은 0부터 359까지 있는데
                # 다른 파일에서 -1 인덱스에 0을 집어넣어서 359는 0으로 나오는 걸 주의하자
                global_point = Point32()

                if 0 < r < 12:
                    # 로컬 극좌표계를 직교좌표계로 바꾸고 글로벌로 바꾼다.
                    local_x = r*cos(angle*pi/100)
                    local_y = r*sin(angle*pi/100)
                    local_point = np.array([[local_x],[local_y],[1]])
                    global_result = t.dot(local_point)
                    global_point.x = global_result[0][0]
                    global_point.y = global_result[1][0]
                    pcd_msg.points.append(global_point)

            # 이제 로컬에 있던 라이다 점들을 글로벌로 다 가져온 상태
            # 이제 경로점과 비교해서 충돌체크할 수 있다.            
            
            # 이중 포문을 이용해서 모든 경로점과 모든 라이다 간에 거리를 본다.
            self.collision = False
            for waypoint in self.path_msg.poses[:10]:
                # 우측 전방 30도와 좌측 전방 30도
                for lidar_point in pcd_msg.points:
                    distance = sqrt(pow(waypoint.pose.position.x-lidar_point.x, 2)+pow(waypoint.pose.position.y-lidar_point.y, 2))
                    # 0.1m
                    if distance < 0.15:
                        self.collision = True

            self.is_lidar = True

    def a_star_local_callback(self, msg):
        self.is_a_star = True
        self.a_star_msg = msg

    def hanvi_callback(self,msg):
        self.is_hanvi = True
        self.hanvi_msg = msg

    def people_check_callback(self, msg):
        self.is_people_check = True
        self.people_check_msg = msg
    
def main(args=None):
    rclpy.init(args=args)

    path_tracker = followTheCarrot()
    rclpy.spin(path_tracker)

    path_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()