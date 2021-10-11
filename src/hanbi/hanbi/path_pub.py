import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist,PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path

from math import pi,cos,sin,sqrt
import tf2_ros
import os


# path_pub 노드는 make_path 노드에서 만든 텍스트 파일을 읽어와 전역 경로(/global_path)로 사용하고, 
# 전역 경로 중 로봇과 가장 가까운 포인트를 시작점으로 실제 로봇이 경로 추종에 사용하는 경로인 지역 경로(/local_path)를 만들어주는 노드입니다.


# 노드 로직 순서
# 1. publisher, subscriber 만들기
# 2. 만들어 놓은 경로 데이터를 읽기 모드로 open
# 3. 경로 데이터를 읽어서 Path 메시지에 데이터를 넣기
# 4. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
# 5. global_path 중 로봇과 가장 가까운 포인트 계산
# 6. local_path 예외 처리
# 7. global_path 업데이트 주기 재설정


class pathPub(Node):

    def __init__(self):
        super().__init__('path_pub')

        # 로직 1. publisher, subscriber 만들기
        self.global_path_pub = self.create_publisher(Path, 'global_path', 10)
        self.local_path_pub = self.create_publisher(Path, 'local_path', 10)

        self.subscription = self.create_subscription(Odometry,'/odom',self.listener_callback,10)
        self.lidar_sub= self.create_subscription(Odometry,'/odom',self.listener_callback,10)

        self.odom_msg=Odometry()
        self.is_odom=False

        # 겹치는 전역 경로를 피하기 위해서 인덱스를 쓰겠다.
        self.index = 0

        #전역경로 메시지
        self.global_path_msg=Path()
        self.global_path_msg.header.frame_id='map'


        # 로직 2. 만들어 놓은 경로 데이터를 읽기 모드로 open

        full_path= ".\\..\\path\\path.txt"
        self.f = open(full_path, 'r')


        # 로직 3. 경로 데이터를 읽어서 Path 메시지에 데이터를 넣기
        # PoseStamped : A representation of pose in free space, composed of position and orientation.
        # 사전학습 3강 코드 참조

        lines = self.f.readlines()
        for line in lines:
            tmp = line.split()
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.y = float(tmp[1])
            
            # orientation represents an orientation in free space in quaternion form.
            # float64 x float64 y float64 z float64 w
            # They are initialized with 0
            
            # w를 1.0으로 설정해주는 것의 의미는
            # which means no change in orientation (i.e. roll pitch and yaw angles are equal to 0)
            read_pose.pose.orientation.w = 1.0
            
            self.global_path_msg.poses.append(read_pose)
        
        self.f.close()

        # 로직 4. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
        time_period=0.02 
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.local_path_size=60

        self.count=0

    def listener_callback(self,msg):
        self.is_odom=True
        self.odom_msg=msg

    def timer_callback(self):
        if self.is_odom ==True:

            local_path_msg=Path()
            local_path_msg.header.frame_id='/map'
            
            x=self.odom_msg.pose.pose.position.x
            y=self.odom_msg.pose.pose.position.y
            current_waypoint=-1

            # 로직 5. global_path 중 로봇과 가장 가까운 포인트 계산
            min_dis=float('inf')

            for i,waypoint in enumerate(self.global_path_msg.poses):
                
                # 겹치는 전역 경로를 피하기 위해서 인덱스를 쓰겠다.
                if i-10 < self.index < i+10:
                    # 피타고라스의 정리
                    distance = sqrt(pow(x-waypoint.pose.position.x,2) + pow(y-waypoint.pose.position.y,2))
                    if distance < min_dis:
                        min_dis = distance
                        current_waypoint = i

            self.index = current_waypoint

            # 로직 6. local_path 예외 처리

            # 이 값이 -1이면 남은 경로가 없다고 봐야하는 것 같다.
            if current_waypoint != -1 :
                # 예를 들면 처음 시작할 때처럼
                if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                    for num in range(current_waypoint,current_waypoint + self.local_path_size):
                        # PoseStamped : A representation of pose in free space, composed of position and orientation.
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w = 1.0
                        # 15개의 경로만 담기게 된다.
                        local_path_msg.poses.append(tmp_pose)
                else:
                    # 끝까지
                    # 경로 마지막에 가면 더 이상 지역 경로가 추가되지 않아서 멈춘다.
                    for num in range(current_waypoint,len(self.global_path_msg.poses)):
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w = 1.0
                        local_path_msg.poses.append(tmp_pose)

            self.local_path_pub.publish(local_path_msg)

        # 로직 7. global_path 업데이트 주기 재설정
        if self.count%10==0 :
            self.global_path_pub.publish(self.global_path_msg)
        self.count+=1

        
def main(args=None):
    rclpy.init(args=args)
    path_pub = pathPub()
    rclpy.spin(path_pub)
    path_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()