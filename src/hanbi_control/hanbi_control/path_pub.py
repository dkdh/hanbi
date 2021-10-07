import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
from ssafy_msgs.msg import TurtlebotStatus

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
params = {
    "MAP" : {
        "RESOLUTION": 0.1,
        "OCCUPANCY_UP": 0.02,
        "OCCUPANCY_DOWN": 0.01,
        "CENTER": (0, 0),
        "SIZE": (50, 50),
        "FILENAME": 'test.png',
        "MAPVIS_RESIZE_SCALE": 2.0,
        "PATH" : 'C:\\dev\\ros2_smart_home\\src\\hanbi_control\\map\\map_final5_.txt'
    },
    "PATH" :{
        "PATH" : 'C:\\dev\\ros2_smart_home\\src\\hanbi_control\\path\\path_final5_.txt'
    }
}
params["MAP"]["origin"] = [params["MAP"]["CENTER"][0]-(params["MAP"]["SIZE"][0]/2), params["MAP"]["CENTER"][1]-(params["MAP"]["SIZE"][1]/2)]
def grid2pose(grid):
    [x,y] = grid
    resol = params["MAP"]["RESOLUTION"]
    origin = params["MAP"]["origin"]
    return [float(origin[0] + x * resol), float((origin[1] + y * resol))]

def pose2grid(pose):
    [x, y] = pose
    return  [int((x - params["MAP"]["origin"][0])/params["MAP"]["RESOLUTION"]),int((y - params["MAP"]["origin"][1])/params["MAP"]["RESOLUTION"])]

    pass


class pathPub(Node):

    def __init__(self):
        super().__init__('path_pub')

        # 로직 1. publisher, subscriber 만들기
        self.global_path_pub = self.create_publisher(Path, 'global_path', 10)
        self.local_path_pub = self.create_publisher(Path, 'local_path', 10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.listener_callback,10)
        self.lidar_sub= self.create_subscription(Odometry,'/odom',self.listener_callback,10)
        self.turtle_sub = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.turtle_callback,10)
        self.global_add_sub = self.create_subscription(Path, 'global_path_add', self.global_add_callback, 1)
        

        self.is_odom=False
        self.is_turtle=False

        self.odom_msg=Odometry()
        self.turtle_msg= TurtlebotStatus()

        #전역경로 메시지
        self.global_path_msg=Path()
        self.global_path_msg.header.frame_id='map'


        '''
        로직 2. 만들어 놓은 경로 데이터를 읽기 모드로 open
        '''
        # 1-1 map data가 있는 파일 경로.
        #      상대경로로 지정해봤음
        full_path= params["PATH"]["PATH"]

        # 1-2 파일 열기
        self.f=open(full_path, 'r')

        '''
        로직 3. 경로 데이터를 읽어서 Path 메시지에 데이터를 넣기
        '''
 
        lines=self.f.readlines()

        for line in lines:
            tmp=list(map(float,line.split()))
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            read_pose.pose.orientation.w=1.0
            self.global_path_msg.poses.append(read_pose)
        self.f.close()


        # 로직 4. 주기마다 실행되는 타이머함수 생성, local_path_size 설정
        time_period=0.1 
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.local_path_size=20 

        self.count=0

    def listener_callback(self,msg):
        self.is_odom=True
        self.odom_msg = msg

    def turtle_callback(self,msg):
        self.is_turtle=True
        self.turtle_msg = msg

    def timer_callback(self):

        if self.is_odom ==True :
            print("timer")

            local_path_msg=Path()
            local_path_msg.header.frame_id='/map'
            
            #get position from odom
            x=self.odom_msg.pose.pose.position.x
            y=self.odom_msg.pose.pose.position.y

            #get positoin from turtlebot_status
            # x = self.turtle_msg.twist.angular.x
            # y = self.turtle_msg.twist.angular.y
            
            # print(x,y)
            current_waypoint=-1
            '''
            로직 5. global_path 중 로봇과 가장 가까운 포인트 계산
            '''

            min_dis=float('inf')
            for i,waypoint in enumerate(self.global_path_msg.poses):
                distance=sqrt(pow(x-waypoint.pose.position.x,2)+pow(y-waypoint.pose.position.y, 2))
                if distance < min_dis:
                    min_dis=distance
                    current_waypoint=i

            # print("current_waypoint ", current_waypoint)
            
            '''
            로직 6. local_path 예외 처리
            '''
            if current_waypoint != -1 :
                if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):                 
                    for num in range(current_waypoint+1, current_waypoint + self.local_path_size):
                        tmp_pose=PoseStamped()
                        tmp_pose.pose.position.x=self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y=self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w=1.0
                        local_path_msg.poses.append(tmp_pose)    

                else :
                    for num in range(current_waypoint+1, len(self.global_path_msg.poses)):
                        tmp_pose=PoseStamped()
                        tmp_pose.pose.position.x=self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y=self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w=1.0
                        local_path_msg.poses.append(tmp_pose)
            self.local_path_pub.publish(local_path_msg)

        else:
            print("timer callback : odom : {0}, turtle : {1}".format(self.is_odom, self.is_turtle))
        # 로직 7. global_path 업데이트 주기 재설정
        if self.count%10==0 :
            self.global_path_pub.publish(self.global_path_msg)
        self.count+=1

    def global_add_callback(self,msg):
        px = msg.poses[0].pose.position.x
        py = msg.poses[0].pose.position.y
        new_global_path_msg = Path()
        new_global_path_msg.header.frame_id='map'
        min_dis = 1e9
        for i,waypoint in enumerate(self.global_path_msg.poses):
            cx = waypoint.pose.position.x
            cy = waypoint.pose.position.y
            # print("{0}, {1} / {2}, {3}".format(px, cx, py, cy))

            distance=sqrt(pow(px-cx,2)+pow(py-cy, 2))
            if distance < min_dis:
                min_dis=distance
                current_waypoint=i

        self.global_path_msg.poses = self.global_path_msg.poses[:current_waypoint] + msg.poses[:] + self.global_path_msg.poses[current_waypoint:]
        print("new_global_path_msg")
        self.global_path_pub.publish(self.global_path_msg)
            
def main(args=None):
    rclpy.init(args=args)

    path_pub = pathPub()

    rclpy.spin(path_pub)

    path_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()