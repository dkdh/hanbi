import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist,Point,PoseStamped
from ssafy_msgs.msg import TurtlebotStatus, HandControl
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path
from math import pi,cos,sin,sqrt,atan2
import numpy as np

params = {
    "MAP" : {
        "RESOLUTION": 0.1,
        "OCCUPANCY_UP": 0.02,
        "OCCUPANCY_DOWN": 0.01,
        "CENTER": (0, 0),
        "SIZE": (50, 50),
        "FILENAME": 'test.png',
        "MAPVIS_RESIZE_SCALE": 2.0,
        "PATH" : 'C:\\dev\\ros2_smart_home\\src\\hanbi_control\\map\\map1005.txt'
    },
    "PATH" :{
        "PATH" : 'C:\\dev\\ros2_smart_home\\src\\hanbi_control\\path\\path_kjh.txt'
    }
}
params["MAP"]["origin"] = [params["MAP"]["CENTER"][0]-(params["MAP"]["SIZE"][0]/2), params["MAP"]["CENTER"][1]-(params["MAP"]["SIZE"][1]/2)]

def pose2grid(pose):
    [x,y] = pose
    resol = params["MAP"]["RESOLUTION"]
    origin = params["MAP"]["origin"]
    return  [int((x - origin[0])/resol),int((y - origin[1])/resol)]

    pass
def grid2pose(grid):
    [x,y] = grid
    resol = params["MAP"]["RESOLUTION"]
    origin = params["MAP"]["origin"]
    return [float(origin[0] - x * resol), float((origin[1] - y * resol))]

from math import sqrt


class path_via(Node):
    
    def __init__(self):
        super().__init__('path_via')
        self.via_point_pub= self.create_publisher(Path, 'via_point', 1)
        self.path_sub = self.create_subscription(Path, '/global_path', self.path_callback,10)
        self.is_path = False
        # self.timer = self.create_timer(4, self.timer_callback)
        self.people_check_sub = self.create_subscription(HandControl, '/people_check', self.people_check_callback,10)
        self.path_msg=Path()

        # 로직 1. 경유 지점 수신
        # self.local_path_pub = self.create_publisher(Path, 'local_path', 10)

        # 로직 2. 맵 데이터 불러오기
        f = open(params["MAP"]["PATH"], "r")

        MAP_DATA = []
        while True:
            line = list(map(int, f.readline().split()))
            if not line: break
            MAP_DATA.append(line)

    # 좌표 지정 기능 완성 전 임시 테스트용
    def timer_callback(self):
        if(not self.path_msg): return

        # 로직 1. 목적지 받기
        # 임시값 지정
        pin_point_pose = [16, -10]
        print("pin_point_pose : {0}".format(pin_point_pose))

        # 로직 2. global_path의 좌표 중 pin_point_pose와 가장 가까운 점 찾기
        min_dis = 1e9
        next_point = [-1, -1]

        for idx, way_point in enumerate(self.path_msg.poses):
            cur_point=way_point.pose.position
            # print(cur_point)
            dis=sqrt(pow(pin_point_pose[0] - cur_point.x,2)+pow(pin_point_pose[1] - cur_point.y,2))
            if  dis < min_dis :
                min_dis=dis
                min_idx = idx
                next_point=[cur_point.x, cur_point.y]

        print("next_point : {0}".format(next_point))

        # 로직 3. 그리드 좌표계 변환
        pin_point = pose2grid(pin_point_pose)
        next_point = pose2grid(next_point)
    
        print("converted : {0}, {1}".format(pin_point, next_point))

        # 로직 4. a_star로 값 전달.
        via_point_msg=Path()
        via_point_msg.header.frame_id='map'

        pin_point_pose=PoseStamped()
        pin_point_pose.pose.position.x=float(pin_point[0])
        pin_point_pose.pose.position.y=float(pin_point[1])
        pin_point_pose.pose.orientation.w=1.0
        via_point_msg.poses.append(pin_point_pose)

        next_point_pose= PoseStamped()
        next_point_pose.pose.position.x=float(next_point[0])
        next_point_pose.pose.position.y=float(next_point[1])
        next_point_pose.pose.orientation.w=1.0
        via_point_msg.poses.append(next_point_pose)

        via_point_msg.header.stamp =rclpy.clock.Clock().now().to_msg()
        self.via_point_pub.publish(via_point_msg)

    def people_check_callback(self,msg):
        print("people_callback")
        # 로직 1. 목적지 pose 좌표계로 받기
        pin_point_pose = [msg.put_distance, msg.put_height]
        print("pin_point_pose : {0}".format(pin_point_pose))

        # 로직 2. global_path의 좌표 중 pin_point_pose와 가장 가까운 점 찾기
        min_dis = 1e9
        next_point = [-1, -1]

        for idx, way_point in enumerate(self.path_msg.poses):
            cur_point=way_point.pose.position
            # print(cur_point)
            dis=sqrt(pow(pin_point_pose[0] - cur_point.x,2)+pow(pin_point_pose[1] - cur_point.y,2))
            if  dis < min_dis :
                min_dis=dis
                min_idx = idx
                next_point=[cur_point.x, cur_point.y]

        print("next_point : {0}".format(next_point))

        # 로직 3. 그리드 좌표계 변환
        pin_point = pose2grid(pin_point_pose)
        next_point = pose2grid(next_point)
    
        print("converted : {0}, {1}".format(pin_point, next_point))

        # 로직 4. a_star로 값 전달.
        via_point_msg=Path()
        via_point_msg.header.frame_id='map'

        pin_point_pose=PoseStamped()
        pin_point_pose.pose.position.x=float(pin_point[0])
        pin_point_pose.pose.position.y=float(pin_point[1])
        pin_point_pose.pose.orientation.w=1.0
        via_point_msg.poses.append(pin_point_pose)

        next_point_pose= PoseStamped()
        next_point_pose.pose.position.x=float(next_point[0])
        next_point_pose.pose.position.y=float(next_point[1])
        next_point_pose.pose.orientation.w=1.0
        via_point_msg.poses.append(next_point_pose)

        via_point_msg.header.stamp =rclpy.clock.Clock().now().to_msg()
        self.via_point_pub.publish(via_point_msg)

    def odom_callback(self, msg):
        self.is_odom=True
        self.odom_msg=msg
        '''
        로직 3. Quaternion 을 euler angle 로 변환
        ''' 
        q=Quaternion(msg.pose.pose.orientation.w,msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,msg.pose.pose.orientation.z)
        _,_,self.robot_yaw=q.to_euler()
    
    def path_callback(self, msg):
        self.is_path=True
        self.path_msg=msg


    def status_callback(self,msg):
        self.is_status=True
        self.status_msg=msg
        

        
def main(args=None):
    rclpy.init(args=args)

    via = path_via()

    rclpy.spin(via)


    via.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()